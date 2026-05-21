"""
Persistent identity for georeferenced targets.

The detector publishes one target at a time. Without persistence, a target the
drone briefly leaves and re-acquires looks like a brand-new object. This module
keeps a registry keyed by GPS coordinate so a re-detection within MATCH_RADIUS_M
of a known target reuses its id instead of allocating a new one.

State lives in a JSON file (default: target_registry.json beside this module).
Writes are throttled — the detection loop runs at camera rate and we'd otherwise
hammer the disk for no benefit. Updates land in memory immediately; the file is
flushed at most every SAVE_THROTTLE_S, and again on flush().
"""

import json
import math
import os
import threading
import time
from pathlib import Path


# GPS + depth uncertainty floor for our setup. With non-RTK GPS (~1-3 m) and
# RealSense depth at typical operating range (~0.5-1 m), two detections of the
# same physical target sit within ~2 m of each other. Two real targets closer
# than this will collide into one id; raise it if your targets are placed
# further apart and you have RTK.
MATCH_RADIUS_M = 2.0
EARTH_RADIUS_M = 6378137.0
SAVE_THROTTLE_S = 0.5


def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres. Cheap enough to call per assignment."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


class TargetRegistry:
    def __init__(self, path, match_radius_m=MATCH_RADIUS_M):
        self._path = Path(path)
        self._radius = match_radius_m
        self._lock = threading.Lock()
        self._targets = {}  # int id -> dict
        self._next_id = 1
        self._last_save = 0.0
        self._dirty = False
        self._load()

    def _load(self):
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text())
            self._targets = {int(k): v for k, v in data.get('targets', {}).items()}
            self._next_id = int(data.get('next_id',
                                         max(self._targets, default=0) + 1))
            print(f"[Registry] Loaded {len(self._targets)} target(s) from {self._path}")
        except Exception as e:
            print(f"[Registry] Failed to load {self._path}: {e}")

    def _save_locked(self, force=False):
        """Caller must hold self._lock. Throttled unless force=True."""
        now = time.monotonic()
        if not force and now - self._last_save < SAVE_THROTTLE_S:
            return
        if not self._dirty:
            return
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(self._path.suffix + '.tmp')
            tmp.write_text(json.dumps({
                'next_id': self._next_id,
                'targets': {str(k): v for k, v in self._targets.items()},
            }, indent=2))
            os.replace(tmp, self._path)
            self._last_save = now
            self._dirty = False
        except Exception as e:
            print(f"[Registry] Failed to save {self._path}: {e}")

    def assign(self, lat, lon, wetness=None):
        """Match by GPS distance or create a new id. Returns the int id.

        On match, the stored lat/lon is updated as a running mean of all
        sightings so the estimate sharpens as more detections come in. wetness
        overwrites any prior value -- a dry target that's since been sprayed
        should reflect the current state.
        """
        with self._lock:
            best_id = None
            best_dist = self._radius
            for tid, t in self._targets.items():
                d = _haversine_m(lat, lon, t['lat'], t['lon'])
                if d < best_dist:
                    best_id = tid
                    best_dist = d

            now = time.time()
            if best_id is None:
                tid = self._next_id
                self._next_id += 1
                self._targets[tid] = {
                    'lat': lat, 'lon': lon,
                    'first_seen': now, 'last_seen': now,
                    'count': 1, 'wetness': wetness,
                }
                print(f"[Registry] New target #{tid} at "
                      f"{lat:.6f}, {lon:.6f} (wetness={wetness})")
                self._dirty = True
                self._save_locked()
                return tid

            t = self._targets[best_id]
            n = t['count']
            t['lat'] = (t['lat'] * n + lat) / (n + 1)
            t['lon'] = (t['lon'] * n + lon) / (n + 1)
            t['count'] = n + 1
            t['last_seen'] = now
            if wetness:
                t['wetness'] = wetness
            self._dirty = True
            self._save_locked()
            return best_id

    def all(self):
        """Snapshot of every known target. Safe to JSON-encode."""
        with self._lock:
            return {tid: dict(t) for tid, t in self._targets.items()}

    def clear(self):
        with self._lock:
            self._targets.clear()
            self._next_id = 1
            self._dirty = True
            self._save_locked(force=True)

    def flush(self):
        """Force the in-memory state to disk; call on shutdown."""
        with self._lock:
            self._save_locked(force=True)
