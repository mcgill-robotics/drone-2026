const screenshotBtn = document.getElementById('screenshot-btn');
const releasePayloadBtn = document.getElementById('release-payload-btn');
const smallPayloadBtn = document.getElementById('small-payload-btn');
const sprayWaterBtn = document.getElementById('spray-water-btn');
const depthCameraStream = document.getElementById('depth-camera-stream');
const depthCameraOverlay = document.getElementById('depth-camera-overlay');
const depthStreamStatus = document.getElementById('depth-stream-status');
const depthDistanceOverlay = document.getElementById('depth-distance-overlay');
const frontCameraStream = document.getElementById('front-camera-stream');
const frontCameraOverlay = document.getElementById('front-camera-overlay');
const frontStreamStatus = document.getElementById('front-stream-status');
const odOverlayCanvas = document.getElementById('od-overlay');
const apiStatusEl = document.getElementById('api-status');
const apiStatusText = apiStatusEl ? apiStatusEl.querySelector('.status-text') : null;
const themeToggleBtn = document.getElementById('theme-toggle');
const toastContainer = document.getElementById('toast-container');

function applyTheme(theme) {
    document.documentElement.dataset.theme = theme;

    if (themeToggleBtn) {
        const icon = themeToggleBtn.querySelector('.theme-toggle-icon');
        const label = themeToggleBtn.querySelector('.theme-toggle-label');

        if (theme === 'sun') {
            if (icon) icon.textContent = '☀️';
            if (label) label.textContent = 'Sun';
        } else {
            if (icon) icon.textContent = '🌙';
            if (label) label.textContent = 'Night';
        }
    }

    try {
        localStorage.setItem('drone-theme', theme);
    } catch (e) {
        // Ignore localStorage errors
    }
}

(function initTheme() {
    let stored = 'sun';

    try {
        stored = localStorage.getItem('drone-theme') || 'sun';
    } catch (e) {
        // Ignore localStorage errors
    }

    applyTheme(stored);
})();

if (themeToggleBtn) {
    themeToggleBtn.addEventListener('click', () => {
        const current = document.documentElement.dataset.theme || 'sun';
        applyTheme(current === 'sun' ? 'night' : 'sun');
    });
}

function setApiStatus(state, label) {
    if (!apiStatusEl) return;

    apiStatusEl.dataset.state = state;

    if (apiStatusText) {
        apiStatusText.textContent = label;
    }
}

function setDepthStatus(state, label) {
    if (!depthStreamStatus) return;

    depthStreamStatus.dataset.state = state;
    depthStreamStatus.textContent = label;
}

function setFrontStatus(state, label) {
    if (!frontStreamStatus) return;
    frontStreamStatus.dataset.state = state;
    frontStreamStatus.textContent = label;
}

// ===== API CONFIGURATION =====
const API_CONFIG = {
    host: window.location.hostname || 'localhost',
    port: 5000,
    get baseUrl() {
        return `http://${this.host}:${this.port}`;
    }
};

let sprayState = {
    isActive: false,
    isPushbuttonMode: true,  // Set to true for pushbutton (hold) behavior
    channel: 8,              // PX4 servo channel for MAV_CMD_DO_SET_SERVO
    pwmOn: 1900,             // PWM when ON (full on, range 1000-2000)
    pwmOff: 1500             // PWM when OFF (neutral)
};

const WHEP_URLS = {
    depth: () => `http://${API_CONFIG.host}:8889/depth/whep`,
    rgb:   () => `http://${API_CONFIG.host}:8889/rgb/whep`,
    front: () => `http://${API_CONFIG.host}:8889/rgb/whep`,
};
// Object Detection shares the RGB stream — overlay is drawn client-side.
const STREAM_FOR_VIEW = {
    depth:   'depth',
    rgb:     'rgb',
    circles: 'rgb',
};
let currentDepthView = 'depth';
let currentStreamKey = null;
const depthSession = { pc: null };
const frontSession = { pc: null };
let depthDistanceEventSource = null;
let detectionSource = null;
let detectionFrame = { width: 640, height: 480, target: null };

async function activateSpray() {
    console.log('[SPRAY] activateSpray() called');

    try {
        const response = await fetch(`${API_CONFIG.baseUrl}/spray`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                action: 'activate',
                channel: sprayState.channel,
                pwm: sprayState.pwmOn
            })
        });

        const data = await response.json();

        if (data.success) {
            sprayState.isActive = true;
            sprayWaterBtn.classList.add('active');
            return true;
        }

        showToast('Spray activation failed: ' + (data.error || 'Unknown error'), 'error');
        return false;

    } catch (error) {
        showToast('Cannot connect to spray system: ' + error.message, 'error');
        return false;
    }
}

async function deactivateSpray() {
    try {
        const response = await fetch(`${API_CONFIG.baseUrl}/spray`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                action: 'deactivate',
                channel: sprayState.channel
            })
        });

        const data = await response.json();

        if (data.success) {
            sprayState.isActive = false;
            sprayWaterBtn.classList.remove('active');
            return true;
        }

        showToast('Spray deactivation failed: ' + (data.error || 'Unknown error'), 'error');
        return false;

    } catch (error) {
        showToast('Cannot connect to spray system: ' + error.message, 'error');
        return false;
    }
}

async function toggleSpray() {
    if (sprayState.isActive) {
        return await deactivateSpray();
    }

    return await activateSpray();
}

if (screenshotBtn) {
    screenshotBtn.addEventListener('click', () => {
        handleScreenshot();
    });
}

releasePayloadBtn.addEventListener('click', () => {
    handleReleasePayload();
});

smallPayloadBtn.addEventListener('click', () => {
    handleSmallPayload();
});

sprayWaterBtn.addEventListener('mousedown', async (e) => {
    e.preventDefault();

    if (sprayState.isPushbuttonMode && !sprayState.isActive) {
        const success = await activateSpray();

        if (success) {
            showToast('Water spray active', 'success');
        }
    }
});

sprayWaterBtn.addEventListener('mouseup', async (e) => {
    e.preventDefault();

    if (sprayState.isPushbuttonMode && sprayState.isActive) {
        const success = await deactivateSpray();

        if (success) {
            showToast('Water spray stopped', 'info');
        }
    }
});

sprayWaterBtn.addEventListener('touchstart', async (e) => {
    e.preventDefault();

    if (sprayState.isPushbuttonMode && !sprayState.isActive) {
        const success = await activateSpray();

        if (success) {
            showToast('Water spray active', 'success');
        }
    }
});

sprayWaterBtn.addEventListener('touchend', async (e) => {
    e.preventDefault();

    if (sprayState.isPushbuttonMode && sprayState.isActive) {
        const success = await deactivateSpray();

        if (success) {
            showToast('Water spray stopped', 'info');
        }
    }
});

sprayWaterBtn.addEventListener('click', async (e) => {
    if (!sprayState.isPushbuttonMode) {
        e.preventDefault();
        await toggleSpray();
    }
});

function handleScreenshot() {
    console.log('[SCREENSHOT] Capture requested');
    fetch(`${API_CONFIG.baseUrl}/camera/screenshot`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            console.log('[SCREENSHOT] API response status:', status);
            console.log('[SCREENSHOT] API response data:', data);

            if (data.success) {
                const views = Object.keys(data.files || {}).join(', ');
                const folder = data.name || (data.folder || '').split('/').pop();
                showToast(`📸 Saved ${views} → ${folder}`, 'success');
                // With depth available, drop straight into the measure view so
                // the operator can click two points and read off the distance.
                if (data.measurable && data.name) {
                    openMeasure(data.name);
                } else {
                    showToast('No depth for this capture — distance measure unavailable', 'warning');
                }
            } else {
                showToast('Screenshot failed: ' + (data.error || 'Unknown error'), 'error');
            }
        })
        .catch(error => {
            console.error('[SCREENSHOT] Connection error:', error);
            showToast('Cannot connect to camera: ' + error.message, 'error');
        });
}

function handleReleasePayload() {
    console.log('[PAYLOAD] Release requested');

    fetch(`${API_CONFIG.baseUrl}/payload/release`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            channels: [6, 7],
            release_pwm: 1900,
            neutral_pwm: 1500,
            pulse_seconds: 0.5
        })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            console.log('[PAYLOAD] API response status:', status);
            console.log('[PAYLOAD] API response data:', data);

            if (data.success) {
                showToast('📦 Payload released!', 'success');
            } else {
                showToast('❌ Payload release failed: ' + (data.error || 'Unknown error'), 'error');
            }
        })
        .catch(error => {
            showToast('❌ Cannot connect to payload system: ' + error.message, 'error');
        });
}

function handleSmallPayload() {
    console.log('[PAYLOAD] Small motor toggle requested');
    // For small payloads configured as a motor (ESC), request toggle/start/stop
    // Use the unified /payload/small endpoint which supports action=start|stop|toggle
    fetch(`${API_CONFIG.baseUrl}/payload/small`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            action: 'toggle',
            channel: 4,
            pwm_on: 1900,
            neutral_pwm: 1500
        })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            console.log('[PAYLOAD] Small payload API response status:', status);
            console.log('[PAYLOAD] Small payload API response data:', data);

            if (data.success) {
                showToast('Small motor toggled!', 'success');
            } else {
                showToast('Small payload release failed: ' + (data.error || 'Unknown error'), 'error');
            }
        })
        .catch(error => {
            showToast('Cannot connect to small payload system: ' + error.message, 'error');
        });
}

async function initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label, session) {
    if (!videoElement) return;

    if (session.pc) {
        session.pc.close();
        session.pc = null;
    }

    statusSetter('connecting', 'Connecting…');

    if (overlayElement) {
        overlayElement.style.display = 'flex';

        const subtext = overlayElement.querySelector('.placeholder-subtext');

        if (subtext) {
            subtext.textContent = 'Connecting to MediaMTX…';
        }
    }

    try {
        const pc = new RTCPeerConnection();
        session.pc = pc;

        pc.ontrack = (event) => {
            videoElement.srcObject = event.streams[0];
            videoElement.play().catch(() => {});

            videoElement.onplaying = () => {
                statusSetter('connected', 'Live');

                if (overlayElement) {
                    overlayElement.style.display = 'none';
                }

                videoElement.onplaying = null;
            };
        };

        pc.onconnectionstatechange = () => {
            if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
                statusSetter('disconnected', 'Offline');

                if (overlayElement) {
                    overlayElement.style.display = 'flex';

                    const subtext = overlayElement.querySelector('.placeholder-subtext');

                    if (subtext) {
                        subtext.textContent = `Lost ${label} stream`;
                    }
                }
                if (pc === session.pc) {
                    setTimeout(() => initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label, session), 1000);
                }
            }
        };

        pc.addTransceiver('video', { direction: 'recvonly' });

        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        await new Promise(resolve => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
                return;
            }

            const timeout = setTimeout(resolve, 1000);

            pc.onicegatheringstatechange = () => {
                if (pc.iceGatheringState === 'complete') {
                    clearTimeout(timeout);
                    resolve();
                }
            };
        });

        const response = await fetch(whepUrl, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/sdp'
            },
            body: pc.localDescription.sdp
        });

        if (!response.ok) {
            throw new Error(`WHEP ${response.status}`);
        }

        const sdpAnswer = await response.text();

        await pc.setRemoteDescription({
            type: 'answer',
            sdp: sdpAnswer
        });

    } catch (err) {
        console.warn(`[${label.toUpperCase()}] WHEP failed:`, err.message);

        statusSetter('disconnected', 'Offline');

        if (overlayElement) {
            overlayElement.style.display = 'flex';

            const subtext = overlayElement.querySelector('.placeholder-subtext');

            if (subtext) {
                subtext.textContent = `Unable to connect to ${label} stream`;
            }
        }
        setTimeout(() => initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label, session), 1000);
    }
}

function setDepthView(view) {
    const streamKey = STREAM_FOR_VIEW[view];
    if (!streamKey || !depthCameraStream) return;

    currentDepthView = view;

    document.querySelectorAll('.view-toggle-btn[data-view]').forEach(btn => {
        const active = btn.dataset.view === view;

        btn.classList.toggle('active', active);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
    });

    if (currentStreamKey !== streamKey) {
        initWhepStream(depthCameraStream, depthCameraOverlay, setDepthStatus, WHEP_URLS[streamKey](), streamKey, depthSession);
        currentStreamKey = streamKey;
    }

    if (view === 'circles') {
        odOverlayCanvas.classList.add('active');
        startDetectionStream();
    } else {
        odOverlayCanvas.classList.remove('active');
        stopDetectionStream();
        clearDetectionCanvas();
    }
}

function startDetectionStream() {
    if (detectionSource) return;
    const url = `${API_CONFIG.baseUrl}/camera/detections`;
    try {
        detectionSource = new EventSource(url);
        detectionSource.onmessage = (event) => {
            try {
                detectionFrame = JSON.parse(event.data);
                drawDetections();
            } catch (e) {
                console.warn('[OD] Bad detection payload:', e);
            }
        };
        detectionSource.onerror = () => {
            console.warn('[OD] Detection stream error, will retry');
            if (detectionSource) {
                detectionSource.close();
                detectionSource = null;
            }
            if (currentDepthView === 'circles') {
                setTimeout(startDetectionStream, 1000);
            }
        };
    } catch (e) {
        console.warn('[OD] Failed to open detection stream:', e);
    }
}

function stopDetectionStream() {
    if (detectionSource) {
        detectionSource.close();
        detectionSource = null;
    }
}

function clearDetectionCanvas() {
    if (!odOverlayCanvas) return;
    const ctx = odOverlayCanvas.getContext('2d');
    ctx.clearRect(0, 0, odOverlayCanvas.width, odOverlayCanvas.height);
}

function drawDetections() {
    if (!odOverlayCanvas || currentDepthView !== 'circles') return;

    // Match canvas internal size to the video element's rendered size.
    const rect = depthCameraStream.getBoundingClientRect();
    const cw = Math.max(1, Math.round(rect.width));
    const ch = Math.max(1, Math.round(rect.height));
    if (odOverlayCanvas.width !== cw) odOverlayCanvas.width = cw;
    if (odOverlayCanvas.height !== ch) odOverlayCanvas.height = ch;

    const ctx = odOverlayCanvas.getContext('2d');
    ctx.clearRect(0, 0, cw, ch);

    const target = detectionFrame.target;

    // Always show a status pill so we can tell the overlay is rendering even
    // when the detector has not yet locked onto a target.
    ctx.font = 'bold 12px sans-serif';
    const badgeText = target ? 'OD · LOCKED' : 'OD · scanning…';
    const badgeW = Math.ceil(ctx.measureText(badgeText).width) + 16;
    const badgeH = 22;
    ctx.fillStyle = target ? 'rgba(0, 180, 60, 0.9)' : 'rgba(0, 0, 0, 0.65)';
    ctx.fillRect(8, 8, badgeW, badgeH);
    ctx.fillStyle = '#ffffff';
    ctx.fillText(badgeText, 16, 8 + 15);

    if (!target) return;

    const srcW = detectionFrame.width || 640;
    const srcH = detectionFrame.height || 480;
    // Video uses object-fit: contain — letterbox to preserve aspect ratio.
    const scale = Math.min(cw / srcW, ch / srcH);
    const offsetX = (cw - srcW * scale) / 2;
    const offsetY = (ch - srcH * scale) / 2;

    const wet = target.wetness === 'wet';
    const boxColor = wet ? '#ff8000' : '#00cc44';

    const bx = target.bbox.x * scale + offsetX;
    const by = target.bbox.y * scale + offsetY;
    const bw = target.bbox.w * scale;
    const bh = target.bbox.h * scale;

    ctx.lineWidth = 2;
    ctx.strokeStyle = boxColor;
    ctx.strokeRect(bx, by, bw, bh);

    if (target.centroid) {
        const cx = target.centroid.x * scale + offsetX;
        const cy = target.centroid.y * scale + offsetY;
        ctx.fillStyle = '#ff4040';
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fill();
    }

    const idTag = typeof target.id === 'number' ? `#${target.id} ` : '';
    const wetLabel = target.wetness ? target.wetness.toUpperCase() : 'TARGET';
    const labelLines = [`${idTag}${wetLabel}`];
    if (typeof target.distance_m === 'number') {
        const diam = typeof target.diameter_m === 'number'
            ? ` · ~${Math.round(target.diameter_m * 100)} cm`
            : '';
        labelLines.push(`${target.distance_m.toFixed(2)} m${diam}`);
    }
    if (typeof target.lat === 'number' && typeof target.lon === 'number') {
        labelLines.push(`${target.lat.toFixed(6)}, ${target.lon.toFixed(6)}`);
    }

    ctx.font = 'bold 14px sans-serif';
    const padX = 6;
    const padY = 4;
    const lineH = 16;
    const textW = Math.max(...labelLines.map(l => ctx.measureText(l).width));
    const boxW = textW + padX * 2;
    const boxH = lineH * labelLines.length + padY * 2;
    let lx = bx;
    let ly = by - boxH - 2;
    if (ly < 0) ly = by + 2;

    ctx.fillStyle = boxColor;
    ctx.fillRect(lx, ly, boxW, boxH);
    ctx.fillStyle = '#ffffff';
    labelLines.forEach((line, i) => {
        ctx.fillText(line, lx + padX, ly + padY + lineH * (i + 1) - 4);
    });
}

window.addEventListener('resize', () => {
    if (currentDepthView === 'circles') drawDetections();
});

// ===== DISTANCE MEASUREMENT =====
// Click two points on a captured frame; the server deprojects each pixel to
// camera-space using the capture's depth + intrinsics and returns the metric
// distance between them.
const measureModal = document.getElementById('measure-modal');
const measureImg = document.getElementById('measure-img');
const measureCanvas = document.getElementById('measure-canvas');
const measureReadout = document.getElementById('measure-readout');
const measureHint = document.getElementById('measure-hint');
const measureCloseBtn = document.getElementById('measure-close');
const measureResetBtn = document.getElementById('measure-reset');

let measureState = { name: null, srcW: 640, srcH: 480, points: [] };

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function openMeasure(name) {
    if (!measureModal) return;
    measureState = { name, srcW: 640, srcH: 480, points: [] };
    measureReadout.textContent = 'Click two points to measure…';
    if (measureHint) measureHint.textContent = 'Click two points on the image';
    // Show the modal first: a cached image can fire onload synchronously, and
    // clientWidth reads 0 while the element is still hidden.
    measureModal.hidden = false;
    measureImg.onload = () => {
        measureState.srcW = measureImg.naturalWidth || 640;
        measureState.srcH = measureImg.naturalHeight || 480;
        syncMeasureCanvas();
    };
    // Cache-bust so a fresh capture never shows a stale frame.
    measureImg.src = `${API_CONFIG.baseUrl}/camera/screenshots/${name}/rgb.png?t=${Date.now()}`;
}

function closeMeasure() {
    if (!measureModal) return;
    measureModal.hidden = true;
    measureState.points = [];
}

function syncMeasureCanvas() {
    if (!measureCanvas || !measureImg) return;
    // Match the drawing buffer to the image's rendered size so points land
    // exactly under the cursor and lines stay crisp.
    const w = Math.max(1, Math.round(measureImg.clientWidth));
    const h = Math.max(1, Math.round(measureImg.clientHeight));
    if (measureCanvas.width !== w) measureCanvas.width = w;
    if (measureCanvas.height !== h) measureCanvas.height = h;
    drawMeasure();
}

function measureToDisplay(p) {
    return {
        x: p.x * (measureCanvas.width / measureState.srcW),
        y: p.y * (measureCanvas.height / measureState.srcH),
    };
}

function drawMeasure() {
    if (!measureCanvas) return;
    const ctx = measureCanvas.getContext('2d');
    ctx.clearRect(0, 0, measureCanvas.width, measureCanvas.height);
    const pts = measureState.points.map(measureToDisplay);

    if (pts.length === 2) {
        ctx.strokeStyle = '#ffcc00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(pts[0].x, pts[0].y);
        ctx.lineTo(pts[1].x, pts[1].y);
        ctx.stroke();
    }

    pts.forEach((p, i) => {
        ctx.fillStyle = '#ff4040';
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 13px sans-serif';
        ctx.fillText(String(i + 1), p.x + 9, p.y - 9);
    });
}

function handleMeasureClick(e) {
    if (!measureState.name) return;
    const rect = measureCanvas.getBoundingClientRect();
    const srcX = clamp((e.clientX - rect.left) * (measureState.srcW / rect.width), 0, measureState.srcW - 1);
    const srcY = clamp((e.clientY - rect.top) * (measureState.srcH / rect.height), 0, measureState.srcH - 1);

    // A click after two points are set starts a fresh measurement.
    if (measureState.points.length >= 2) measureState.points = [];
    measureState.points.push({ x: srcX, y: srcY });
    drawMeasure();

    if (measureState.points.length === 1) {
        measureReadout.textContent = 'Click the second point…';
    } else if (measureState.points.length === 2) {
        requestMeasure();
    }
}

function requestMeasure() {
    const [p1, p2] = measureState.points;
    measureReadout.textContent = 'Measuring…';
    fetch(`${API_CONFIG.baseUrl}/camera/measure`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: measureState.name, p1, p2 })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ data }) => {
            if (data.success) {
                const cm = (data.distance_m * 100).toFixed(1);
                const d1 = data.p1 && typeof data.p1.depth_m === 'number' ? data.p1.depth_m.toFixed(2) : '?';
                const d2 = data.p2 && typeof data.p2.depth_m === 'number' ? data.p2.depth_m.toFixed(2) : '?';
                measureReadout.textContent = `Distance: ${data.distance_m.toFixed(3)} m (${cm} cm)  ·  depths ${d1} m / ${d2} m`;
            } else {
                measureReadout.textContent = '⚠ ' + (data.error || 'Measurement failed');
            }
        })
        .catch(error => {
            console.error('[MEASURE] Connection error:', error);
            measureReadout.textContent = '⚠ Cannot reach measurement API: ' + error.message;
        });
}

if (measureCanvas) measureCanvas.addEventListener('click', handleMeasureClick);
if (measureCloseBtn) measureCloseBtn.addEventListener('click', closeMeasure);
if (measureResetBtn) measureResetBtn.addEventListener('click', () => {
    measureState.points = [];
    measureReadout.textContent = 'Click two points to measure…';
    drawMeasure();
});
if (measureModal) measureModal.addEventListener('click', (e) => {
    // Click on the dim backdrop (outside the dialog) closes the modal.
    if (e.target === measureModal) closeMeasure();
});
document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && measureModal && !measureModal.hidden) closeMeasure();
});
window.addEventListener('resize', () => {
    if (measureModal && !measureModal.hidden) syncMeasureCanvas();
});

// ===== CAPTURES GALLERY =====
// Browse saved screenshots, switch between the RGB/Depth/OD views, measure
// distance (reuses the measure modal), and edit a per-capture notes.txt.
const capturesGrid = document.getElementById('captures-grid');
const capturesEmpty = document.getElementById('captures-empty');
const capturesRefreshBtn = document.getElementById('captures-refresh');
const captureDetail = document.getElementById('capture-detail');
const captureDetailTitle = document.getElementById('capture-detail-title');
const captureDetailImg = document.getElementById('capture-detail-img');
const captureMeasureBtn = document.getElementById('capture-measure-btn');
const captureNotes = document.getElementById('capture-notes');
const captureNotesSaveBtn = document.getElementById('capture-notes-save');
const captureNotesStatus = document.getElementById('capture-notes-status');

let selectedCapture = null;   // the capture object currently shown in the detail pane
let captureView = 'rgb';      // which view image the detail pane is showing

function captureFileUrl(name, file) {
    // Cache-bust so re-opening a capture after an overwrite shows fresh pixels.
    return `${API_CONFIG.baseUrl}/camera/screenshots/${name}/${file}?t=${Date.now()}`;
}

function loadCaptures() {
    if (!capturesGrid) return;
    fetch(`${API_CONFIG.baseUrl}/camera/captures`)
        .then(r => r.json())
        .then(data => renderCapturesGrid((data && data.captures) || []))
        .catch(err => {
            console.warn('[CAPTURES] load failed:', err);
            showToast('Could not load captures: ' + err.message, 'error');
        });
}

function renderCapturesGrid(captures) {
    capturesGrid.querySelectorAll('.capture-thumb').forEach(el => el.remove());
    if (!captures.length) {
        if (capturesEmpty) capturesEmpty.hidden = false;
        return;
    }
    if (capturesEmpty) capturesEmpty.hidden = true;
    captures.forEach(cap => {
        const thumbFile = cap.files.rgb || cap.files.od || cap.files.depth;
        const card = document.createElement('button');
        card.className = 'capture-thumb';
        card.dataset.name = cap.name;
        if (selectedCapture && selectedCapture.name === cap.name) card.classList.add('selected');

        const img = document.createElement('img');
        if (thumbFile) img.src = captureFileUrl(cap.name, thumbFile);
        img.alt = cap.name;

        const label = document.createElement('span');
        label.className = 'capture-thumb-label';
        label.textContent = cap.name + (cap.has_notes ? ' 📝' : '');

        card.appendChild(img);
        card.appendChild(label);
        card.addEventListener('click', () => selectCapture(cap));
        capturesGrid.appendChild(card);
    });
}

function selectCapture(cap) {
    selectedCapture = cap;
    captureView = cap.files.rgb ? 'rgb' : (cap.files.depth ? 'depth' : 'od');
    if (captureDetail) captureDetail.hidden = false;
    if (captureDetailTitle) captureDetailTitle.textContent = cap.name;
    updateCaptureViewButtons();
    showCaptureImage();

    if (captureMeasureBtn) {
        captureMeasureBtn.disabled = !cap.measurable;
        captureMeasureBtn.title = cap.measurable
            ? 'Click two points to estimate distance'
            : 'No depth saved for this capture — measurement unavailable';
    }

    capturesGrid.querySelectorAll('.capture-thumb').forEach(el => {
        el.classList.toggle('selected', el.dataset.name === cap.name);
    });

    loadCaptureNotes(cap.name);
}

function updateCaptureViewButtons() {
    document.querySelectorAll('.view-toggle-btn[data-cap-view]').forEach(btn => {
        const v = btn.dataset.capView;
        const available = !!(selectedCapture && selectedCapture.files[v]);
        btn.disabled = !available;
        const active = v === captureView;
        btn.classList.toggle('active', active);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
    });
}

function showCaptureImage() {
    if (!selectedCapture || !captureDetailImg) return;
    const file = selectedCapture.files[captureView];
    if (file) captureDetailImg.src = captureFileUrl(selectedCapture.name, file);
}

function loadCaptureNotes(name) {
    if (!captureNotes) return;
    captureNotes.value = '';
    captureNotesStatus.textContent = 'Loading…';
    fetch(`${API_CONFIG.baseUrl}/camera/captures/${name}/notes`)
        .then(r => r.json())
        .then(data => {
            captureNotes.value = (data && data.notes) || '';
            captureNotesStatus.textContent = '';
        })
        .catch(() => { captureNotesStatus.textContent = 'Could not load notes'; });
}

function saveCaptureNotes() {
    if (!selectedCapture) return;
    const name = selectedCapture.name;
    captureNotesStatus.textContent = 'Saving…';
    fetch(`${API_CONFIG.baseUrl}/camera/captures/${name}/notes`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ notes: captureNotes.value })
    })
        .then(r => r.json())
        .then(data => {
            if (data.success) {
                captureNotesStatus.textContent = 'Saved ✓';
                selectedCapture.has_notes = captureNotes.value.trim().length > 0;
                const label = capturesGrid.querySelector(`.capture-thumb[data-name="${name}"] .capture-thumb-label`);
                if (label) label.textContent = name + (selectedCapture.has_notes ? ' 📝' : '');
                setTimeout(() => { if (captureNotesStatus.textContent === 'Saved ✓') captureNotesStatus.textContent = ''; }, 2000);
            } else {
                captureNotesStatus.textContent = '⚠ ' + (data.error || 'Save failed');
            }
        })
        .catch(err => { captureNotesStatus.textContent = '⚠ ' + err.message; });
}

if (capturesRefreshBtn) capturesRefreshBtn.addEventListener('click', loadCaptures);
if (captureMeasureBtn) captureMeasureBtn.addEventListener('click', () => {
    if (selectedCapture && selectedCapture.measurable) openMeasure(selectedCapture.name);
});
if (captureNotesSaveBtn) captureNotesSaveBtn.addEventListener('click', saveCaptureNotes);
document.querySelectorAll('.view-toggle-btn[data-cap-view]').forEach(btn => {
    btn.addEventListener('click', () => {
        if (btn.disabled) return;
        captureView = btn.dataset.capView;
        updateCaptureViewButtons();
        showCaptureImage();
    });
});
const capturesTabBtn = document.getElementById('tab-captures');
if (capturesTabBtn) capturesTabBtn.addEventListener('click', loadCaptures);

document.querySelectorAll('.view-toggle-btn[data-view]').forEach(btn => {
    btn.addEventListener('click', () => {
        setDepthView(btn.dataset.view);
    });
});

function initDepthDistanceStream() {
    if (!depthDistanceOverlay) return;

    if (depthDistanceEventSource) {
        depthDistanceEventSource.close();
        depthDistanceEventSource = null;
    }

    const url = `${API_CONFIG.baseUrl}/camera/depth-distance`;
    depthDistanceEventSource = new EventSource(url);

    depthDistanceEventSource.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            depthDistanceOverlay.textContent = data.message || 'Depth distance unavailable';

            if (data.valid) {
                depthDistanceOverlay.dataset.state = 'valid';
            } else {
                depthDistanceOverlay.dataset.state = 'invalid';
            }

        } catch (error) {
            depthDistanceOverlay.textContent = 'Depth distance parse error';
            depthDistanceOverlay.dataset.state = 'invalid';
        }
    };

    depthDistanceEventSource.onerror = () => {
        depthDistanceOverlay.textContent = 'Depth distance stream offline';
        depthDistanceOverlay.dataset.state = 'invalid';
    };
}

function showToast(message, type = 'info') {
    const toast = document.createElement('div');

    toast.className = `toast ${type}`;
    toast.textContent = message;

    toastContainer.appendChild(toast);

    setTimeout(() => {
        toast.classList.add('removing');

        setTimeout(() => {
            toast.remove();
        }, 300);
    }, 3000);
}

async function checkAPIHealth() {
    try {
        const response = await fetch(`${API_CONFIG.baseUrl}/health`, {
            method: 'GET'
        });

        const data = await response.json();

        return data.connected;

    } catch (error) {
        console.warn('[API] Health check failed:', error.message);
        return false;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    console.log('Drone Control Panel initialized');
    console.log('[SPRAY] Using pushbutton mode');

    showToast('System ready for operation', 'success');
    setDepthView(currentDepthView);
    initDepthDistanceStream();
    initWhepStream(frontCameraStream, frontCameraOverlay, setFrontStatus, WHEP_URLS.front(), 'front', frontSession);

    setApiStatus('connecting', 'Connecting…');

    const refreshApiStatus = (notifyOnFail) => {
        checkAPIHealth().then(connected => {
            if (connected) {
                setApiStatus('connected', 'API Online');
            } else {
                setApiStatus('disconnected', 'API Offline');

                if (notifyOnFail) {
                    showToast('API not connected. Ensure api_server.py is running on port 5000', 'warning');
                }
            }
        });
    };

    refreshApiStatus(true);
    setInterval(() => refreshApiStatus(false), 10000);
});

document.addEventListener('keydown', (e) => {
    if (e.key.toLowerCase() === 's' && e.ctrlKey) {
        e.preventDefault();
        handleScreenshot();
    }

    if (e.key.toLowerCase() === 'p' && e.ctrlKey) {
        e.preventDefault();
        handleReleasePayload();
    }

    if (e.key.toLowerCase() === 'w' && e.ctrlKey) {
        e.preventDefault();

        if (sprayState.isPushbuttonMode && !sprayState.isActive) {
            activateSpray().then(success => {
                if (success) {
                    showToast('Water spray active (Ctrl+W)', 'success');
                }
            });
        }
    }
});

document.addEventListener('keyup', (e) => {
    if (e.key.toLowerCase() === 'w' && e.ctrlKey) {
        if (sprayState.isPushbuttonMode && sprayState.isActive) {
            deactivateSpray().then(success => {
                if (success) {
                    showToast('Water spray stopped', 'info');
                }
            });
        }
    }
});

window.addEventListener('beforeunload', (e) => {
    if (sprayState.isActive) {
        e.preventDefault();
        e.returnValue = '';
    }
});

(function initModeTabs() {
    const tabs = document.querySelectorAll('.mode-tab');
    const panels = document.querySelectorAll('.mode-panel');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            const target = tab.dataset.tab;

            tabs.forEach(t => {
                const active = t === tab;

                t.classList.toggle('active', active);
                t.setAttribute('aria-selected', active ? 'true' : 'false');
            });

            panels.forEach(p => {
                const active = p.id === `mode-${target}`;

                p.classList.toggle('active', active);

                if (active) {
                    p.removeAttribute('hidden');
                } else {
                    p.setAttribute('hidden', '');
                }
            });
        });
    });
})();
