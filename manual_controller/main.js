const rgbScreenshotBtn = document.getElementById('rgb-screenshot-btn');
const depthScreenshotBtn = document.getElementById('depth-screenshot-btn');
const releasePayloadBtn = document.getElementById('release-payload-btn');
const smallPayloadBtn = document.getElementById('small-payload-btn');
const sprayWaterBtn = document.getElementById('spray-water-btn');
const depthCameraStream = document.getElementById('depth-camera-stream');
const depthCameraOverlay = document.getElementById('depth-camera-overlay');
const depthStreamStatus = document.getElementById('depth-stream-status');
const depthDistanceOverlay = document.getElementById('depth-distance-overlay');
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
    channel: 3,              // PX4 servo channel (AUX1) for MAV_CMD_DO_SET_SERVO
    pwmOn: 1900,             // PWM when ON (full on, range 1000-2000)
    pwmOff: 1500             // PWM when OFF (neutral)
};

const WHEP_URLS = {
    depth: () => `http://${API_CONFIG.host}:8889/depth/whep`,
    rgb: () => `http://${API_CONFIG.host}:8889/rgb/whep`
};
// Object Detection shares the RGB stream — overlay is drawn client-side.
const STREAM_FOR_VIEW = {
    depth:   'depth',
    rgb:     'rgb',
    circles: 'rgb',
};
let currentDepthView = 'depth';
let currentStreamKey = null;
let activeWhepPc = null;
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

if (rgbScreenshotBtn) {
    rgbScreenshotBtn.addEventListener('click', () => {
        handleScreenshot('rgb');
    });
}

if (depthScreenshotBtn) {
    depthScreenshotBtn.addEventListener('click', () => {
        handleScreenshot('depth');
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

function getCurrentMission() {
    const activePanel = document.querySelector('.mode-panel.active');

    if (!activePanel) {
        return 'mission1';
    }

    if (activePanel.id === 'mode-mission-two') {
        return 'mission2';
    }

    return 'mission1';
}

async function handleScreenshot(view) {
    const mission = getCurrentMission();

    console.log(`[SCREENSHOT] ${view} screenshot requested for ${mission}`);

    try {
        const response = await fetch(`${API_CONFIG.baseUrl}/camera/screenshot`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                view: view,
                mission: mission
            })
        });

        const data = await response.json();

        if (data.success) {
            showToast(`${view.toUpperCase()} screenshot saved: ${data.filename}`, 'success');
            console.log('[SCREENSHOT] Saved:', data.filepath);
        } else {
            showToast('Screenshot failed: ' + (data.error || 'Unknown error'), 'error');
        }

    } catch (error) {
        console.error('[SCREENSHOT] Connection error:', error);
        showToast('Cannot connect to screenshot API: ' + error.message, 'error');
    }
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

async function initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label) {
    if (!videoElement) return;

    if (activeWhepPc) {
        activeWhepPc.close();
        activeWhepPc = null;
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
        activeWhepPc = pc;

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

                if (pc === activeWhepPc) {
                    setTimeout(() => {
                        initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label);
                    }, 1000);
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

        setTimeout(() => {
            initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label);
        }, 1000);
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
        initWhepStream(depthCameraStream, depthCameraOverlay, setDepthStatus, WHEP_URLS[streamKey](), streamKey);
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
    if (e.key.toLowerCase() === 'r' && e.ctrlKey) {
        e.preventDefault();
        handleScreenshot('rgb');
    }

    if (e.key.toLowerCase() === 'd' && e.ctrlKey) {
        e.preventDefault();
        handleScreenshot('depth');
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
