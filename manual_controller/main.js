// DOM Elements
const screenshotBtn = document.getElementById('screenshot-btn');
const releasePayloadBtn = document.getElementById('release-payload-btn');
const smallPayloadBtn = document.getElementById('small-payload-btn');
const sprayWaterBtn = document.getElementById('spray-water-btn');
const depthCameraStream = document.getElementById('depth-camera-stream');
const depthCameraOverlay = document.getElementById('depth-camera-overlay');
const depthStreamStatus = document.getElementById('depth-stream-status');
const apiStatusEl = document.getElementById('api-status');
const apiStatusText = apiStatusEl ? apiStatusEl.querySelector('.status-text') : null;
const themeToggleBtn = document.getElementById('theme-toggle');
const toastContainer = document.getElementById('toast-container');

// ===== THEME (sunlight readability) =====
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
    try { localStorage.setItem('drone-theme', theme); } catch (e) { /* ignore */ }
}

(function initTheme() {
    let stored = 'sun';
    try { stored = localStorage.getItem('drone-theme') || 'sun'; } catch (e) { /* ignore */ }
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
    if (apiStatusText) apiStatusText.textContent = label;
}

function setDepthStatus(state, label) {
    if (!depthStreamStatus) return;
    depthStreamStatus.dataset.state = state;
    depthStreamStatus.textContent = label;
}

// ===== API CONFIGURATION =====
const API_CONFIG = {
    host: window.location.hostname || 'localhost',
    port: 5000,
    get baseUrl() {
        return `http://${this.host}:${this.port}`;
    }
};

// ===== SPRAY SYSTEM STATE =====
let sprayState = {
    isActive: false,
    isPushbuttonMode: true,  // Set to true for pushbutton (hold) behavior
    channel: 3,              // Servo channel
    pwmOn: 1900,            // PWM value when ON
    pwmOff: 1500            // PWM value when OFF (neutral)
};

const WHEP_URLS = {
    depth: () => `http://${API_CONFIG.host}:8889/depth/whep`,
    rgb:   () => `http://${API_CONFIG.host}:8889/rgb/whep`,
};
let currentDepthView = 'depth';
let activeWhepPc = null;

// ===== SPRAY CONTROL SYSTEM =====
/**
 * Activate spray pump via API
 */
async function activateSpray() {
    console.log('[SPRAY] activateSpray() called, attempting API call to', API_CONFIG.baseUrl);
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

        console.log('[SPRAY] API response status:', response.status);
        const data = await response.json();
        console.log('[SPRAY] API response data:', data);
        
        if (data.success) {
            sprayState.isActive = true;
            sprayWaterBtn.classList.add('active');
            console.log('[SPRAY] Activated successfully');
            return true;
        } else {
            console.error('[SPRAY] Failed to activate:', data.error);
            showToast('Spray activation failed: ' + (data.error || 'Unknown error'), 'error');
            return false;
        }
    } catch (error) {
        console.error('[SPRAY] Connection error:', error);
        showToast('Cannot connect to spray system: ' + error.message, 'error');
        return false;
    }
}

/**
 * Deactivate spray pump via API
 */
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
            console.log('[SPRAY] Deactivated successfully');
            return true;
        } else {
            console.error('[SPRAY] Failed to deactivate:', data.error);
            showToast('Spray deactivation failed: ' + (data.error || 'Unknown error'), 'error');
            return false;
        }
    } catch (error) {
        console.error('[SPRAY] Connection error:', error);
        showToast('Cannot connect to spray system: ' + error.message, 'error');
        return false;
    }
}

/**
 * Toggle spray state (for click mode)
 */
async function toggleSpray() {
    if (sprayState.isActive) {
        return await deactivateSpray();
    } else {
        return await activateSpray();
    }
}

// ===== CONTROL BUTTON HANDLERS =====
screenshotBtn.addEventListener('click', () => {
    handleScreenshot();
});

releasePayloadBtn.addEventListener('click', () => {
    handleReleasePayload();
});

smallPayloadBtn.addEventListener('click', () => {
    handleSmallPayload();
});

// Spray water button - Pushbutton mode (hold to spray)
sprayWaterBtn.addEventListener('mousedown', async (e) => {
    console.log('[SPRAY] Mousedown event triggered');
    e.preventDefault();
    if (sprayState.isPushbuttonMode) {
        if (!sprayState.isActive) {
            console.log('[SPRAY] Activating spray...');
            const success = await activateSpray();
            if (success) {
                showToast('Water spray active', 'success');
            }
        }
    }
});

sprayWaterBtn.addEventListener('mouseup', async (e) => {
    console.log('[SPRAY] Mouseup event triggered');
    e.preventDefault();
    if (sprayState.isPushbuttonMode) {
        if (sprayState.isActive) {
            console.log('[SPRAY] Deactivating spray...');
            const success = await deactivateSpray();
            if (success) {
                showToast('Water spray stopped', 'info');
            }
        }
    }
});

// Also support touch events for mobile/tablet
sprayWaterBtn.addEventListener('touchstart', async (e) => {
    e.preventDefault();
    if (sprayState.isPushbuttonMode) {
        if (!sprayState.isActive) {
            const success = await activateSpray();
            if (success) {
                showToast('Water spray active', 'success');
            }
        }
    }
});

sprayWaterBtn.addEventListener('touchend', async (e) => {
    e.preventDefault();
    if (sprayState.isPushbuttonMode) {
        if (sprayState.isActive) {
            const success = await deactivateSpray();
            if (success) {
                showToast('Water spray stopped', 'info');
            }
        }
    }
});

// Fallback click handler for click mode (if needed)
sprayWaterBtn.addEventListener('click', async (e) => {
    if (!sprayState.isPushbuttonMode) {
        e.preventDefault();
        await toggleSpray();
    }
});

function handleScreenshot() {
    showToast('Screenshot captured!', 'info');
    console.log('Screenshot action triggered');
    // TODO: Implement screenshot functionality
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
            console.error('[PAYLOAD] Connection error:', error);
            showToast('❌ Cannot connect to payload system: ' + error.message, 'error');
        });
}

function handleSmallPayload() {
    console.log('[PAYLOAD] Small payload release requested');
    fetch(`${API_CONFIG.baseUrl}/payload/small-release`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            channel: 4,
            release_pwm: 1900,
            neutral_pwm: 1500,
            pulse_seconds: 0.5
        })
    })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            console.log('[PAYLOAD] Small payload API response status:', status);
            console.log('[PAYLOAD] Small payload API response data:', data);

            if (data.success) {
                showToast('Small payload released!', 'success');
            } else {
                showToast('Small payload release failed: ' + (data.error || 'Unknown error'), 'error');
            }
        })
        .catch(error => {
            console.error('[PAYLOAD] Small payload connection error:', error);
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
        if (subtext) subtext.textContent = 'Connecting to MediaMTX…';
    }

    try {
        const pc = new RTCPeerConnection();
        activeWhepPc = pc;

        pc.ontrack = (event) => {
            videoElement.srcObject = event.streams[0];
            videoElement.play().catch(() => {});
            videoElement.onplaying = () => {
                statusSetter('connected', 'Live');
                if (overlayElement) overlayElement.style.display = 'none';
                videoElement.onplaying = null;
            };
        };

        pc.onconnectionstatechange = () => {
            if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
                statusSetter('disconnected', 'Offline');
                if (overlayElement) {
                    overlayElement.style.display = 'flex';
                    const subtext = overlayElement.querySelector('.placeholder-subtext');
                    if (subtext) subtext.textContent = `Lost ${label} stream`;
                }
                if (pc === activeWhepPc) {
                    setTimeout(() => initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label), 1000);
                }
            }
        };

        pc.addTransceiver('video', { direction: 'recvonly' });

        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        await new Promise(resolve => {
            if (pc.iceGatheringState === 'complete') { resolve(); return; }
            const timeout = setTimeout(resolve, 1000);
            pc.onicegatheringstatechange = () => {
                if (pc.iceGatheringState === 'complete') { clearTimeout(timeout); resolve(); }
            };
        });

        const response = await fetch(whepUrl, {
            method: 'POST',
            headers: { 'Content-Type': 'application/sdp' },
            body: pc.localDescription.sdp,
        });

        if (!response.ok) throw new Error(`WHEP ${response.status}`);

        const sdpAnswer = await response.text();
        await pc.setRemoteDescription({ type: 'answer', sdp: sdpAnswer });

    } catch (err) {
        console.warn(`[${label.toUpperCase()}] WHEP failed:`, err.message);
        statusSetter('disconnected', 'Offline');
        if (overlayElement) {
            overlayElement.style.display = 'flex';
            const subtext = overlayElement.querySelector('.placeholder-subtext');
            if (subtext) subtext.textContent = `Unable to connect to ${label} stream`;
        }
        setTimeout(() => initWhepStream(videoElement, overlayElement, statusSetter, whepUrl, label), 1000);
    }
}

function setDepthView(view) {
    if (!WHEP_URLS[view] || !depthCameraStream) return;

    currentDepthView = view;

    document.querySelectorAll('.view-toggle-btn[data-view]').forEach(btn => {
        const active = btn.dataset.view === view;
        btn.classList.toggle('active', active);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
    });

    initWhepStream(depthCameraStream, depthCameraOverlay, setDepthStatus, WHEP_URLS[view](), view);
}

document.querySelectorAll('.view-toggle-btn[data-view]').forEach(btn => {
    btn.addEventListener('click', () => setDepthView(btn.dataset.view));
});

// ===== TOAST NOTIFICATION SYSTEM =====
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;

    toastContainer.appendChild(toast);

    // Auto-remove toast after 3 seconds
    setTimeout(() => {
        toast.classList.add('removing');
        setTimeout(() => {
            toast.remove();
        }, 300);
    }, 3000);
}

// ===== API HEALTH CHECK =====
async function checkAPIHealth() {
    try {
        const response = await fetch(`${API_CONFIG.baseUrl}/health`, {
            method: 'GET',
            timeout: 2000
        });
        const data = await response.json();
        console.log('[API] Health check - Connected:', data.connected);
        return data.connected;
    } catch (error) {
        console.warn('[API] Health check failed:', error.message);
        return false;
    }
}

// ===== INITIALIZATION =====
document.addEventListener('DOMContentLoaded', () => {
    console.log('Drone Control Panel initialized');
    console.log('[SPRAY] Using pushbutton mode (hold to spray)');
    showToast('System ready for operation', 'success');
    setDepthView(currentDepthView);
    
    setApiStatus('connecting', 'Connecting…');
    const refreshApiStatus = (notifyOnFail) => {
        checkAPIHealth().then(connected => {
            if (connected) {
                setApiStatus('connected', 'API Online');
            } else {
                setApiStatus('disconnected', 'API Offline');
                if (notifyOnFail) {
                    showToast('Spray control API not connected. Ensure api_server.py is running on port 5000', 'warning');
                }
            }
        });
    };
    refreshApiStatus(true);
    setInterval(() => refreshApiStatus(false), 10000);
});

// ===== KEYBOARD SHORTCUTS =====
document.addEventListener('keydown', (e) => {
    // S for screenshot
    if (e.key.toLowerCase() === 's' && e.ctrlKey) {
        e.preventDefault();
        handleScreenshot();
    }
    // P for payload
    if (e.key.toLowerCase() === 'p' && e.ctrlKey) {
        e.preventDefault();
        handleReleasePayload();
    }
    // W for water (activate and hold)
    if (e.key.toLowerCase() === 'w' && e.ctrlKey) {
        e.preventDefault();
        if (sprayState.isPushbuttonMode && !sprayState.isActive) {
            activateSpray().then(success => {
                if (success) showToast('Water spray active (Ctrl+W)', 'success');
            });
        }
    }
});

document.addEventListener('keyup', (e) => {
    // Release spray on W key release
    if (e.key.toLowerCase() === 'w' && e.ctrlKey) {
        if (sprayState.isPushbuttonMode && sprayState.isActive) {
            deactivateSpray().then(success => {
                if (success) showToast('Water spray stopped', 'info');
            });
        }
    }
});

// ===== PREVENT PAGE RELOAD =====
window.addEventListener('beforeunload', (e) => {
    if (sprayState.isActive) {
        e.preventDefault();
        e.returnValue = '';
    }
});


// ===== MODE TABS =====
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
                if (active) p.removeAttribute('hidden');
                else p.setAttribute('hidden', '');
            });
        });
    });
})();
