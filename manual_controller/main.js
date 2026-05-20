// DOM Elements
const screenshotBtn = document.getElementById('screenshot-btn');
const releasePayloadBtn = document.getElementById('release-payload-btn');
const sprayWaterBtn = document.getElementById('spray-water-btn');
const addTargetBtn = document.getElementById('add-target-btn');
const targetsList = document.getElementById('targets-list');
const targetCountBadge = document.getElementById('target-count');
const toastContainer = document.getElementById('toast-container');

// ===== API CONFIGURATION =====
const API_CONFIG = {
    host: 'localhost',
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

let targetCount = 0;

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
    showToast('Payload release initiated!', 'warning');
    console.log('Release payload action triggered');
    // TODO: Implement payload release functionality
}

// ===== MAP TARGET MANAGEMENT =====
addTargetBtn.addEventListener('click', () => {
    addTarget();
});

function addTarget() {
    targetCount++;
    const targetItem = document.createElement('div');
    targetItem.className = 'target-item';
    const targetId = targetCount;

    // Generate random coordinates for demo (would come from map click in real implementation)
    const lat = (Math.random() * 180 - 90).toFixed(4);
    const lng = (Math.random() * 360 - 180).toFixed(4);

    targetItem.innerHTML = `
        <span class="target-item-coords">Target ${targetId}: ${lat}°, ${lng}°</span>
        <button class="target-item-remove" onclick="removeTarget(${targetId})">✕</button>
    `;

    targetsList.appendChild(targetItem);
    updateTargetCount();
    showToast(`Target ${targetId} added to map`, 'success');
    console.log(`Target ${targetId} added at ${lat}, ${lng}`);
}

function removeTarget(targetId) {
    const targets = targetsList.querySelectorAll('.target-item');
    targets.forEach((target, index) => {
        if (target.textContent.includes(`Target ${targetId}`)) {
            target.remove();
        }
    });
    targetCount--;
    updateTargetCount();
    showToast(`Target ${targetId} removed`, 'info');
    console.log(`Target ${targetId} removed`);
}

function updateTargetCount() {
    targetCountBadge.textContent = `Targets: ${targetsList.children.length}`;
}

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
    
    // Check API connection on load
    checkAPIHealth().then(connected => {
        if (!connected) {
            showToast('Warning: Spray control API not connected. Ensure api_server.py is running on port 5000', 'warning');
        }
    });
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
    if (targetCount > 0) {
        // Optional: add confirmation before closing
    }
});

