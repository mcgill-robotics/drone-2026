// DOM Elements
const tabButtons = document.querySelectorAll('.tab-button');
const tabContents = document.querySelectorAll('.tab-content');
const screenshotBtn = document.getElementById('screenshot-btn');
const releasePayloadBtn = document.getElementById('release-payload-btn');
const sprayWaterBtn = document.getElementById('spray-water-btn');
const addTargetBtn = document.getElementById('add-target-btn');
const targetsList = document.getElementById('targets-list');
const targetCountBadge = document.getElementById('target-count');
const toastContainer = document.getElementById('toast-container');

let targetCount = 0;

// ===== TAB NAVIGATION =====
tabButtons.forEach(button => {
    button.addEventListener('click', () => {
        const tabName = button.getAttribute('data-tab');
        switchTab(tabName);
    });
});

function switchTab(tabName) {
    // Remove active class from all tabs and contents
    tabButtons.forEach(btn => btn.classList.remove('active'));
    tabContents.forEach(content => content.classList.remove('active'));

    // Add active class to selected tab and content
    document.querySelector(`[data-tab="${tabName}"]`).classList.add('active');
    document.getElementById(tabName).classList.add('active');
}

// ===== CONTROL BUTTON HANDLERS =====
screenshotBtn.addEventListener('click', () => {
    handleScreenshot();
});

releasePayloadBtn.addEventListener('click', () => {
    handleReleasePayload();
});

sprayWaterBtn.addEventListener('click', () => {
    handleSprayWater();
});

function handleScreenshot() {
    showToast('📸 Screenshot captured!', 'info');
    console.log('Screenshot action triggered');
    // TODO: Implement screenshot functionality
}

function handleReleasePayload() {
    showToast('📦 Payload release initiated!', 'warning');
    console.log('Release payload action triggered');
    // TODO: Implement payload release functionality
}

function handleSprayWater() {
    showToast('💧 Water spray activated!', 'success');
    console.log('Spray water action triggered');
    // TODO: Implement water spray functionality
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

// ===== INITIALIZATION =====
document.addEventListener('DOMContentLoaded', () => {
    console.log('Drone Control Panel initialized');
    showToast('System ready for operation', 'success');
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
    // W for water
    if (e.key.toLowerCase() === 'w' && e.ctrlKey) {
        e.preventDefault();
        handleSprayWater();
    }
});

// ===== PREVENT PAGE RELOAD =====
window.addEventListener('beforeunload', (e) => {
    if (targetCount > 0 || /* other unsaved state */) {
        // Optional: add confirmation before closing
    }
});
