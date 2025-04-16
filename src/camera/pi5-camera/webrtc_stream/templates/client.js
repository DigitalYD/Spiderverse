// Global variables
let pc = null;
let statsInterval = null;

// Get HTML elements
const videoElement = document.getElementById('video');
const startButton = document.getElementById('start');
const stopButton = document.getElementById('stop');
const statsDiv = document.getElementById('stats');

// Start the WebRTC connection
async function start() {
    try {
        // Create a new RTCPeerConnection
        const config = {
            iceServers: [{urls: ['stun:stun.l.google.com:19302']}]
        };
        pc = new RTCPeerConnection(config);
        
        // Handle incoming video track
        pc.addEventListener('track', (event) => {
            if (event.track.kind === 'video') {
                videoElement.srcObject = event.streams[0];
            }
        });

        // Create and send offer
        const offer = await pc.createOffer({
            offerToReceiveVideo: true
        });
        await pc.setLocalDescription(offer);

        const response = await fetch('/offer', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type
            })
        });

        const answer = await response.json();
        await pc.setRemoteDescription(answer);

        // Start collecting stats
        startStats();

        // Update button state
        startButton.disabled = true;
        stopButton.disabled = false;
    } catch (e) {
        console.error("Error starting WebRTC:", e);
        statsDiv.textContent = `Error: ${e.toString()}`;
        stop();
    }
}

// Stop the WebRTC connection
function stop() {
    // Stop stats collection
    if (statsInterval) {
        clearInterval(statsInterval);
        statsInterval = null;
    }

    // Close peer connection
    if (pc) {
        pc.close();
        pc = null;
    }

    // Clear video
    videoElement.srcObject = null;

    // Update button state
    startButton.disabled = false;
    stopButton.disabled = true;
}

// Collect and display connection stats
function startStats() {
    statsInterval = setInterval(async () => {
        if (!pc) return;
        
        const stats = await pc.getStats();
        let statsText = '';
        
        stats.forEach(report => {
            if (report.type === 'inbound-rtp' && report.kind === 'video') {
                statsText += `Resolution: ${report.frameWidth}x${report.frameHeight}\n`;
                statsText += `Frames Received: ${report.framesReceived}\n`;
                statsText += `Frames Decoded: ${report.framesDecoded}\n`;
                if (report.framesDropped !== undefined) {
                    statsText += `Frames Dropped: ${report.framesDropped}\n`;
                }
                if (report.framesPerSecond !== undefined) {
                    statsText += `FPS: ${report.framesPerSecond.toFixed(1)}\n`;
                }
                statsText += `Packets Lost: ${report.packetsLost}\n`;
                statsText += `Jitter: ${report.jitter.toFixed(3)}s\n`;
            } else if (report.type === 'transport') {
                statsText += `Bytes Received: ${(report.bytesReceived / 1024 / 1024).toFixed(2)} MB\n`;
            }
        });
        
        statsDiv.textContent = statsText;
    }, 1000);
}

// Handle page unload
window.addEventListener('beforeunload', () => {
    stop();
});
