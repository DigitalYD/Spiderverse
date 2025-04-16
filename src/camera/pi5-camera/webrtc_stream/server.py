#!/usr/bin/env python3

import argparse
import asyncio
import json
import os
import ssl
import uuid
import time
import fractions

import cv2
from aiohttp import web
from av import VideoFrame
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder, MediaRelay

ROOT = os.path.dirname(__file__)

pcs = set()
relay = MediaRelay()

class CameraVideoStreamTrack(MediaStreamTrack):
    """Camera video stream track"""

    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.last_log = time.time()
        self.frame_count = 0

    async def recv(self):
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_log > 5.0:
            fps = self.frame_count / (current_time - self.last_log)
            print(f"Camera FPS: {fps:.1f}")
            self.frame_count = 0
            self.last_log = current_time

        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame, restarting camera")
            self.cap.release()
            self.cap = cv2.VideoCapture(0)
            ret, frame = self.cap.read()
            if not ret:
                # If still failing, return a black frame
                frame = np.zeros((720, 1280, 3), np.uint8)

        # Convert from BGR to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Create VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = current_time * 1000  # milliseconds
        video_frame.time_base = fractions.Fraction(1, 1000)  # milliseconds
        
        return video_frame

async def index(request):
    content = open(os.path.join(ROOT, "templates/index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):
    content = open(os.path.join(ROOT, "templates/client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)

    def log_info(msg, *args):
        print(pc_id + " " + msg, *args)

    log_info("Created for %s", request.remote)

    # Handle shutdowns
    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        log_info("ICE connection state is %s", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        log_info("Connection state is %s", pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # Connect the video track
    video_track = CameraVideoStreamTrack()
    pc.addTrack(video_track)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def on_shutdown(app):
    # Close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC webcam server")
    parser.add_argument("--port", type=int, default=8080, help="Port for HTTP server (default: 8080)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)")
    parser.add_argument("--verbose", "-v", action="count")
    args = parser.parse_args()

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)

    web.run_app(app, host=args.host, port=args.port)
