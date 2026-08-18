#!/usr/bin/env python3
"""Create and fully decode-verify the 35-second FK boundary presentation."""

import json
import os
import subprocess
import tempfile

import cv2


WORKSPACE = "/home/openarm/humanoid_sim_ws"
PRESENTATION = os.path.join(WORKSPACE, "presentation")
OUTPUT = os.path.join(PRESENTATION, "fk_workspace_boundary_demo.mp4")
METADATA = os.path.join(PRESENTATION, "fk_workspace_boundary_demo_metadata.json")
FPS = 30
FRAMES_PER_SLIDE = 105
SLIDES = [
    "fk_workspace_front_c0.png", "fk_workspace_front_c1.png", "fk_workspace_front_c2.png",
    "fk_workspace_front_c3.png", "fk_workspace_front_all.png",
    "fk_workspace_right_c0.png", "fk_workspace_right_c1.png", "fk_workspace_right_c2.png",
    "fk_workspace_right_c3.png", "fk_workspace_right_all.png",
]


def run(command):
    return subprocess.run(command, check=True, text=True, capture_output=True)


def main():
    if os.path.exists(OUTPUT) or os.path.exists(METADATA):
        raise RuntimeError("Refusing to overwrite FK boundary video")
    images = []
    for name in SLIDES:
        path = os.path.join(PRESENTATION, name)
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (1080, 1920):
            raise RuntimeError(f"Invalid FK presentation figure: {path}")
        images.append(image)
    with tempfile.TemporaryDirectory(prefix="fk_workspace_video_", dir="/tmp") as temp:
        avi = os.path.join(temp, "staging.avi")
        writer = cv2.VideoWriter(avi, cv2.VideoWriter_fourcc(*"MJPG"), FPS, (1920, 1080))
        if not writer.isOpened():
            raise RuntimeError("MJPG staging writer unavailable")
        for image in images:
            for _ in range(FRAMES_PER_SLIDE):
                writer.write(image)
        writer.release()
        run(["gst-launch-1.0", "-q", "filesrc", f"location={avi}", "!", "avidemux", "!",
             "jpegdec", "!", "videoconvert", "!", "x264enc", "speed-preset=medium",
             "bitrate=5000", "key-int-max=60", "!", "mp4mux", "faststart=true", "!",
             "filesink", f"location={OUTPUT}"])
    probe = run(["gst-discoverer-1.0", OUTPUT]).stdout
    run(["gst-launch-1.0", "-q", "filesrc", f"location={OUTPUT}", "!", "decodebin", "!", "fakesink"])
    capture = cv2.VideoCapture(OUTPUT)
    decoded = 0
    while True:
        ok, _ = capture.read()
        if not ok: break
        decoded += 1
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)); height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS)); capture.release()
    expected = len(images) * FRAMES_PER_SLIDE
    if decoded != expected or (width, height) != (1920, 1080) or abs(fps - FPS) > 0.01:
        raise RuntimeError(f"Video decode mismatch: {decoded}/{expected}, {width}x{height}, {fps}")
    if "H.264" not in probe or "Quicktime" not in probe:
        raise RuntimeError("Video codec/container verification failed")
    metadata = {
        "path": OUTPUT, "duration_seconds": expected / FPS, "resolution": f"{width}x{height}",
        "fps": fps, "codec": "H.264", "container": "MP4/QuickTime",
        "file_size_bytes": os.path.getsize(OUTPUT), "decoded_frames": decoded, "slides": SLIDES,
        "ik_used": False, "trajectory_execution": False,
    }
    with open(METADATA, "x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True); stream.write("\n")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
