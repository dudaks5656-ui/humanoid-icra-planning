#!/usr/bin/env python3
"""Create a deterministic 36-second projection presentation video.

The frames are the already generated, data-derived 1920x1080 PNG figures.  This
is an offline presentation artifact; it does not execute a robot or run IK.
"""

import json
import os
import subprocess
import tempfile

import cv2


WORKSPACE = "/home/openarm/humanoid_sim_ws"
PRESENTATION = os.path.join(WORKSPACE, "presentation")
OUTPUT = os.path.join(PRESENTATION, "workspace_front_right_projection_demo.mp4")
METADATA = os.path.join(PRESENTATION, "workspace_front_right_projection_demo_metadata.json")
FPS = 30
SECONDS_PER_SLIDE = 3
SLIDES = [
    "workspace_front_reference.png",
    "workspace_front_c0.png",
    "workspace_front_c1.png",
    "workspace_front_c2.png",
    "workspace_front_c3.png",
    "workspace_front_c0_vs_c3.png",
    "workspace_right_reference.png",
    "workspace_right_c0.png",
    "workspace_right_c1.png",
    "workspace_right_c2.png",
    "workspace_right_c3.png",
    "workspace_right_c0_vs_c3.png",
]


def run(command):
    return subprocess.run(command, check=True, text=True, capture_output=True)


def main():
    if os.path.exists(OUTPUT) or os.path.exists(METADATA):
        raise RuntimeError("Projection video output already exists; refusing to overwrite")
    paths = [os.path.join(PRESENTATION, name) for name in SLIDES]
    images = []
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot decode presentation figure: {path}")
        if image.shape[:2] != (1080, 1920):
            raise RuntimeError(f"Unexpected figure resolution {image.shape[:2]}: {path}")
        images.append(image)

    with tempfile.TemporaryDirectory(prefix="workspace_projection_video_", dir="/tmp") as temp:
        avi = os.path.join(temp, "projection_slides.avi")
        writer = cv2.VideoWriter(avi, cv2.VideoWriter_fourcc(*"MJPG"), FPS, (1920, 1080))
        if not writer.isOpened():
            raise RuntimeError("OpenCV MJPG staging writer is unavailable")
        for image in images:
            for _ in range(FPS * SECONDS_PER_SLIDE):
                writer.write(image)
        writer.release()
        run([
            "gst-launch-1.0", "-q", "filesrc", f"location={avi}", "!", "avidemux", "!",
            "jpegdec", "!", "videoconvert", "!", "x264enc", "speed-preset=medium",
            "bitrate=5000", "key-int-max=60", "!", "mp4mux",
            "faststart=true", "!", "filesink", f"location={OUTPUT}",
        ])

    probe = run(["gst-discoverer-1.0", OUTPUT]).stdout
    run(["gst-launch-1.0", "-q", "filesrc", f"location={OUTPUT}", "!", "decodebin", "!", "fakesink"])
    capture = cv2.VideoCapture(OUTPUT)
    decoded = 0
    while True:
        ok, _ = capture.read()
        if not ok:
            break
        decoded += 1
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    capture.release()
    expected_frames = len(images) * FPS * SECONDS_PER_SLIDE
    if decoded != expected_frames or (width, height) != (1920, 1080) or abs(fps - FPS) > 0.01:
        raise RuntimeError(
            f"Video verification failed: frames={decoded}/{expected_frames} size={width}x{height} fps={fps}"
        )
    if "H.264" not in probe or "Quicktime" not in probe:
        raise RuntimeError(f"Unexpected container/codec:\n{probe}")
    metadata = {
        "path": OUTPUT,
        "duration_seconds": expected_frames / FPS,
        "resolution": f"{width}x{height}",
        "fps": fps,
        "codec": "H.264",
        "container": "MP4/QuickTime",
        "file_size_bytes": os.path.getsize(OUTPUT),
        "decoded_frames": decoded,
        "slides": SLIDES,
        "new_ik_sampling": False,
        "trajectory_execution": False,
    }
    with open(METADATA, "x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
