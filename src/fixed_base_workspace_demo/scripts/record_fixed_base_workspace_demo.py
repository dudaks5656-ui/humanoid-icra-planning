#!/usr/bin/env python3
"""Record the RViz-only fixed-base workspace demo with installed GStreamer.

This script launches only robot_state_publisher, the visualization node, and
RViz.  It never contacts a controller, trajectory action, or hardware API.
"""

import argparse
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import time

WORKSPACE = pathlib.Path("/home/openarm/humanoid_sim_ws")
PRESENTATION = WORKSPACE / "presentation"
RUNTIME = PRESENTATION / "fixed_base_workspace_demo_runtime.json"


def find_rviz_window(timeout=30.0):
    deadline = time.monotonic() + timeout
    candidates = []
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["xwininfo", "-root", "-tree"], text=True, capture_output=True, check=False
        )
        if result.returncode == 0:
            candidates.clear()
            for line in result.stdout.splitlines():
                if "rviz" not in line.lower():
                    continue
                match = re.search(r"\b(0x[0-9a-fA-F]+)\b", line)
                if not match:
                    continue
                xid_hex = match.group(1)
                details = subprocess.run(
                    ["xwininfo", "-id", xid_hex], text=True, capture_output=True, check=False
                )
                width = re.search(r"Width:\s*(\d+)", details.stdout)
                height = re.search(r"Height:\s*(\d+)", details.stdout)
                abs_x = re.search(r"Absolute upper-left X:\s*(-?\d+)", details.stdout)
                abs_y = re.search(r"Absolute upper-left Y:\s*(-?\d+)", details.stdout)
                if details.returncode == 0 and width and height and abs_x and abs_y:
                    w, h = int(width.group(1)), int(height.group(1))
                    # Ignore Qt selection-owner/helper windows (typically
                    # 1x1 or 20x20) that appear before the real RViz window.
                    if w < 800 or h < 600:
                        continue
                    candidates.append((w * h, int(xid_hex, 16), w, h,
                                       int(abs_x.group(1)), int(abs_y.group(1)), line.strip()))
            if candidates:
                return max(candidates)
        time.sleep(0.2)
    raise RuntimeError("RViz window was not discoverable through xwininfo")


def video_metadata(path):
    import cv2

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV cannot open video: {path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame is None or frame.size == 0:
            raise RuntimeError(f"Empty decoded frame {decoded}: {path}")
        decoded += 1
    capture.release()
    if decoded <= 0 or fps <= 0 or width <= 0 or height <= 0:
        raise RuntimeError(f"Invalid decoded video metadata: {path}")
    if expected_frames and abs(decoded - expected_frames) > 1:
        raise RuntimeError(f"Frame decode mismatch: decoded={decoded}, declared={expected_frames}")
    return {
        "fps": fps,
        "frame_count": decoded,
        "declared_frame_count": expected_frames,
        "duration_s": decoded / fps,
        "width": width,
        "height": height,
        "file_size_bytes": path.stat().st_size,
        "all_frames_decoded": True,
    }


def extract_frame(video, seconds, output):
    import cv2

    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_MSEC, max(0.0, seconds) * 1000.0)
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None or frame.size == 0:
        raise RuntimeError(f"Cannot extract {output} at {seconds:.3f}s")
    if not cv2.imwrite(str(output), frame):
        raise RuntimeError(f"Cannot write representative frame: {output}")
    return {"path": str(output), "time_s": seconds, "width": int(frame.shape[1]), "height": int(frame.shape[0])}


def stop_process_group(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("full", "short"), required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    PRESENTATION.mkdir(parents=True, exist_ok=True)
    output = PRESENTATION / (
        "fixed_base_workspace_demo.mp4" if args.mode == "full" else "fixed_base_workspace_demo_short.mp4"
    )
    metadata_path = PRESENTATION / f"fixed_base_workspace_demo_recording_{args.mode}.json"
    launch_log = PRESENTATION / f"fixed_base_workspace_demo_{args.mode}_launch.log"
    if output.exists() and not args.overwrite:
        raise RuntimeError(f"Refusing to overwrite existing recording without --overwrite: {output}")
    if output.exists():
        output.unlink()
    # Do not let a manual scene from a previous launch seed scene_first_seen.
    # The demo recreates this atomically during its own startup.
    if RUNTIME.exists():
        RUNTIME.unlink()

    duration_scale = 1.0 if args.mode == "full" else 0.55
    recording_seconds = 64.0 if args.mode == "full" else 40.0
    frames = int(math.ceil(recording_seconds * 30.0))
    launch_command = [
        "ros2", "launch", "fixed_base_workspace_demo", "fixed_base_workspace_demo.launch.py",
        "demo_scene:=auto", f"duration_scale:={duration_scale}", "use_rviz:=true",
    ]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "42"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    # pip OpenCV exports a private Qt plugin path when imported.  Keep RViz on
    # the system Qt/xcb plugin; OpenCV is imported only after recording.
    for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR", "QT_PLUGIN_PATH"):
        if "cv2" in environment.get(key, ""):
            environment.pop(key, None)
    with launch_log.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(
            launch_command, cwd=WORKSPACE, env=environment, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
        try:
            area, xid, source_width, source_height, window_x, window_y, window_line = find_rviz_window()
            del area
            # Let the window manager finish its initial placement and give the
            # synchronized overlay one tracking cycle before capture begins.
            time.sleep(1.0)
            area, xid, source_width, source_height, window_x, window_y, window_line = find_rviz_window(timeout=5.0)
            del area
            recorder_command = [
                "gst-launch-1.0", "-e",
                "ximagesrc", f"startx={window_x}", f"starty={window_y}",
                f"endx={window_x + source_width - 1}", f"endy={window_y + source_height - 1}",
                "use-damage=false", "show-pointer=false",
                "!", "videorate", "drop-only=true", "max-rate=30",
                "!", "video/x-raw,framerate=30/1",
                "!", "identity", "sync=true", f"eos-after={frames}",
                "!", "videoconvert",
                "!", "videoscale", "add-borders=true",
                "!", "video/x-raw,width=1920,height=1080,format=I420,pixel-aspect-ratio=1/1",
                "!", "x264enc", "bitrate=8000", "speed-preset=medium", "key-int-max=60", "threads=0",
                "!", "video/x-h264,stream-format=avc,alignment=au",
                "!", "mp4mux", "faststart=true",
                "!", "filesink", f"location={output}",
            ]
            recording_start = time.monotonic()
            recorder = subprocess.Popen(recorder_command, cwd=WORKSPACE, env=environment)
            scene_first_seen = {}
            last_scene = None
            while recorder.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError(f"ROS launch exited during recording; inspect {launch_log}")
                try:
                    status = json.loads(RUNTIME.read_text(encoding="utf-8"))
                    scene = status.get("scene")
                    if scene and scene != last_scene:
                        scene_first_seen.setdefault(scene, time.monotonic() - recording_start)
                        last_scene = scene
                except (FileNotFoundError, json.JSONDecodeError):
                    pass
                time.sleep(0.05)
            if recorder.returncode != 0:
                raise RuntimeError(f"GStreamer recorder failed with code {recorder.returncode}")
        finally:
            stop_process_group(launch)

    if not output.is_file() or output.stat().st_size <= 0:
        raise RuntimeError(f"Recording is missing or empty: {output}")
    info = video_metadata(output)
    if info["width"] != 1920 or info["height"] != 1080 or abs(info["fps"] - 30.0) > 0.1:
        raise RuntimeError(f"Unexpected recording format: {info}")

    extracted = {}
    if args.mode == "full":
        desired = {
            "frame_c0.png": ("C0", 2.8),
            "frame_c3.png": ("C3", 3.8),
            "frame_combined_only.png": ("COMBINED_C3", 1.6),
        }
        for name, (scene, offset) in desired.items():
            if scene not in scene_first_seen:
                raise RuntimeError(f"Required scene was not observed while recording: {scene}")
            extracted[name] = extract_frame(output, scene_first_seen[scene] + offset, PRESENTATION / name)

    discover = subprocess.run(
        ["gst-discoverer-1.0", str(output)], text=True, capture_output=True, check=False
    )
    if discover.returncode != 0 or "H.264" not in discover.stdout or "Quicktime" not in discover.stdout:
        raise RuntimeError(f"GStreamer container/codec validation failed:\n{discover.stdout}\n{discover.stderr}")
    metadata = {
        "mode": args.mode,
        "video_path": str(output),
        "launch_command": launch_command,
        "recording_command": recorder_command,
        "rviz_window": {
            "xid": xid,
            "source_width": source_width,
            "source_height": source_height,
            "screen_x": window_x,
            "screen_y": window_y,
            "description": window_line,
        },
        "scene_first_seen_s": scene_first_seen,
        "video": info,
        "gst_discoverer": discover.stdout,
        "representative_frames": extracted,
        "trajectory_execution": False,
        "controller": False,
        "ros2_control": False,
        "hardware": False,
        "amr_motion": False,
    }
    temporary = metadata_path.with_suffix(metadata_path.suffix + ".tmp")
    temporary.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    os.replace(temporary, metadata_path)
    print(json.dumps({"status": "PASS", "output": str(output), **info}, indent=2))


if __name__ == "__main__":
    main()
