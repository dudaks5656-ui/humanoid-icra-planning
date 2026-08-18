#!/usr/bin/env python3
"""Capture the RViz radial validation demo and decode-check every frame."""

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
RUNTIME = PRESENTATION / "radial_workspace_validation_demo_runtime.json"


def find_rviz(timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        tree = subprocess.run(["xwininfo", "-root", "-tree"], text=True, capture_output=True, check=False)
        candidates = []
        for line in tree.stdout.splitlines():
            if "rviz" not in line.lower():
                continue
            xid = re.search(r"\b(0x[0-9a-fA-F]+)\b", line)
            if not xid:
                continue
            detail = subprocess.run(["xwininfo", "-id", xid.group(1)], text=True, capture_output=True, check=False)
            values = [re.search(pattern, detail.stdout) for pattern in
                      (r"Width:\s*(\d+)", r"Height:\s*(\d+)",
                       r"Absolute upper-left X:\s*(-?\d+)", r"Absolute upper-left Y:\s*(-?\d+)")]
            if all(values):
                width, height, x, y = (int(value.group(1)) for value in values)
                if width >= 800 and height >= 600:
                    candidates.append((width * height, int(xid.group(1), 16), width, height, x, y, line.strip()))
        if candidates:
            return max(candidates)
        time.sleep(0.2)
    raise RuntimeError("RViz window was not discoverable")


def stop_group(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def decode(path):
    import cv2
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    declared = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frames = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame is None or frame.size == 0:
            raise RuntimeError(f"Empty frame {frames}")
        frames += 1
    capture.release()
    if frames <= 0 or abs(frames - declared) > 1:
        raise RuntimeError(f"Decode mismatch decoded={frames} declared={declared}")
    return {"fps": fps, "frame_count": frames, "duration_s": frames / fps,
            "width": width, "height": height, "file_size_bytes": path.stat().st_size,
            "all_frames_decoded": True}


def extract(video, seconds, output):
    import cv2
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000.0)
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None or not cv2.imwrite(str(output), frame):
        raise RuntimeError(f"Cannot extract {output}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("full", "short"), required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    PRESENTATION.mkdir(exist_ok=True)
    suffix = "" if args.mode == "full" else "_short"
    output = PRESENTATION / f"radial_workspace_validation_demo{suffix}.mp4"
    metadata_path = PRESENTATION / f"radial_workspace_validation_demo_recording_{args.mode}.json"
    log_path = PRESENTATION / f"radial_workspace_validation_demo_{args.mode}_launch.log"
    if output.exists() and not args.overwrite:
        raise RuntimeError(f"Refusing to overwrite {output}")
    if output.exists():
        output.unlink()
    if RUNTIME.exists():
        RUNTIME.unlink()
    scale = 1.0 if args.mode == "full" else 0.78
    seconds = 45.0 if args.mode == "full" else 35.0
    frames = int(math.ceil(seconds * 30))
    command = ["ros2", "launch", "radial_workspace_validation", "radial_workspace_validation_demo.launch.py",
               "demo_scene:=auto", f"duration_scale:={scale}", "use_rviz:=true", "use_overlay:=true"]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "42"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR", "QT_PLUGIN_PATH"):
        if "cv2" in environment.get(key, ""):
            environment.pop(key, None)
    with log_path.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(command, cwd=WORKSPACE, env=environment, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True, text=True)
        try:
            _, xid, width, height, x, y, description = find_rviz()
            time.sleep(1.0)
            _, xid, width, height, x, y, description = find_rviz(timeout=5.0)
            record_command = [
                "gst-launch-1.0", "-e", "ximagesrc", f"startx={x}", f"starty={y}",
                f"endx={x+width-1}", f"endy={y+height-1}", "use-damage=false", "show-pointer=false",
                "!", "videorate", "drop-only=true", "max-rate=30", "!", "video/x-raw,framerate=30/1",
                "!", "identity", "sync=true", f"eos-after={frames}", "!", "videoconvert", "!", "videoscale",
                "add-borders=true", "!", "video/x-raw,width=1920,height=1080,format=I420,pixel-aspect-ratio=1/1",
                "!", "x264enc", "bitrate=8000", "speed-preset=medium", "key-int-max=60", "threads=0",
                "!", "video/x-h264,stream-format=avc,alignment=au", "!", "mp4mux", "faststart=true",
                "!", "filesink", f"location={output}",
            ]
            started = time.monotonic()
            recorder = subprocess.Popen(record_command, cwd=WORKSPACE, env=environment)
            first_seen = {}
            while recorder.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError(f"Launch exited; inspect {log_path}")
                try:
                    scene = json.loads(RUNTIME.read_text(encoding="utf-8"))["scene"]
                    first_seen.setdefault(scene, time.monotonic() - started)
                except (FileNotFoundError, json.JSONDecodeError, KeyError):
                    pass
                time.sleep(0.05)
            if recorder.returncode:
                raise RuntimeError(f"GStreamer failed: {recorder.returncode}")
        finally:
            stop_group(launch)
    info = decode(output)
    discover = subprocess.run(["gst-discoverer-1.0", str(output)], text=True, capture_output=True, check=False)
    if info["width"] != 1920 or info["height"] != 1080 or "H.264" not in discover.stdout:
        raise RuntimeError(f"Video metadata validation failed: {info}\n{discover.stdout}")
    if args.mode == "full":
        desired = {
            "radial_front_c0.png": ("C0_FRONT", 2.5),
            "radial_front_c3.png": ("C3_FRONT", 2.5),
            "radial_hole_example.png": ("HOLE", 3.0),
            "radial_min_max_pose.png": ("C3_MAX", 2.0),
        }
        for name, (scene, offset) in desired.items():
            if scene not in first_seen:
                raise RuntimeError(f"Scene missing from recording: {scene}")
            extract(output, first_seen[scene] + offset, PRESENTATION / name)
    payload = {"mode": args.mode, "video_path": str(output), "launch_command": command,
               "recording_command": record_command, "scene_first_seen_s": first_seen,
               "video": info, "gst_discoverer": discover.stdout,
               "trajectory_execution": False, "controller": False, "ros2_control": False,
               "hardware": False, "amr_motion": False}
    temporary = metadata_path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(temporary, metadata_path)
    print(json.dumps({"status": "PASS", **info}, indent=2))


if __name__ == "__main__":
    main()
