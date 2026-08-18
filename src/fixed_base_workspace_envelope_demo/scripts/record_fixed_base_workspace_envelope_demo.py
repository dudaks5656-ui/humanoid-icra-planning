#!/usr/bin/env python3
"""Record and decode-validate the RViz-only workspace envelope demo."""

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
RUNTIME = PRESENTATION / "fixed_base_workspace_envelope_demo_runtime.json"


def find_rviz_window(timeout=30.0):
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


def decode_metadata(path):
    import cv2
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV cannot open {path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    declared = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame is None or frame.size == 0:
            raise RuntimeError(f"Empty decoded frame {decoded}")
        decoded += 1
    capture.release()
    if decoded <= 0 or fps <= 0 or abs(decoded - declared) > 1:
        raise RuntimeError(f"Frame decode mismatch decoded={decoded} declared={declared}")
    return {"fps": fps, "frame_count": decoded, "declared_frame_count": declared,
            "duration_s": decoded / fps, "width": width, "height": height,
            "file_size_bytes": path.stat().st_size, "all_frames_decoded": True}


def extract(video, seconds, output):
    import cv2
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_MSEC, max(0.0, seconds) * 1000.0)
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None or frame.size == 0 or not cv2.imwrite(str(output), frame):
        raise RuntimeError(f"Cannot extract {output} at {seconds:.3f}s")
    return {"path": str(output), "time_s": seconds, "width": int(frame.shape[1]), "height": int(frame.shape[0])}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("full", "short"), required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    PRESENTATION.mkdir(parents=True, exist_ok=True)
    suffix = "" if args.mode == "full" else "_short"
    output = PRESENTATION / f"fixed_base_workspace_envelope_demo{suffix}.mp4"
    metadata_path = PRESENTATION / f"fixed_base_workspace_envelope_demo_recording_{args.mode}.json"
    launch_log = PRESENTATION / f"fixed_base_workspace_envelope_demo_{args.mode}_launch.log"
    if output.exists() and not args.overwrite:
        raise RuntimeError(f"Refusing to overwrite without --overwrite: {output}")
    if output.exists():
        output.unlink()
    if RUNTIME.exists():
        RUNTIME.unlink()

    duration_scale = 1.0 if args.mode == "full" else 0.58
    recording_seconds = 66.0 if args.mode == "full" else 43.0
    frame_limit = int(math.ceil(recording_seconds * 30.0))
    launch_command = [
        "ros2", "launch", "fixed_base_workspace_envelope_demo",
        "fixed_base_workspace_envelope_demo.launch.py", "demo_scene:=auto",
        "visualization_mode:=surface", f"duration_scale:={duration_scale}",
        "use_rviz:=true", "use_overlay:=true",
    ]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "42"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR", "QT_PLUGIN_PATH"):
        if "cv2" in environment.get(key, ""):
            environment.pop(key, None)

    with launch_log.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(launch_command, cwd=WORKSPACE, env=environment,
                                  stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True, text=True)
        try:
            _, xid, width, height, x, y, window_line = find_rviz_window()
            time.sleep(1.0)
            _, xid, width, height, x, y, window_line = find_rviz_window(timeout=5.0)
            recorder_command = [
                "gst-launch-1.0", "-e", "ximagesrc", f"startx={x}", f"starty={y}",
                f"endx={x + width - 1}", f"endy={y + height - 1}",
                "use-damage=false", "show-pointer=false", "!", "videorate", "drop-only=true", "max-rate=30",
                "!", "video/x-raw,framerate=30/1", "!", "identity", "sync=true", f"eos-after={frame_limit}",
                "!", "videoconvert", "!", "videoscale", "add-borders=true",
                "!", "video/x-raw,width=1920,height=1080,format=I420,pixel-aspect-ratio=1/1",
                "!", "x264enc", "bitrate=8000", "speed-preset=medium", "key-int-max=60", "threads=0",
                "!", "video/x-h264,stream-format=avc,alignment=au", "!", "mp4mux", "faststart=true",
                "!", "filesink", f"location={output}",
            ]
            recording_start = time.monotonic()
            recorder = subprocess.Popen(recorder_command, cwd=WORKSPACE, env=environment)
            scene_first_seen = {}
            previous = None
            while recorder.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError(f"ROS launch exited during recording; inspect {launch_log}")
                try:
                    status = json.loads(RUNTIME.read_text(encoding="utf-8"))
                    scene = status.get("scene")
                    if scene and scene != previous:
                        scene_first_seen.setdefault(scene, time.monotonic() - recording_start)
                        previous = scene
                except (FileNotFoundError, json.JSONDecodeError):
                    pass
                time.sleep(0.05)
            if recorder.returncode != 0:
                raise RuntimeError(f"GStreamer failed with code {recorder.returncode}")
        finally:
            stop_group(launch)

    info = decode_metadata(output)
    if info["width"] != 1920 or info["height"] != 1080 or abs(info["fps"] - 30.0) > 0.1:
        raise RuntimeError(f"Unexpected recording format: {info}")
    discover = subprocess.run(["gst-discoverer-1.0", str(output)], text=True, capture_output=True, check=False)
    if discover.returncode != 0 or "H.264" not in discover.stdout or "Quicktime" not in discover.stdout:
        raise RuntimeError(f"Codec/container validation failed:\n{discover.stdout}\n{discover.stderr}")

    extracted = {}
    if args.mode == "full":
        desired = {
            "envelope_c0.png": ("C0", 3.0), "envelope_c1.png": ("C1", 3.0),
            "envelope_c2.png": ("C2", 3.0), "envelope_c3.png": ("C3", 3.5),
            "envelope_c0_vs_c3.png": ("C0_VS_C3", 3.5),
            "envelope_combined_only.png": ("COMBINED_ONLY", 4.0),
        }
        for name, (scene, offset) in desired.items():
            if scene not in scene_first_seen:
                raise RuntimeError(f"Required scene not observed: {scene}")
            extracted[name] = extract(output, scene_first_seen[scene] + offset, PRESENTATION / name)

    metadata = {
        "mode": args.mode, "video_path": str(output), "launch_command": launch_command,
        "recording_command": recorder_command,
        "rviz_window": {"xid": xid, "source_width": width, "source_height": height,
                        "screen_x": x, "screen_y": y, "description": window_line},
        "scene_first_seen_s": scene_first_seen, "video": info,
        "gst_discoverer": discover.stdout, "representative_frames": extracted,
        "trajectory_execution": False, "controller": False, "ros2_control": False,
        "hardware": False, "amr_motion": False,
    }
    temporary = metadata_path.with_suffix(metadata_path.suffix + ".tmp")
    temporary.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    os.replace(temporary, metadata_path)
    print(json.dumps({"status": "PASS", "output": str(output), **info}, indent=2))


if __name__ == "__main__":
    main()
