#!/usr/bin/env python3
"""Encode the staged differential story without overlaying four complete shells."""
import json
import os
import shutil
import subprocess

import cv2

WS = "/home/openarm/humanoid_sim_ws"
PRE = f"{WS}/presentation"
OUT = f"{PRE}/boundary_sweep_workspace_differential_demo.mp4"
META = f"{PRE}/boundary_sweep_workspace_differential_demo_metadata.json"
FPS = 30
HOLD_SECONDS = 4
SLIDES = [
    "boundary_3d_c0.png",
    "boundary_3d_yaw_effect.png",
    "boundary_3d_pitch_effect.png",
    "boundary_3d_single_dof_shared.png",
    "boundary_3d_combined_only.png",
    "boundary_3d_differential.png",
    "boundary_3d_c0_vs_c3_expansion.png",
    "boundary_3d_four_configurations.png",
]


def binary(name, environment_name):
    override = os.environ.get(environment_name)
    if override:
        return override
    found = shutil.which(name)
    if not found:
        raise RuntimeError(f"{name} not found; set {environment_name}")
    return found


def main():
    if os.path.exists(OUT) or os.path.exists(META):
        raise RuntimeError("Refusing differential video overwrite")
    ffmpeg = binary("ffmpeg", "FFMPEG_BINARY")
    ffprobe = binary("ffprobe", "FFPROBE_BINARY")
    images = []
    for name in SLIDES:
        image = cv2.imread(f"{PRE}/{name}")
        if image is None:
            raise RuntimeError(f"Missing slide: {name}")
        images.append(cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA))
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-n",
        "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", "1920x1080",
        "-framerate", str(FPS), "-i", "pipe:0", "-an", "-c:v", "libx264",
        "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", "-profile:v", "high",
        "-level:v", "4.0", "-movflags", "+faststart", OUT,
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    for image in images:
        for _ in range(FPS * HOLD_SECONDS):
            process.stdin.write(image.tobytes())
    process.stdin.close()
    if process.wait():
        raise RuntimeError("FFmpeg encoding failed")
    probe = json.loads(
        subprocess.check_output(
            [
                ffprobe, "-v", "error", "-select_streams", "v:0", "-count_frames",
                "-show_entries", "stream=codec_name,profile,level,pix_fmt,width,height,r_frame_rate,nb_read_frames",
                "-show_entries", "format=duration,size", "-of", "json", OUT,
            ],
            text=True,
        )
    )
    subprocess.run([ffmpeg, "-v", "error", "-i", OUT, "-map", "0:v:0", "-f", "null", "-"], check=True)
    stream = probe["streams"][0]
    metadata = {
        "path": "presentation/boundary_sweep_workspace_differential_demo.mp4",
        "codec_name": stream["codec_name"],
        "profile": stream["profile"],
        "level": stream["level"],
        "pixel_format": stream["pix_fmt"],
        "resolution": f"{stream['width']}x{stream['height']}",
        "fps": stream["r_frame_rate"],
        "decoded_frames": int(stream["nb_read_frames"]),
        "duration_seconds": float(probe["format"]["duration"]),
        "file_size_bytes": int(probe["format"]["size"]),
        "audio": False,
        "faststart": True,
        "slide_count": len(SLIDES),
        "seconds_per_slide": HOLD_SECONDS,
        "encoder": "FFmpeg libx264",
    }
    with open(META, "x", encoding="utf-8") as stream_file:
        json.dump(metadata, stream_file, indent=2, sort_keys=True)
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
