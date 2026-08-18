#!/usr/bin/env python3
"""Encode a deterministic 45-second H.264 presentation from validated figures."""
import json, os, subprocess
import cv2

WS="/home/openarm/humanoid_sim_ws";PRE=f"{WS}/presentation";OUT=f"{PRE}/lift_slice_fk_workspace_demo.mp4"
SLIDES=[]
for i in range(4):SLIDES += [f"lift_slice_front_c{i}.png",f"lift_slice_right_c{i}.png"]
SLIDES += ["lift_slice_front_compare.png","lift_slice_right_compare.png"]
SLIDES += [f"lift_slice_3d_c{i}.png" for i in range(4)]+["lift_slice_3d_all.png"]
FPS=30;HOLD=3

def main():
    if os.path.exists(OUT):raise RuntimeError(f"Refusing overwrite: {OUT}")
    images=[]
    for name in SLIDES:
        image=cv2.imread(os.path.join(PRE,name));
        if image is None:raise RuntimeError(f"Missing figure: {name}")
        images.append(cv2.resize(image,(1920,1080),interpolation=cv2.INTER_AREA))
    pipeline=["gst-launch-1.0","-q","fdsrc","fd=0","!","rawvideoparse","format=bgr","width=1920","height=1080","framerate=30/1","!","videoconvert","!","x264enc","speed-preset=medium","bitrate=5000","key-int-max=60","!","mp4mux","faststart=true","!","filesink",f"location={OUT}"]
    process=subprocess.Popen(pipeline,stdin=subprocess.PIPE)
    try:
        for image in images:
            for _ in range(FPS*HOLD):process.stdin.write(image.tobytes())
        process.stdin.close()
    finally:
        code=process.wait()
    if code:raise RuntimeError(f"GStreamer H.264 encoding failed: {code}")
    discover=subprocess.check_output(["gst-discoverer-1.0",OUT],text=True)
    if "H.264" not in discover:raise RuntimeError("GStreamer did not identify H.264")
    cap=cv2.VideoCapture(OUT);decoded=0
    while True:
        ok,_=cap.read()
        if not ok:break
        decoded+=1
    width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));cap.release()
    meta={"path":"presentation/lift_slice_fk_workspace_demo.mp4","duration_seconds":decoded/FPS,"codec":"H.264","codec_name":"h264","resolution":f"{width}x{height}","fps":FPS,"decoded_frames":decoded,"file_size_bytes":os.path.getsize(OUT),"slide_count":len(images),"recording_backend":"GStreamer x264enc/mp4mux","gst_discover_h264":True}
    with open(f"{PRE}/lift_slice_fk_workspace_demo_metadata.json","x",encoding="utf-8") as f:json.dump(meta,f,indent=2,sort_keys=True)
    print(meta)
if __name__=="__main__":main()
