#!/usr/bin/env python3
import cv2,csv,json,os,subprocess
WS="/home/openarm/humanoid_sim_ws";P=f"{WS}/presentation";OUT=f"{P}/boundary_sweep_workspace_demo.mp4";FPS=30;HOLD=3
SLIDES=[f"boundary_front_c{i}.png" for i in range(4)]+["boundary_front_compare.png"]+[f"boundary_right_c{i}.png" for i in range(4)]+["boundary_right_compare.png","boundary_3d_compare.png"]
def main():
 if os.path.exists(OUT):raise RuntimeError("Refusing video overwrite")
 images=[cv2.resize(cv2.imread(f"{P}/{n}"),(1920,1080)) for n in SLIDES]
 if any(x is None for x in images):raise RuntimeError("Missing slide")
 pipe=["gst-launch-1.0","-q","fdsrc","fd=0","!","rawvideoparse","format=bgr","width=1920","height=1080","framerate=30/1","!","videoconvert","!","x264enc","speed-preset=medium","bitrate=5000","key-int-max=60","!","mp4mux","faststart=true","!","filesink",f"location={OUT}"];proc=subprocess.Popen(pipe,stdin=subprocess.PIPE)
 for image in images:
  for _ in range(FPS*HOLD):proc.stdin.write(image.tobytes())
 proc.stdin.close();code=proc.wait()
 if code:raise RuntimeError("GStreamer encoding failed")
 discover=subprocess.check_output(["gst-discoverer-1.0",OUT],text=True);cap=cv2.VideoCapture(OUT);decoded=0;width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
 while cap.read()[0]:decoded+=1
 cap.release();meta={"path":"presentation/boundary_sweep_workspace_demo.mp4","duration_seconds":decoded/FPS,"codec":"H.264","codec_name":"h264","resolution":f"{width}x{height}","fps":FPS,"decoded_frames":decoded,"file_size_bytes":os.path.getsize(OUT),"slide_count":len(images),"recording_backend":"GStreamer x264enc/mp4mux","gst_discover_h264":"H.264" in discover}
 with open(f"{P}/boundary_sweep_workspace_demo_metadata.json","x") as f:json.dump(meta,f,indent=2,sort_keys=True)
 print(meta)
if __name__=="__main__":main()
