nohup ffserver &
sleep 1 
nohup ffmpeg -r 25 -f v4l2 -channel 0 -video_size 640x480 -i /dev/video0 -pix_fmt yuv420p -an -maxrate 8M http://localhost:8090/camera.ffm
