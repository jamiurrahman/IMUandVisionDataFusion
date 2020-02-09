ffmpeg -i *.mp4 -vf "select=not(mod(n\,2))" -vsync vfr img_%03d.jpg
for i in {2..248}
do
	y1=$(printf "%03d" "$((i-1))")
	y2=$(printf "%03d" "$i")
	y3=$(printf "%03d" "$((i+1))")
	"../../cmake-build-debug/KeypointMatching" "img_$y1.jpg" "img_$y2.jpg" "img_$y3.jpg" "keypoint_$i.txt"
done
