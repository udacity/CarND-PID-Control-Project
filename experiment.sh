#!/usr/bash
cd build
k=1
while [ $k -le 1000 ]
do
	echo "start $k th episode"
	xdotool search "self_driving_car_nanodegree_program" mousemove 158 534 click 1
	sleep 0.5
	xdotool search "self_driving_car_nanodegree_program" mousemove 158 534 click 1
	sleep 0.5
	xdotool search "self_driving_car_nanodegree_program" mousemove 513 677 click 1
	sleep 4
	./pid
	sleep 0.5
	xdotool key "Escape"
	sleep 0.5
	k=$((k+1))
done
