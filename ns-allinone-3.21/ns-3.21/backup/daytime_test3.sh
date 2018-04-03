#!/bin/bash
seed=$1
date=$2
NofAP=$3

for i in {6..12}
do
	x=$((i+1))
	for j in $(seq $x $NofAP) 
	do
		./waf --run "scratch/mesh_daytime -map1_pos=$i -map2_pos=$j -seed=$1" >> ./simulation_results/$date/$date"_daytime_"$seed.txt
	done
done
