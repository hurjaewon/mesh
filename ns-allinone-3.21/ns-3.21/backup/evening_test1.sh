#!/bin/bash
seed=$1
date=$2
NofAP=$3

for i in {0..2}
do
	x=$((i+1))
	for j in $(seq $x $NofAP) 
	do
		./waf --run "scratch/mesh_evening -map1_pos=$i -map2_pos=$j -seed=$1" >> ./simulation_results/$date/$date"_evening_"$seed.txt
	done
done
