#!/bin/bash

for i in 2 4 6 8 10 12 14 16 18 20 22 24 26 28 30 32 34 36 38
do
	echo "step = $i" >> log_mesh_test
	./waf --run "scratch/mesh_test -step=$i -dist=39.0" >> log_mesh_test
done


