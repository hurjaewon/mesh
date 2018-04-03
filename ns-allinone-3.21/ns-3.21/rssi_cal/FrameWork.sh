#!/bin/bash
str=$1

/usr/local/MATLAB/R2017a/bin/matlab -nojvm -nodesktop -nosplash -r "InputCalib('$str');quit;"

