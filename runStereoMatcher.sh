#!/bin/bash

BINPATH=/Users/yuncong/Documents/StereoMatch/StereoMatch
IMGPATH=/Users/yuncong/Documents/workspace/StairsPointCloud/staircase_new

IMGID=$1



$BINPATH script $IMGPATH/top${IMGID}_rect_rot.pgm $IMGPATH/bottom${IMGID}_rect_rot.pgm 
