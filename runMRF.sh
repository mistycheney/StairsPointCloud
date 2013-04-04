#!/bin/bash

BIN=/Users/yuncong/Documents/StereoMatcher/mrfstereo/mrfstereo
IMGPATH=/Users/yuncong/Documents/workspace/StairsPointCloud/staircase_new

IMGID=$1
shift
if [ -z $IMGID ]; then 
    $BIN
else
    $BIN $@ $IMGPATH/top${IMGID}_rect_rot.pgm $IMGPATH/bottom${IMGID}_rect_rot.pgm $IMGPATH/top${IMGID}_rect_rot_disp_mrf.pgm
fi
