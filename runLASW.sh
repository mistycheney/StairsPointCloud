#!/bin/bash

BINPATH=/Users/yuncong/Documents/StereoMatch-LASW/StereoMatch
IMGPATH=/Users/yuncong/Documents/workspace/StairsPointCloud/staircase_new
SCRIPTPATH=/Users/yuncong/Documents/imagedirs/imagedirs/LASWscripts

OLDPATH=`pwd`
IMGID=$1

convert $IMGPATH/top${IMGID}_rect_rot.pgm -resize 25% $IMGPATH/top${IMGID}_rect_rot_small.pgm 
convert $IMGPATH/bottom${IMGID}_rect_rot.pgm -resize 25% $IMGPATH/bottom${IMGID}_rect_rot_small.pgm 

echo "input_file $IMGPATH/top${IMGID}_rect_rot_small.pgm
input_file $IMGPATH/bottom${IMGID}_rect_rot_small.pgm" > $SCRIPTPATH/data_in_stairs.txt

cd $SCRIPTPATH

$BINPATH script exp.txt

cp AD_asw_35.pgm $IMGPATH/top${IMGID}_rect_rot_small_disp_LASW.pgm

cd $OLDPATH
