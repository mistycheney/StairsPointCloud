/*
 * canny.hpp
 *
 *  Created on: Mar 28, 2013
 *      Author: yuncong
 */

#ifndef CANNY_HPP_
#define CANNY_HPP_

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/photo/photo.hpp"

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "disparityMap.hpp"

using namespace cv;
using namespace std;

int find_edges();
int test_inpaint();

//void findLineSegments(int imId) {
//	Mat xyz;
//	disparityMap(imId, xyz);
//
//	string filename;
//	filename = str(boost::format("stairs_images/left%d.jpg") % imId);
//	Mat img = imread(filename, -1);
//	Mat imgBlur;
//	GaussianBlur(img, imgBlur, Size(11, 11), 1);
//
//	int thresh1_slider = 85;
//	int thresh2_slider = 13;
//	int hough_thresh = 88;
//	int minLineLength = 5;
//	int maxLineGap = 44;
//
//	Mat imgLines;
//	Canny(imgBlur, imgLines, (double) thresh1_slider, (double) thresh2_slider);
//
//	vector<Vec4i> lines;
//	HoughLinesP(imgLines, lines, 3, CV_PI / 60, max(10, hough_thresh),
//			max(10, minLineLength), max(10, maxLineGap));
//
////	Mat imgLinesColor;
////	cvtColor(imgLines, imgLinesColor, CV_GRAY2BGR);
//
//	double angle;
//	double EPSILON = 0.000001;
//
//	unsigned int ptSize = lines.size();
//	vector<float> samples(ptSize, 3);
//
//	for (size_t i = 0; i < lines.size(); i++) {
////		line(color_dst, Point(lines[i][0], lines[i][1]),
////				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
//
//		Vec3f e1 = xyz.at<Vec3f>(lines[i][1], lines[i][0]);
//		Vec3f e2 = xyz.at<Vec3f>(lines[i][3], lines[i][2]);
//		if (e1[2] > 20 || e2[2] > 20) {
//			continue;
//		}
//		Vec3f d = e2 - e1;
//		Vec3f normalVec = d/norm(d);
//		float normalVecArr[3] = [normalVec[0],normalVec[1],normalVec[2]];
//		1,3,CV_32F, normalVecArr);
//		samples.push_back(row);
//
////		angle = atan2(lines[i][1] - lines[i][3], lines[i][0] - lines[i][2]);
////		if (angle > CV_PI / 2) {
////			angle = angle - CV_PI;
////		} else if (angle < -CV_PI / 2) {
////			angle = angle + CV_PI;
////		}
////		cout << angle << endl;
//	}
//
//	int clusterCount = 3;
//	Mat labels;
//	int attempts = 5;
//	Mat centers;
//	kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 0.0001, 10000),
//			attempts, KMEANS_RANDOM_CENTERS, centers );
//	cout << centers << endl;
//}




#endif /* CANNY_HPP_ */
