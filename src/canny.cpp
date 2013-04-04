/*
 * canny.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: yuncong
 */

#include "canny.hpp"

using namespace cv;
using namespace std;

Mat out;
Mat src;

Mat mask;
Mat disp1;
int inpaint_radius;

int thresh1_slider = 85;
int thresh2_slider = 13;
int hough_thresh = 88;
int minLineLength = 5;
int maxLineGap = 44;

void on_inpaint_trackbar(int, void*) {
	inpaint(disp1, mask, out, max(1, inpaint_radius), INPAINT_TELEA);
	imshow("inpaint", out);
}

int test_inpaint() {
	disp1 = imread("disp.jpg", 0);
	imshow("disp", disp1);
	disp1.copyTo(mask);

	threshold(mask, mask, 10, 255, CV_THRESH_BINARY);
	mask = 255 - mask;
	imshow("mask", mask);

	namedWindow("inpaint", 1);
	createTrackbar("radius", "inpaint", &inpaint_radius, 50,
			on_inpaint_trackbar);
	waitKey();
	return 0;
}


void on_trackbar(int, void*) {
	Canny(src, out, (double) thresh1_slider, (double) thresh2_slider);

	vector<Vec4i> lines;
	HoughLinesP(out, lines, 3, CV_PI / 60, max(10, hough_thresh),
			max(10, minLineLength), max(10, maxLineGap));

	Mat color_dst;
	cvtColor(out, color_dst, CV_GRAY2BGR);

	double angle;
	double EPSILON = 0.000001;

	for (size_t i = 0; i < lines.size(); i++) {
		line(color_dst, Point(lines[i][0], lines[i][1]),
				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);

		angle = atan2(lines[i][1] - lines[i][3], lines[i][0] - lines[i][2]);
		if (angle > CV_PI / 2) {
			angle = angle - CV_PI;
		} else if (angle < -CV_PI / 2) {
			angle = angle + CV_PI;
		}
		cout << angle << endl;
	}
	cout << endl;

	imshow("canny", color_dst);
}

int find_edges() {
	int imId = 5;
	string filenames[2];
	filenames[0] = str(boost::format("stairs_images/left%d.jpg") % imId);
	filenames[1] = str(boost::format("stairs_images/right%d.jpg") % imId);

	Mat img[2];
	img[0] = imread(filenames[0], -1);
	img[1] = imread(filenames[1], -1);
	Size imageSize = img[0].size();

	GaussianBlur(img[0], src, Size(11, 11), 1);
	imshow("blur", src);

	namedWindow("canny", 1);

	createTrackbar("threshold1", "canny", &thresh1_slider, 500, on_trackbar);
	createTrackbar("threshold2", "canny", &thresh2_slider, 500, on_trackbar);
	createTrackbar("hough_thresh", "canny", &hough_thresh, 500, on_trackbar);
	createTrackbar("minLineLength", "canny", &minLineLength, 500, on_trackbar);
	createTrackbar("maxLineGap", "canny", &maxLineGap, 500, on_trackbar);
	waitKey();

	return 0;

}

