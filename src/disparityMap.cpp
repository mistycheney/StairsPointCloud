/*
 * stairsPointCloud.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: yuncong
 */

#include "disparityMap.hpp"

using namespace cv;
using namespace std;

Mat img_rect[2];
StereoSGBM sgbm;
int SADWindowSize = 13;
int numberOfDisparitiesMultiple = 13;
int preFilterCap = 100;
bool no_display = false;
//float scale = 1.f;
Size imageSize;
int minDisparity = 0;
int uniquenessRatio = 3;
int speckleWindowSize = 0;
int sgbmP1 = 10, sgbmP2 = 5000;
Mat dispTop, dispBottom, disp8;
Mat top_rect_rot, bottom_rect_rot;

void on_sgbm_trackbar(int, void*) {
//	int numberOfDisparities =
//				numberOfDisparities > 0 ?
//						numberOfDisparities : ((imageSize.width / 8) + 15) & -16;
	int numberOfDisparities = 16 * max(1, numberOfDisparitiesMultiple);

	sgbm.preFilterCap = preFilterCap;
	sgbm.SADWindowSize = SADWindowSize;

//	int cn = img_rect[1].channels();
//	sgbm.P1 = 8 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
//	sgbm.P2 = 32 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.P1 = sgbmP1;
	sgbm.P2 = sgbmP2;
	sgbm.minDisparity = minDisparity;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = uniquenessRatio;
	sgbm.speckleWindowSize = speckleWindowSize;
	sgbm.speckleRange = 1;
	sgbm.disp12MaxDiff = -1;
	sgbm.fullDP = false;

	cout << "sgbm.P1 " << sgbm.P1 << endl;
	cout << "sgbm.P2 " << sgbm.P2 << endl;
	cout << "sgbm.preFilterCap " << sgbm.preFilterCap << endl;
	cout << "sgbm.SADWindowSize " << sgbm.SADWindowSize << endl;
	cout << "sgbm.minDisparity " << sgbm.minDisparity << endl;
	cout << "sgbm.numberOfDisparities " << sgbm.numberOfDisparities << endl;
	cout << "sgbm.uniquenessRatio " << sgbm.uniquenessRatio << endl;
	cout << "sgbm.speckleWindowSize " << sgbm.speckleWindowSize << endl;
	cout << endl;


	sgbm(top_rect_rot, bottom_rect_rot, dispTop);
//	sgbm(bottom_rect_rot, top_rect_rot, dispBottom);
//
//	for (int i=0; i<imageSize.height; i++) {
//		for (int j=0; j<imageSize.width; j++) {
//			double dl = dispTop.at<double>(i,j);
//			double dr = dispBottom.at<double>(i,j+dl);
//			if (isinf(dl)==0 && isinf(dr)==0 && abs(dl) - abs(dr) > 10) {
//				dispTop.at<double>(i,j) = nan("1");
//			}
//		}
//	}

	dispTop.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));

	Mat disp_small;
	Size disp_size = disp8.size();
	resize(disp8, disp_small, Size(disp_size.width / 4, disp_size.height / 4));

	imshow("sgbm", disp_small);

}

int disparityMap(int imgId) {

	bool useCalibrated = true;
	int i, j, k;

	Mat rmap[2][2];

	FileStorage fs("extrinsics.yml", FileStorage::READ);

	Mat R, T, R1, R2, P1, P2, Q;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
	fs.release();

	Mat M1, M2, D1, D2;
	fs.open("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs.release();

	string filenames[2];
	filenames[0] = "staircase_new/top" + boost::lexical_cast<string>(imgId)
			+ ".jpg";
	filenames[1] = "staircase_new/bottom" + boost::lexical_cast<string>(imgId)
			+ ".jpg";

	Mat img[2];
	img[0] = imread(filenames[0], 0);
	img[1] = imread(filenames[1], 0);
	imageSize = img[0].size();
	Size imageSizeSmall = Size(imageSize.width / 4, imageSize.height / 4);

	Mat top_small, bottom_small;

	Mat imgHE[2];
	equalizeHist(img[0], imgHE[0]);
	equalizeHist(img[1], imgHE[1]);
	resize(imgHE[0], top_small, imageSizeSmall);
	resize(imgHE[1], bottom_small, imageSizeSmall);
	imshow("topHE", top_small);
	imshow("bottomHE", bottom_small);

	Mat imgNorm[2];
	int normWinSize = 11;
	Mat A = Mat::zeros(normWinSize,normWinSize,CV_32F);
	A.at<float>(normWinSize/2,normWinSize/2) = 1;
	Mat M = A - 1/(normWinSize*normWinSize);
	filter2D(img[0], imgNorm[0], -1, M);
	filter2D(img[1], imgNorm[1], -1, M);
	resize(imgNorm[0], top_small, imageSizeSmall);
	resize(imgNorm[1], bottom_small, imageSizeSmall);
	imshow("topNorm", top_small);
	imshow("bottomNorm", bottom_small);

	Rect validRoi[2];

	stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0],
			rmap[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0],
			rmap[1][1]);

	bool isVerticalStereo = fabs(P2.at<double>(1, 3))
			> fabs(P2.at<double>(0, 3));

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo) {
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);
	} else {
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (k = 0; k < 2; k++) {
		Mat rimg, cimg;
		remap(imgHE[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);

		if (imgHE[k].channels() == 1) {
			cvtColor(rimg, cimg, CV_GRAY2BGR);
		} else {
			cimg = rimg;
		}
		Mat canvasPart =
				!isVerticalStereo ?
						canvas(Rect(w * k, 0, w, h)) :
						canvas(Rect(0, h * k, w, h));
		resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
		if (useCalibrated) {
			Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
					cvRound(validRoi[k].width * sf),
					cvRound(validRoi[k].height * sf));

			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
		}

		img_rect[k] = rimg;
	}

	if (!isVerticalStereo)
		for (j = 0; j < canvas.rows; j += 16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0),
					1, 8);
	else
		for (j = 0; j < canvas.cols; j += 16)
			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0),
					1, 8);

	namedWindow("rectified", 1);
	imshow("rectified", canvas);

	vector<int> flags;
	flags.push_back(CV_IMWRITE_PXM_BINARY);
	flags.push_back(1);
//	cvtColor(img_rect[0].t(), top_rect_rot, CV_BGR2GRAY);
	top_rect_rot = img_rect[0].t();
	string fn_top_r = "staircase_new/top" + boost::lexical_cast<string>(imgId)
			+ "_rect_rot.pgm";
	imwrite(fn_top_r, top_rect_rot, flags);
//	cvtColor(img_rect[1].t(), bottom_rect_rot, CV_BGR2GRAY);
	bottom_rect_rot = img_rect[1].t();
	string fn_bottom_r = "staircase_new/bottom"
			+ boost::lexical_cast<string>(imgId) + "_rect_rot.pgm";
	imwrite(fn_bottom_r, bottom_rect_rot, flags);

	namedWindow("sgbm", 1);
	createTrackbar("P1", "sgbm",
				&sgbmP1, 300, on_sgbm_trackbar);
	createTrackbar("P2", "sgbm",
				&sgbmP2, 10000, on_sgbm_trackbar);
	createTrackbar("numberOfDisparitiesMultiple", "sgbm",
			&numberOfDisparitiesMultiple, 50, on_sgbm_trackbar);
	createTrackbar("SADWindowSize", "sgbm", &SADWindowSize, 50,
			on_sgbm_trackbar);
	createTrackbar("preFilterCap", "sgbm", &preFilterCap, 300,
			on_sgbm_trackbar);
	createTrackbar("minDisparity", "sgbm", &minDisparity, 100,
			on_sgbm_trackbar);
	createTrackbar("uniquenessRatio", "sgbm", &uniquenessRatio, 20,
			on_sgbm_trackbar);
	createTrackbar("speckleWindowSize", "sgbm", &speckleWindowSize, 250,
			on_sgbm_trackbar);
	on_sgbm_trackbar(0, NULL);

	waitKey();

//	enum {
//		STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3
//	};
////	int alg = STEREO_SGBM;
//	float scale = 1.f;

//	StereoBM bm;
//	StereoVar var;

//	if (alg == STEREO_BM) {
//		if (img_r[0].channels() == 3) {
//			Mat temp;
//			cvtColor(img_r[0], temp, CV_RGB2GRAY);
//			img_r[0] = temp;
//
//			cvtColor(img_r[1], temp, CV_RGB2GRAY);
//			img_r[1] = temp;
//		}
//	}

//	if (scale != 1.f) {
//		Mat temp1, temp2;
//		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//		resize(img_r[0], temp1, Size(), scale, scale, method);
//		img_r[0] = temp1;
//		resize(img_r[1], temp2, Size(), scale, scale, method);
//		img_r[1] = temp2;
//	}

//	numberOfDisparities =
//			numberOfDisparities > 0 ?
//					numberOfDisparities : ((imageSize.width / 8) + 15) & -16;

//	bm.state->roi1 = validRoi[0];
//	bm.state->roi2 = validRoi[1];
//	bm.state->preFilterCap = 31;
//	bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
//	bm.state->minDisparity = 0;
//	bm.state->numberOfDisparities = numberOfDisparities;
//	bm.state->textureThreshold = 10;
//	bm.state->uniquenessRatio = 15;
//	bm.state->speckleWindowSize = 100;
//	bm.state->speckleRange = 32;
//	bm.state->disp12MaxDiff = 1;

//	var.levels = 3;                              // ignored with USE_AUTO_PARAMS
//	var.pyrScale = 0.5;                          // ignored with USE_AUTO_PARAMS
//	var.nIt = 25;
//	var.minDisp = -numberOfDisparities;
//	var.maxDisp = 0;
//	var.poly_n = 3;
//	var.poly_sigma = 0.0;
//	var.fi = 15.0f;
//	var.lambda = 0.03f;
//	var.penalization = var.PENALIZATION_TICHONOV; // ignored with USE_AUTO_PARAMS
//	var.cycle = var.CYCLE_V;                     // ignored with USE_AUTO_PARAMS
//	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS
//			| var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;

//	if (alg == STEREO_BM)
//		bm(img_r[0], img_r[1], disp);
//	else if (alg == STEREO_VAR) {
//		var(img_r[0], img_r[1], disp);
//	} else if (alg == STEREO_SGBM || alg == STEREO_HH) {
//		img_r[0] = img_r[0].t();
//		img_r[1] = img_r[1].t();
//		sgbm(img_r[0], img_r[1], disp);
//	}

//	if (alg != STEREO_VAR)
//		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
//	else
//		disp.convertTo(disp8, CV_8U);
//
//	if (!no_display) {
//		namedWindow("top", 1);
//		imshow("top", img_r[0]);
//		namedWindow("bottom", 1);
//		imshow("bottom", img_r[1]);
//		namedWindow("disparity", 1);
//		imshow("disparity", disp8);
//		printf("press any key to continue...");
//		fflush(stdout);
//		waitKey();
//		printf("\n");
//	}

	const string disparity_filename = "staircase_new/top" + boost::lexical_cast<string>(imgId)
				+ "_rect_rot_disp_SGBM.pgm";
	imwrite(disparity_filename, disp8);

	Mat xyz;
	reprojectImageTo3D(dispTop, xyz, Q, true);

	const double max_z = 2000;

	printf("storing the point cloud...");
	fflush(stdout);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int y = 0; y < xyz.rows; y++) {
		for (int x = 0; x < xyz.cols; x++) {
			pcl::PointXYZRGB pcl_point;
			Vec3f point = xyz.at<Vec3f>(y, x);
			if (isinf(point[0]) != 0) {
				continue;
			}
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;
//			cout << point[0] << "," << point[1] << "," << point[2] << endl;
			pcl_point.x = point[0];
			pcl_point.y = point[1];
			pcl_point.z = point[2];

			Vec3b color = top_rect_rot.at<Vec3b>(y, x);
			uint32_t rgb = (static_cast<uint32_t>(color[2]) << 16
					| static_cast<uint32_t>(color[1]) << 8
					| static_cast<uint32_t>(color[0]));
			pcl_point.rgb = *reinterpret_cast<float*>(&rgb);
			cloud->points.push_back(pcl_point);
		}
	}

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	String cloud_name = "cloud" + boost::lexical_cast<string>(imgId) + ".pcd";
	pcl::io::savePCDFile(cloud_name, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to "
			<< cloud_name << std::endl;

	destroyAllWindows();

	return 0;
}
