/*
 * StairsPointCloud.hpp
 *
 *  Created on: Mar 18, 2013
 *      Author: yuncong
 */

#ifndef STAIRSPOINTCLOUD_HPP_
#define STAIRSPOINTCLOUD_HPP_

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "Eigen/Eigen"

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <pcl/ml/kmeans.h>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>
#include <sstream>

#include "config.hpp"

using namespace std;

void displayMainLoop(
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template<typename PointT>
int displayRGBCloud(typename pcl::PointCloud<PointT>::Ptr cloud, string title =
		"viewer") {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer(title));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
	viewer->addPointCloud<PointT>(cloud, rgb, "cloud");

	displayMainLoop(viewer);
}

//void addRGBToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
////	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
////			cloud, 0, 0, 255);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
//			cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
//	while (!viewer->wasStopped()) {
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}

int displayCloudRGBNormals(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(
//			cloud_with_normals);
//	viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_with_normals, rgb,
//			"cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(
			cloud_with_normals, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_with_normals,
			single_color, "cloud");

	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_with_normals, 30,
			0.1, "normals");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

int displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

pcl::console::TicToc tt;

template<typename PointT>
void voxelDownsample(typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<PointT>::Ptr outCloud) {
	std::cerr << "voxelDownsample...", tt.tic();

	typename pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE,
			DOWNSAMPLE_VOXEL_SIZE);
	sor.filter(*outCloud);
	std::cerr << "PointCloud after filtering: "
			<< outCloud->width * outCloud->height << " data points ("
			<< pcl::getFieldsList(*outCloud) << ")";
	std::cerr << "...done: " << tt.toc() << " ms, " << endl;
}

template<typename PointT, typename NormalT>
void estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<NormalT>::Ptr normals) {

	std::cerr << "estimateNormals...", tt.tic();

	typename pcl::NormalEstimation<PointT, NormalT> ne;
	typename pcl::search::KdTree<PointT>::Ptr tree(
			new typename pcl::search::KdTree<PointT>);

	typename pcl::PointCloud<NormalT>::Ptr normals_all(
			new typename pcl::PointCloud<NormalT>);

	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
//	ne.setRadiusSearch(1);
	ne.setRadiusSearch(ESTIMATE_NORMALS_RADIUS);
//	ne.setKSearch(10);
	ne.compute(*normals_all);

	std::cerr << "done: " << tt.toc() << " ms, " << normals_all->points.size()
			<< " normals\n";

	std::vector<int> index;
	pcl::removeNaNNormalsFromPointCloud(*normals_all, *normals, index);

	std::cerr << "removing NAN, done: " << tt.toc() << " ms, "
			<< normals->points.size() << " normals\n";
}

template<typename PointT, typename NormalT>
void smoothNormals(typename pcl::PointCloud<PointT>::Ptr cloud,
		typename pcl::PointCloud<NormalT>::Ptr normals) {
	pcl::MovingLeastSquares<PointT, NormalT> mls;
	typename pcl::search::KdTree<PointT>::Ptr tree(
			new typename pcl::search::KdTree<PointT>);
	typename pcl::PointCloud<NormalT>::Ptr normals_all(
			new typename pcl::PointCloud<NormalT>);

	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(false);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(*normals_all);

	normals_all->width = cloud->points.size();
	normals_all->height = 1;
	normals_all->is_dense = false; /* important */

	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*normals_all, *normals, index);

	for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pit =
			normals->begin(); pit != normals->end(); pit++) {
		pcl::flipNormalTowardsViewpoint(*pit, 0, 0, 0, pit->normal_x,
				pit->normal_y, pit->normal_z);
	}
}

template<typename T>
void removeNaNInplace(typename pcl::PointCloud<T>::Ptr cloud) {
	std::cerr << "removeNaNInplace..." << cloud->points.size() << " points\n", tt.tic();
	std::vector<int> index;
	typename pcl::PointCloud<T>::Ptr cloud_copy(
			new typename pcl::PointCloud<T>);
	pcl::copyPointCloud(*cloud, *cloud_copy);
	pcl::removeNaNNormalsFromPointCloud(*cloud_copy, *cloud, index);
	std::cerr << "done: " << tt.toc() << " ms, " << cloud->points.size()
			<< " points\n";
}

template<typename T>
void removeOutlier(typename pcl::PointCloud<T>::Ptr inCloud,
		typename pcl::PointCloud<T>::Ptr outCloud) {
	std::cerr << "removeOutlier..." << inCloud->points.size() << " points...", tt.tic();

	pcl::StatisticalOutlierRemoval<T> sor;
	sor.setInputCloud(inCloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.2);
	sor.filter(*outCloud);
	std::cerr << "done: " << tt.toc() << " ms, " << outCloud->points.size()
			<< " points\n";
}

template<typename T>
void euclideanClustering(typename pcl::PointCloud<T>::Ptr inCloud,
		std::vector<pcl::PointIndices>& cluster_indices) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("euclideanClustering"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	typename pcl::search::KdTree<T>::Ptr tree(
			new typename pcl::search::KdTree<T>);
	pcl::EuclideanClusterExtraction<T> ec;
	ec.setClusterTolerance(0.03);
	ec.setMinClusterSize(10);
//	  ec.setMaxClusterSize (25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(inCloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it =
			cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		typename pcl::PointCloud<T>::Ptr cloud_cluster(
				new typename pcl::PointCloud<T>);
		for (std::vector<int>::const_iterator pit = it->indices.begin();
				pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(inCloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: "
				<< cloud_cluster->points.size() << " data points.\n";
		j++;

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
				cloud_cluster, rand() & 255, rand() & 255, rand() & 255);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cluster, single_color,
				boost::lexical_cast<string>(j));
	}

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template<typename PointT, typename NormalT>
void regionGrowingSegmentation(typename pcl::PointCloud<PointT>::Ptr inCloud,
		typename pcl::PointCloud<NormalT>::Ptr inNormal,
		std::vector<pcl::PointIndices>& clusters) {
	std::cerr << "region growing segmentation...", tt.tic();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("euclideanClustering"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize(100);
//	reg.setMaxClusterSize(10000);
	reg.setNumberOfNeighbours(10);
	reg.setInputCloud(inCloud);
	reg.setSearchMethod(tree);
	//reg.setIndices (indices);
	reg.setInputNormals(inNormal);
	reg.setSmoothnessThreshold(20.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	reg.extract(clusters);

	std::cerr << ">> Done: " << tt.toc() << " ms, " << clusters.size()
			<< " clusters\n";

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator cit = clusters.begin();
			cit != clusters.end(); ++cit) {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = cit->indices.begin();
				pit != cit->indices.end(); pit++)
			cloud_cluster->points.push_back(inCloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: "
				<< cloud_cluster->points.size() << " data points." << std::endl;
		j++;

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
				cloud_cluster, rand() & 255, rand() & 255, rand() & 255);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cluster, single_color,
				boost::lexical_cast<string>(j));
	}

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

bool customRegionGrowing(const pcl::PointXYZRGBNormal& point_a,
		const pcl::PointXYZRGBNormal& point_b, float squared_distance) {
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal,
			point_b_normal = point_b.normal;
	if (fabs(point_a_normal.dot(point_b_normal))
			> cos(CUSTOM_REGION_GROWING_ANGLE_THRESH * CV_PI / 180))
		return (true);
	return (false);
}

template<typename PointT, typename PointNormalT>
void conditionalEuclideanSegmentation(
		typename pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals,
		pcl::IndicesClustersPtr clusters) {

//	pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters),
//			large_clusters (new pcl::IndicesClusters);

	std::cerr << "conditionalEuclideanSegmentation...\n", tt.tic();
	pcl::ConditionalEuclideanClustering<PointNormalT> cec(false);
	cec.setInputCloud(cloud_with_normals);
	cec.setConditionFunction(&customRegionGrowing);
	cec.setClusterTolerance(CONDITIONAL_EUCLIDEAN_SEG_TOLERANCE);
	cec.setMinClusterSize(CONDITIONAL_EUCLIDEAN_SEG_MINSIZE);
//	cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
	cec.segment(*clusters);
//	cec.getRemovedClusters (small_clusters, large_clusters);

//	typename pcl::PointCloud<PointT>::Ptr outCloud(new
//			typename pcl::PointCloud<PointT>);
//	colorCloud<pcl::PointXYZRGBNormal>(cloud_with_normals, outCloud,
//			clusters);
	cerr << "done: " << tt.toc() << " ms, " << clusters->size() << " clusters"
			<< endl;
}

template<typename PointT>
void colorCloud(typename pcl::PointCloud<PointT>::Ptr inCloud,
		typename pcl::PointCloud<PointT>::Ptr outCloud,
		pcl::IndicesClustersPtr clusters) {
	pcl::copyPointCloud(*inCloud, *outCloud);
	srand(time(NULL));
	for (size_t i = 0; i < clusters->size(); ++i) {
		int r = rand() % 255;
		int g = rand() % 255;
		int b = rand() % 255;
		for (size_t j = 0; j < (*clusters)[i].indices.size(); ++j) {
			setPointRGB(&(outCloud->points[(*clusters)[i].indices[j]]), r, g,
					b);
		}
	}
}

template<typename T>
void concatenateCloud(typename pcl::PointCloud<T>::Ptr c1,
		typename pcl::PointCloud<T>::Ptr c2) {
	c1->points.insert(c1->points.end(), c2->points.begin(), c2->points.end());
	c1->width = c1->size() + c2->size();
	c1->height = 1;
}

template<typename T>
void concatenateVectors(vector<T>& vector1, vector<T>& vector2) {
	vector1.insert(vector1.end(), vector2.begin(), vector2.end());
}

template<typename PointT>
void setPointRGB(PointT* p, uint8_t r, uint8_t g, uint8_t b) {
	uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
	p->rgb = *reinterpret_cast<float*>(&rgb);
}

template<typename PointT>
void getPointRGB(PointT p, uint8_t& r, uint8_t& g, uint8_t& b) {
// unpack rgb into r/g/b
	uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
	r = (rgb >> 16) & 0x0000ff;
	g = (rgb >> 8) & 0x0000ff;
	b = (rgb) & 0x0000ff;
}

void drawNormalSphere(pcl::PointCloud<pcl::Normal>::Ptr normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	for (unsigned int i = 0; i < normals->points.size(); i++) {
		pcl::PointXYZ pt(normals->points[i].normal_x,
				normals->points[i].normal_y, normals->points[i].normal_z);
		normal_cloud->push_back(pt);
	}
	displayCloud(normal_cloud);
}

template<typename PointT>
void extractCloud(typename pcl::PointCloud<PointT>::Ptr inCloud,
		typename pcl::PointCloud<PointT>::Ptr outCloud,
		pcl::PointIndices::Ptr inliers,
		typename pcl::PointCloud<PointT>::Ptr remainingCloud) {
	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(inCloud);
	extractor.setIndices(inliers);
	extractor.setNegative(false);
	extractor.filter(*outCloud);
	extractor.setNegative(true);
	extractor.filter(*remainingCloud);
}

template<typename PointNormalT>
void sampleConsensus(
		typename pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals,
		typename pcl::ModelCoefficients::Ptr coefficients) {

	cerr << cloud_with_normals->size() << " points" << endl;

	pcl::SACSegmentationFromNormals<PointNormalT, PointNormalT> seg;

	typename pcl::PointCloud<PointNormalT>::Ptr
	cloud_remaining(new typename pcl::PointCloud<PointNormalT>),
	cloud_inlier(new typename pcl::PointCloud<PointNormalT>);

	pcl::copyPointCloud(*cloud_with_normals, *cloud_remaining);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("hull"));

	int plane_ind = 0;
	while (cloud_remaining->size() > 100) {
		cout << cloud_remaining->size() << " points\n";

		seg.setInputNormals(cloud_remaining);
		seg.setInputCloud(cloud_remaining);
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.5);
		//	seg.setMaxIterations(1000);
		Eigen::Vector3f z_axis(0, 0, 1);
		seg.setAxis(z_axis);
		seg.setEpsAngle(CV_PI / 8);
//	seg.setNormalDistanceWeight(0.1);
//	seg.setMinMaxOpeningAngle(-CV_PI/8, CV_PI/8);
//	seg.setDistanceFromOrigin(10);

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		seg.segment(*inliers, *coefficients);

		extractCloud<PointNormalT>(cloud_remaining, cloud_inlier, inliers,
				cloud_remaining);

		pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> single_color(
				cloud_inlier, rand() & 255, rand() & 255, rand() & 255);
		plane_ind += 1;
		viewer->addPointCloud<PointNormalT>(cloud_inlier, single_color,
				boost::lexical_cast<string>(plane_ind));

		//	cerr << "coefficients:" << endl << *coefficients << endl;
		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}
	}
	while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

template<typename PointT>
void fitPlaneToCloud(typename pcl::PointCloud<PointT>::Ptr cloud_with_normals,
		pcl::PointIndices::Ptr inliers,
		pcl::ModelCoefficients::Ptr coefficients) {
	pcl::SACSegmentationFromNormals<PointT, PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	//	seg.setMaxIterations(1000);

	seg.setInputCloud(cloud_with_normals);
	seg.setInputNormals(cloud_with_normals);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset."
				<< std::endl;
		return;
	}
}

template<typename PointT>
void projectToPlane(typename pcl::PointCloud<PointT>::Ptr cloud_with_normals,
		typename pcl::PointCloud<PointT>::Ptr cloud_projected,
		pcl::ModelCoefficients::Ptr coefficients,
		typename pcl::PointCloud<PointT>::Ptr hull,
		std::vector<pcl::Vertices> hull_vertices) {

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	fitPlaneToCloud<PointT>(cloud_with_normals, inliers, coefficients);

	pcl::ProjectInliers<PointT> proj;
	pcl::ConvexHull<PointT> chull;

	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_with_normals);
	proj.setIndices(inliers);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_projected_filtered2(
//			new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PassThrough<pcl::PointXYZRGB> pass;
//	pass.setInputCloud(cloud_plane_projected_filtered);
//	pass.setFilterFieldName("x");
//	pass.setFilterLimits(0, 10);
//	pass.filter(*cloud_plane_projected_filtered2);
//	std::cerr << "PointCloud after filtering has: "
//			<< cloud_plane_projected_filtered2->points.size() << " data points."
//			<< std::endl;

	chull.setInputCloud(cloud_projected);
//		chull.setAlpha(0.1);
	chull.reconstruct(*hull, hull_vertices);
}

//	unsigned int ptSize = normals->points.size();
//
//	cv::Mat samples(ptSize, 3, CV_32F, (void*)&normals->points[0]);
//	int clusterCount = 3;
//	Mat labels;
//	int attempts = 5;
//	Mat centers;
//	kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 0.0001, 10000),
//			attempts, KMEANS_RANDOM_CENTERS, centers );
//	cout << centers;
////	Eigen::Matrix<float,2,3> eigenT;
////	cv::cv2eigen(centers,eigenT);
//	Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor> > eigenT ((float*)centers.data);
//
//	vector<Eigen::Vector4f> centroids(clusterCount);
//	for (int i=0; i<clusterCount; i++) {
//		centroids.at(i) = Eigen::Vector4f (eigenT(i,0),eigenT(i,1),eigenT(i,2),0.0f);
//	}
//	cout << pcl::getAngle3D(centroids[0],centroids[1]) << endl;
//	cout << pcl::getAngle3D(centroids[1],centroids[2]) << endl;
//	cout << pcl::getAngle3D(centroids[2],centroids[0]) << endl;

//	pcl::Kmeans km = pcl::Kmeans(ptSize, 3);
//	km.setClusterSize(3);
//
//	vector<vector<float> > normalPts;
//	for (pcl::PointCloud<pcl::Normal>::iterator it = normals->begin();
//			it != normals->end(); ++it) {
//		vector<float> thisPts(it->normal, it->normal + 3 * sizeof(float));
//		normalPts.push_back(thisPts);
//	}
//	km.setInputData(normalPts);
//	km.initialClusterPoints();
//
//	for (int i=0; i<100; i++) {
//		km.computeCentroids();
//	}
//
//	vector<pcl::Kmeans::Point> centroids = km.get_centroids();
//	for (vector<pcl::Kmeans::Point>::const_iterator it = centroids.begin();
//			it != centroids.end(); it++) {
//		copy(it->begin(), it->end(), ostream_iterator<float>(cout, "\t"));
//		cout << endl;
//	}

#endif /* STAIRSPOINTCLOUD_HPP_ */
