/*
 * stairsPointCloud.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: yuncong
 */

#include "StairsPointCloud.hpp"
#include "disparityMap.hpp"
#include "canny.hpp"

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGBNormal pn_t;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_t;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr_t;

int main(int argc, char** argv) {

	disparityMap(5);

//	find_edges();
//	return 0;

	int imId = 5;
//	findLineSegments(imId);
//	return 0;

	cv::namedWindow("dummy", 1);
	cv::destroyAllWindows();

	// ---------- load point cloud -------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	string cloud_name = str(boost::format("cloud%d.pcd") % imId);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloud_name, *cloud) == -1) {
		PCL_ERROR("Couldn't read file stairs_cloud.pcd \n");
		return (-1);
	}

//	displayRGBCloud(cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	removeOutlier<pcl::PointXYZRGB>(cloud, cloud_filtered);
//	displayRGBCloud(cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	voxelDownsample<pcl::PointXYZRGB>(cloud_filtered, cloud_down);
	displayRGBCloud<pcl::PointXYZRGB>(cloud_down);

//	std::vector<pcl::PointIndices> cluster_indices;
//	euclideanClustering<pcl::PointXYZRGB>(cloud_down, cluster_indices);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	estimateNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_down, normals);
	string normal_name = str(boost::format("normal%d.pcd") % imId);
	pcl::io::savePCDFile(normal_name, *normals);
//	pcl::io::loadPCDFile<pcl::Normal>(normal_name, *normals);

	cloud_ptr_t cloud_with_normals(new cloud_t);
	pcl::copyPointCloud(*cloud_down, *cloud_with_normals);
	pcl::copyPointCloud(*normals, *cloud_with_normals);
	displayCloudRGBNormals(cloud_with_normals);

//	return 0;

//	regionGrowingSegmentation<pcl::PointXYZRGB, pcl::Normal>(
//			cloud_down, normals, cluster_indices);
//	drawNormalSphere(normals);

	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
	conditionalEuclideanSegmentation<pcl::PointXYZRGB, pn_t>(
			cloud_with_normals, clusters);

	cloud_ptr_t cloud_colored(new cloud_t);
	colorCloud<pn_t>(cloud_with_normals, cloud_colored, clusters);
	displayRGBCloud<pn_t>(cloud_colored);

//	cloud_ptr_t cloud_plane(new cloud_t);
//	cloud_ptr_t cloud_projected(new cloud_t);
//	cloud_ptr_t hull(new cloud_t);
//	std::vector<pcl::Vertices> hull_vertices;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

//	pcl::PointCloud<pcl::Normal>::Ptr plane_normals(
//			new pcl::PointCloud<pcl::Normal>);
//	cloud_ptr_t cloud_large_patches(new cloud_t);

//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
//					new pcl::visualization::PCLVisualizer("hull"));

//	pcl::ModelCoefficients::Ptr sc_coeffs(new pcl::ModelCoefficients);
//	sampleConsensus<pn_t>(cloud_with_normals, sc_coeffs);

//	for (size_t i = 0; i < clusters->size(); i++) {
//		pcl::PointIndicesPtr one_cluster(new pcl::PointIndices);
//		one_cluster->indices = (*clusters)[i].indices;
//
//		extractCloud<pn_t>(cloud_colored, cloud_plane, one_cluster);
//
//		projectToPlane<pn_t>(cloud_plane, cloud_projected,
//				coefficients, hull, hull_vertices);
//
////		concatenateCloud<pn_t>(cloud_large_patches, cloud_plane);
//		pcl::visualization::PointCloudColorHandlerRGBField<pn_t> rgb(cloud_projected);
//		viewer->addPointCloud<pn_t>(cloud_projected, rgb,
//				boost::lexical_cast<string>(i));
//		viewer->addPolygon<pn_t>(hull, 255, 255, 255,
//				boost::lexical_cast<string>(i));
//
//		pcl::Normal plane_normal(coefficients->values[0],
//				coefficients->values[1],
//				coefficients->values[2]);
//		plane_normals->push_back(plane_normal);
//	}

//	while (!viewer->wasStopped()) {
//			viewer->spinOnce(100);
//			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//		}

//	displayRGBCloud<pn_t>(cloud_large_patches);

//	cloud_ptr_t cloud_large_patches_inlier(new cloud_t);
////	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	sampleConsensus<pn_t>(cloud_large_patches, cloud_large_patches_inlier, coefficients);
//	displayRGBCloud<pn_t>(cloud_large_patches_inlier);

//	drawNormalSphere(plane_normals);

	return 0;

}
