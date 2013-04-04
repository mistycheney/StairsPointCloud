/*
 * disparityMap.hpp
 *
 *  Created on: Mar 18, 2013
 *      Author: yuncong
 */

#ifndef DISPARITYMAP_HPP_
#define DISPARITYMAP_HPP_

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

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

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

using namespace cv;
using namespace std;

int disparityMap(int imgId);


#endif /* DISPARITYMAP_HPP_ */
