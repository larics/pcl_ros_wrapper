#include <pcl_ros_wrapper/segmentation/sac.hpp>

void pcl_ros_wrapper::segmentation::crop_ground_plane(
  PointCloudT::Ptr&             input,
  const plane_detection_params& params,
  double                        ground_depth,
  bool                          verbose)
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  auto                                inliers = boost::make_shared<pcl::PointIndices>();
  auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
  auto cloud_plane  = boost::make_shared<PointCloudT>();
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params.ransac_iterations);
  seg.setDistanceThreshold(params.distance_threshold);
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  if (coefficients->values[2] < 0.9) {
    if (verbose) { std::cout << "Biggest surface is NOT ground!\n"; }
    return;
  }

  Eigen::Vector4f minPoint, maxPoint;
  pcl::getMinMax3D(*input, minPoint, maxPoint);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input, *inliers, centroid);
  maxPoint[2]     = centroid[2] + ground_depth;
  auto cropPoints = filters::do_crop_box_indices(input, minPoint, maxPoint);

  auto                               temp_cloud = boost::make_shared<PointCloudT>();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input);
  extract.setIndices(cropPoints);
  extract.setNegative(true);
  extract.filter(*temp_cloud);
  *input = *temp_cloud;
}

void pcl_ros_wrapper::segmentation::iterative_ground_plane_filter(
  PointCloudT::Ptr&             input,
  const plane_detection_params& params,
  int                           max_iters_plane_removal,
  bool                          verbose)
{
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  auto                                inliers = boost::make_shared<pcl::PointIndices>();
  auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
  auto cloud_plane  = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params.ransac_iterations);
  seg.setDistanceThreshold(params.distance_threshold);

  int  i          = 0;
  int  nr_points  = static_cast<int>(input->size());
  auto temp_cloud = boost::make_shared<PointCloudT>();
  while (input->size() > 0.3 * nr_points && i < max_iters_plane_removal) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
      std::cout << "Could not estimate a planar model for the given dataset."
                << std::endl;
      break;
    }

    // If the biggest planne is not a ground plane anymore stop removing
    if (coefficients->values[2] < 0.9) {
      if (verbose) { std::cout << "Biggest surface is NOT ground!\n"; }
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    if (verbose) {

      std::cout << i << ". [" << coefficients->values[0] << ", "
                << coefficients->values[1] << ", " << coefficients->values[2] << ", "
                << coefficients->values[3] << "]\n";

      std::cout << "PointCloud representing the planar component: " << cloud_plane->size()
                << " data points." << std::endl;
    }

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*temp_cloud);
    *input = *temp_cloud;
    i++;
  }
}
