#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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


void pcl_ros_wrapper::segmentation::crop_ground_plane_smart(
  PointCloudT::Ptr&             input,
  const plane_detection_params& params,
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

  auto                               temp_cloud = boost::make_shared<PointCloudT>();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
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

std::vector<pcl_ros_wrapper::PointCloudT::Ptr>
  pcl_ros_wrapper::segmentation::do_euclidean_clustering(const PointCloudT::Ptr& input,
                                                         const cluster_params&   params,
                                                         bool                    verbose)
{
  if (input->empty()) { return std::vector<PointCloudT::Ptr>(); }

  // Creating the KdTree object for the search method of the extraction
  auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(input);

  std::vector<pcl::PointIndices>                 cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(params.cluster_tolerance);// 2cm
  ec.setMinClusterSize(params.min_cluster_size);
  ec.setMaxClusterSize(params.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  std::vector<PointCloudT::Ptr> output;
  for (const auto& point_indices : cluster_indices) {
    auto cloud_cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto& point_index : point_indices.indices) {
      cloud_cluster->push_back((*input)[point_index]);//*
    }
    cloud_cluster->width    = cloud_cluster->size();
    cloud_cluster->height   = 1;
    cloud_cluster->is_dense = true;
    output.emplace_back(cloud_cluster);

    if (verbose) {
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size()
                << " data points." << std::endl;
    }
  }

  return output;
}

pcl_ros_wrapper::PointCloudT::Ptr pcl_ros_wrapper::segmentation::project_to_plane(
  const PointCloudT::Ptr&            cloud,
  const pcl::ModelCoefficients::Ptr& coefficients)
{
  auto projected_cloud = boost::make_shared<PointCloudT>();

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_cloud);

  return projected_cloud;
}
