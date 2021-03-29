#include <pcl_ros_wrapper/registration/icp.hpp>

using namespace pcl_ros_wrapper;

registration::icp_info do_xy_yaw_icp(
  const PointCloudT::ConstPtr&    source,
  const PointCloudT::ConstPtr&    target,
  const registration::icp_params& params,
  bool                            verbose = false,
  const Eigen::Matrix4f&          guess   = Eigen::Matrix4f::Identity())
{
  auto transformationEstimation =
    boost::make_shared<registration::TransformationEstimationLM>();
  auto warpFunction = boost::make_shared<registration::Warp2DYaw>();
  transformationEstimation->setWarpFunction(warpFunction);
  return registration::do_icp(
    source, target, params, transformationEstimation, verbose, guess);
}

registration::icp_info do_icp(
  const PointCloudT::ConstPtr&                                                 source,
  const PointCloudT::ConstPtr&                                                 target,
  const registration::icp_params&                                              params,
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::TransformationEstimationPtr estimation,
  bool                   verbose = false,
  const Eigen::Matrix4f& guess   = Eigen::Matrix4f::Identity())
{
  if (params.maximum_iterations == 0) {
    return registration::icp_info{
      boost::make_shared<PointCloudT>(*target), true, Eigen::Matrix4f::Identity(), 0
    };
  }

  pcl::console::TicToc timer;
  timer.tic();
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(params.maximum_iterations);
  icp.setRANSACOutlierRejectionThreshold(params.ransac_outlier_rejection_threshold);
  icp.setMaxCorrespondenceDistance(params.max_correspondence_distance);
  icp.setRANSACIterations(100);

  if (estimation) { icp.setTransformationEstimation(estimation); }

  icp.setInputSource(source);
  icp.setInputTarget(target);

  auto aligned = boost::make_shared<PointCloudT>();
  icp.align(*aligned, guess);

  auto msec = timer.toc();

  if (verbose) {
    std::cout << "\n-----------------------------------\n";
    std::cout << "ICP has converged:" << icp.hasConverged()
              << "with score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    std::cout << "ICP took: " << msec << " milliseconds" << std::endl;
  }

  return registration::icp_info{
    aligned, icp.hasConverged(), icp.getFinalTransformation(), icp.getFitnessScore()
  };
}

registration::icp_info do_xyz_yaw_icp(
  const PointCloudT::ConstPtr&    source,
  const PointCloudT::ConstPtr&    target,
  const registration::icp_params& params,
  bool                            verbose = false,
  const Eigen::Matrix4f&          guess   = Eigen::Matrix4f::Identity())
{

  auto transformationEstimation =
    boost::make_shared<registration::TransformationEstimationLM>();
  auto warpFunction = boost::make_shared<registration::Warp3D>();
  transformationEstimation->setWarpFunction(warpFunction);
  return registration::do_icp(
    source, target, params, transformationEstimation, verbose, guess);
}

registration::icp_info do_xy_icp(
  const PointCloudT::ConstPtr&    source,
  const PointCloudT::ConstPtr&    target,
  const registration::icp_params& params,
  bool                            verbose = false,
  const Eigen::Matrix4f&          guess   = Eigen::Matrix4f::Identity())
{

  auto transformationEstimation =
    boost::make_shared<registration::TransformationEstimationLM>();
  auto warpFunction = boost::make_shared<registration::Warp2D>();
  transformationEstimation->setWarpFunction(warpFunction);
  return registration::do_icp(
    source, target, params, transformationEstimation, verbose, guess);
}

registration::icp_info do_yaw_icp(
  const PointCloudT::ConstPtr&    source,
  const PointCloudT::ConstPtr&    target,
  const registration::icp_params& params,
  bool                            verbose = false,
  const Eigen::Matrix4f&          guess   = Eigen::Matrix4f::Identity())
{

  auto transformationEstimation =
    boost::make_shared<registration::TransformationEstimationLM>();
  auto warpFunction = boost::make_shared<registration::WarpYaw>();
  transformationEstimation->setWarpFunction(warpFunction);
  return registration::do_icp(
    source, target, params, transformationEstimation, verbose, guess);
}