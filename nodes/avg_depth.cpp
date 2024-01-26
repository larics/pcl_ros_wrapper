#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros_wrapper/filters/crop_box.hpp>
#include <math.h>
#include <tf/tf.h>

double roll = 0;
double pitch = 0;
double yaw = 0;
double h = 0;
double rate = 5;
const auto pclCloud = boost::make_shared<pcl_ros_wrapper::PointCloudT>();
ros::Publisher alt_pub;

void avgDepthPubCallback(const ros::TimerEvent& event)
{
	//get box
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pclcloud (new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f centr (0.0, 0.0, 0.0, 0.0);
  Eigen::Vector4f min (-1, -1, 0, 0.0);
  Eigen::Vector4f max (1, 1, 15, 0.0);
	Eigen::Vector3f rpy (-roll, -pitch, 0.0);
	
  auto cropped_pclcloud = pcl_ros_wrapper::filters::do_local_crop_box(pclCloud, centr, min, max, rpy);

  double sum = 0.0;
  int n = 0;
	for(const auto& point:cropped_pclcloud->points)
    {
        n++;
        sum += point.z;
    }
  h = sum/n;
  // ROS_INFO_STREAM("[AVG DEPTH] The height is: "<<h);
  std_msgs::Float32 avg_height_msg;
  avg_height_msg.data = h;
  alt_pub.publish(avg_height_msg);

}


void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr)
{
	pcl::fromROSMsg(*cloud_ptr, *pclCloud);
	std::vector<int> v;
	pcl::removeNaNFromPointCloud(*pclCloud, *pclCloud, v);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

  roll = 0;
  pitch = 0;
  yaw = 0;

  tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_depth");
  // ros::NodeHandle private_node("~");
  
  ros::NodeHandle node;
  node.getParam("/camera_depth/rate", rate); 
  ros::Subscriber imu_sub = node.subscribe("mavros/imu/data", 10, &imuCallback);
  ros::Subscriber pcl_sub = node.subscribe("camera/depth_registered/points", 10, &pclCallback);
  alt_pub = node.advertise<std_msgs::Float32>("average_depth", 1);
  ros::Timer avg_depth_timer = node.createTimer(ros::Duration(1/rate), avgDepthPubCallback);

  ros::spin();
  return 0;

  
};