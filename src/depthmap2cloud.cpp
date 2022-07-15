#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "depthmap2pointcloud.h"
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>
#include <cassert>

static const std::string OPENCV_WINDOW = "Image window";

class DepthMap2PointCloud
{
  ros::NodeHandle nh_;
  ros::Publisher pcl_pub_; // publisher for the point cloud
  ros::Publisher PoseWithCovariance; // publishes the pose with covariance for constructing the map.
  ros::Subscriber Pose; // receives the pose from the rosbag
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  PointCloud pcloud; // custom data-type.
  depthmap2pcloud depcl; // from header
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  tf2_ros::Buffer Buffer_; // for transforming the PointCloud
  tf2_ros::TransformListener* tfListener;
  tf2_ros::TransformBroadcaster tfbcast; // for converting the PoseStampedPtr -> Transform
  geometry_msgs::PoseStampedPtr pcl_conv; // will be used to transform PointCloud
public:
  DepthMap2PointCloud()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/os_node/llol_odom/pano", 1,
    &DepthMap2PointCloud::imageCb, this);
    Pose = nh_.subscribe("/os_node/llol_odom/pose", 9, &DepthMap2PointCloud::poseCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1); //optional
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points", 1);
    PoseWithCovariance = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1); // this is pose_with_covariance
      
    tfListener = new tf2_ros::TransformListener(Buffer_);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~DepthMap2PointCloud()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
	
  void poseCallback(const geometry_msgs::PoseStampedPtr& msg)  // thin wrapper PoseStamped -> PoseStampedWithCovariance
  {
    boost::array<float, 36> covariance = {0}; // initializes with all zeros
    pose_msg.pose.pose.position.x = msg->pose.position.x;
    pose_msg.pose.pose.position.y = msg->pose.position.y;
    pose_msg.pose.pose.position.z = msg->pose.position.z;
    pose_msg.pose.pose.orientation.x = msg->pose.orientation.x;
    pose_msg.pose.pose.orientation.y = msg->pose.orientation.y;
    pose_msg.pose.pose.orientation.z = msg->pose.orientation.z;
    pose_msg.pose.pose.orientation.w = msg->pose.orientation.w;
    pose_msg.header = msg->header; // same timestamps.
    // pose_msg.header.frame_id = msg->header.frame_id; // same meta information
    //ROS_INFO("POSE timestamp : [%f]", pose_msg.header.stamp.toSec());
    
    pose_msg.pose.covariance = covariance;
    this->pcl_conv = msg; // saves this pose.
    PoseWithCovariance.publish(pose_msg); // publish the pose for elevation mapping.
    
  }
  
  
  
  void generatePointCloud(PointCloud& pcloud, pcl::PointCloud<pcl::PointXYZ>& publishCloud ) // converts from struct PointCloud -> sensor_msgs/PointCloud2
  {
    int num_points = pcloud.x.rows();
     // std::cout<<"Num Points : "<<num_points<<'\n';
    
    for (int i = 0; i < pcloud.x.rows(); i++)
    {
     pcl::PointXYZ point;
     point.x = pcloud.x(i);
     point.y = pcloud.y(i);
     point.z = pcloud.z(i);
     publishCloud.push_back(point);
    }
  }
	
	
  void cv_u16c2_to_cv_u16c1(cv::Mat& image, cv::Mat& depthImage)
  {
	
	cv::Mat _ignore(image.rows, image.cols, CV_16UC1); // this is not required
	cv::Mat holder[] = {depthImage, _ignore};
	int from_to[] = {0,0 , 1,1}; // image[0] -> depthImage[0] && image[1] -> _ignore[1]
  cv::mixChannels(&image, 1, holder, 2, from_to, 2);
  }
  
  void transformPointCloud2RobotFrame(sensor_msgs::PointCloud2 transformedCloud, sensor_msgs::PointCloud2& pcloud,
                                     tf2_ros::TransformBroadcaster& broadcaster)
  {
      geometry_msgs::PoseStampedPtr msg;
      // geometry_msgs::TransformStamped transformStamped = Buffer_.lookupTransform("odom", "pano", ros::Time(0));
      msg = this->pcl_conv; // uses the previously saved pose.
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header = msg->header;
      transformStamped.child_frame_id = "pano";
      transformStamped.transform.translation.x = msg->pose.position.x;
      transformStamped.transform.translation.y = msg->pose.position.y;
      transformStamped.transform.translation.z = msg->pose.position.z;
      transformStamped.transform.rotation.x = msg->pose.orientation.x;
      transformStamped.transform.rotation.y = msg->pose.orientation.y;
      transformStamped.transform.rotation.z = msg->pose.orientation.z;
      transformStamped.transform.rotation.w = msg->pose.orientation.w;
      
      tf2::doTransform(transformedCloud, pcloud, transformStamped); // this converts the point cloud from lidar frame -> robot frame
      broadcaster.sendTransform(transformStamped);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC2);
	  // std::cout<<"Successfully converted image";
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat image(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC2, cv_ptr->image.data);
	cv::Mat depthImage(image.rows, image.cols, CV_16UC1);
	  
	DepthMap2PointCloud::cv_u16c2_to_cv_u16c1(image, depthImage); // extracts the range image
    // Update GUI Window
	
	pcloud = depcl.convert2pointcloud(depthImage);
  
	pcl::PointCloud<pcl::PointXYZ> publishCloud; // holds the point cloud for publishing
  generatePointCloud(pcloud, publishCloud);
  
  cv::imshow(OPENCV_WINDOW, depthImage); // can use for visualization.
  cv::waitKey(3);
	
	
    
  // Output modified video stream
  sensor_msgs::PointCloud2 pcloud_msg;
  pcl::toROSMsg(publishCloud, pcloud_msg); 
  
   
  pcloud_msg.header = msg->header;
  pcloud_msg.header.stamp.sec = msg->header.stamp.sec;
  pcloud_msg.header.frame_id = msg->header.frame_id;
  pcloud_msg.header.stamp.nsec = msg->header.stamp.nsec;
  pcl_pub_.publish(pcloud_msg);
  //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depthmap2cloud");
  DepthMap2PointCloud dcl;
  ros::spin();
  return 0;
}