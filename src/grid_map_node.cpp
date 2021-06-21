#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <grid_map_core/GridMapMath.hpp>
#include <pcl/common/io.h>
#include <ros/console.h>

#include <pcl_ros/point_cloud.h> 
#include <pcl/common/projection_matrix.h> 
#include <pcl/io/openni_grabber.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/console/parse.h>

#include <ros/ros.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_ros/transforms.h>

namespace gm = ::grid_map::grid_map_pcl;

class GridMapNode {
public:
  grid_map::GridMapPclLoader gridMapPclLoader;
  bool cloudUpdated;
  ros::Subscriber inputPub;
  ros::Publisher outputPub;
  ros::Timer processTimer;
  ros::NodeHandle nh;
  // convert pointcloud2 msgs to PCLPC2
  //pcl::PointCloud::Ptr cloud = new 

public:
  GridMapNode(ros::NodeHandle& nodeh) : nh(nodeh)
  {
    ROS_INFO("Something's happening @ GridMapNode class");
    cloudUpdated=false;
    inputPub = nh.subscribe("/pcdp_output", 1, &GridMapNode::callback, this);
    outputPub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    processTimer = nh.createTimer(ros::Duration(1.0), &GridMapNode::loop, this);
  }

  inline void callback(const sensor_msgs::PointCloud2& input_msg)
  {
    ROS_INFO("Got cloud!");
    cloudUpdated=true;
    
    // I think there's an issue here, commented this out
    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    // pcl::fromROSMsg(input_msg, *input_cloud); // this is where it crashes
    
    // must convert to a type that setInputCloud can use --> pcl::PointCloud<pcl::PointXYZ>::Ptr
    // create the containers
    pcl::PCLPointCloud2 cloudBlob;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
    // Convert to PCL data type
    pcl_conversions::toPCL(input_msg, cloudBlob);
    //ROS_INFO("@ callback, just converted to PCLPointCloud2 (pcl_conversions::toPCL)");
    // Convert to final type
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    //ROS_INFO("@ callback, just converted to PointCloud::Ptr (pcl::fromPCLPointCloud2)");
    
    gridMapPclLoader.setInputCloud(cloud);
    //ROS_INFO("@ callback, sent converted msg to setInputCloud!");
  }

  inline void loop(const ros::TimerEvent& event) {
    if (cloudUpdated) {
      ROS_INFO("Publishing cloud!");
      cloudUpdated=false;
      gm::processPointcloud(&gridMapPclLoader, nh);
      // grid_map_msgs::GridMap msg;
      grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
      gridMap.setFrameId(gm::getMapFrame(nh));
      sensor_msgs::PointCloud2 output_msg;
      grid_map::GridMapRosConverter::toPointCloud(gridMap, "flat", output_msg);
      outputPub.publish(output_msg);
      ROS_INFO("Done w publishing");
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_node");
  ros::NodeHandle nodeh("~");
  // gm::setVerbosityLevelToDebugIfFlagSet(nh);

  GridMapNode gmn(nodeh);

  ROS_INFO("Initialized.");

  ros::spin();
}
