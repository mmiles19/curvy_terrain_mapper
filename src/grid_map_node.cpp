#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace gm = ::grid_map::grid_map_pcl;

class GridMapNode {
public:
  grid_map::GridMapPclLoader gridMapPclLoader;
  bool cloudUpdated;
  ros::Subscriber inputPub;
  ros::Publisher outputPub;
  ros::Timer processTimer;
  ros::NodeHandle nh;

public:
  GridMapNode(ros::NodeHandle& nodeh) : nh(nodeh)
  {
    cloudUpdated=false;
    inputPub = nh.subscribe("input", 1, &GridMapNode::callback, this);
    outputPub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    processTimer = nh.createTimer(ros::Duration(1.0), &GridMapNode::loop, this);
  }

  inline void callback(const sensor_msgs::PointCloud2& input_msg)
  {
    ROS_INFO("Got cloud!");
    cloudUpdated=true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::fromROSMsg(input_msg, *input_cloud);
    gridMapPclLoader.setInputCloud(input_cloud);
    ROS_INFO("Done w cloud");
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