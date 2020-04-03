// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <curvy_terrain_mapper/preanalysis.h>
#include <curvy_terrain_mapper/regions.h>
#include <curvy_terrain_mapper/regiongrowing.h>
#include <curvy_terrain_mapper/voxSAC.h>
#include <curvy_terrain_mapper/splitmerge.h>

#include <curvy_terrain_mapper/ros_functions.hpp>

#include <dynamic_reconfigure/server.h>
#include <curvy_terrain_mapper/CurvyTerrainMapperConfig.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <colormap/palettes.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

CurvyTerrainMapperParams params_;

bool busy;

// std::string config_filepath_;
// YAML::Node config_;
// std::string fixed_frame_id_;
ros::Time stamp_;
ros::Publisher main_cloud_pub_;
ros::Publisher normal_cloud_pub_;
ros::Publisher curvature_pub_;
ros::Publisher costmap_pub_;
ros::Publisher costmap3d_pub_;
ros::Publisher seg_regions_pub_;

inline void inputCB(const sensor_msgs::PointCloud2& input_msg)
{
    if(!ros::ok()){ return; }
    if(busy){ return; }
    uint count;

    busy = true;

    stamp_ = input_msg.header.stamp;

// Loading input point cloud //

	double loadS = pcl::getTime();

    PointCloudT::Ptr mainCloud;
	mainCloud.reset (new PointCloudT);

    pcl::fromROSMsg(input_msg, *mainCloud);

// Starting preanalysis //

    ROS_INFO("---");
    ROS_INFO("Received point cloud with %d points",mainCloud->width);

	ROS_INFO("Starting preanalysis");
    double preAS = pcl::getTime();

    Preanalysis pre;
    pre.loadConfig(params_.preanalysis);
    NormalCloud::Ptr normalCloud;
    normalCloud.reset(new NormalCloud);
    PointCloudT floorPC;
    PointCloudC coloredNormalCloud;

    Eigen::Matrix4d T_fixed_input_mat = Eigen::Matrix4d::Identity();
    if (input_msg.header.frame_id != params_.fixed_frame_id)
        getTransformFromTree(params_.fixed_frame_id, input_msg.header.frame_id, &T_fixed_input_mat/*, stamp_*/);
    // Eigen::Matrix4d invTransformCloud = transformCloud.inverse();

    // transform mainCloud (default identity), 
    // downsample (default true, 0.01m res), 
    // normal estimation and init normalCloud with output, 
    // filter ghost/shadow points (nonexistent points at discontinuities due to sensor noise), 
    // extract floor and init floorPC with output, 
    // init prepNorMap with xyz's of mainCloud points and rgb's of mainCloud normals
    pre.run(mainCloud, normalCloud, coloredNormalCloud, floorPC, T_fixed_input_mat);
    double preAE = pcl::getTime();
    ROS_INFO("Preanalysis took: %f",preAE-preAS);

    pubMainCloud(&main_cloud_pub_, *mainCloud, params_.fixed_frame_id, stamp_);
    pubNormalCloud(&normal_cloud_pub_, *mainCloud, *normalCloud, params_.fixed_frame_id, stamp_);
    pubCurvature(&curvature_pub_, *mainCloud, *normalCloud, params_.fixed_frame_id, stamp_);
    
// Starting segmentation //

    ROS_INFO("Starting segmentation");
    double segS = pcl::getTime();
    regions segRegions;
    int segMode = params_.segmentationmode;
    switch (segMode) 
    {
        case 0:
        {
            ROS_INFO("Using Region Growing algorithm");
            RegionGrowing reGrow;
            reGrow.loadConfig(params_.regiongrowing);
            reGrow.setInputCloud(mainCloud);
            reGrow.setNormalCloud(normalCloud);
            // extract and init segRegions with smooth regions
            reGrow.run(segRegions);
            break;
        }
        case 1:
        {
            ROS_INFO("Using Voxel SAC algorithm");
            voxSAC voxelSAC;
            voxelSAC.setInputCloud(mainCloud);
            voxelSAC.setNormalCloud(normalCloud);
            voxelSAC.run(segRegions);
            break;
        }
        case 2:
        {
            ROS_INFO("Using Split & Merge algorithm");
            splitMerge sam;
            sam.setInputCloud(mainCloud);
            sam.setNormalCloud(normalCloud);
            sam.splitProcess();
            sam.mergeProcess(segRegions);
            break;
        }
    }
    double segE = pcl::getTime();
    ROS_INFO("Segmentation found %d regions",segRegions.size());
    ROS_INFO("Segmentation took: %f",segE-segS);

    pubRegions(&seg_regions_pub_,segRegions, params_.fixed_frame_id, stamp_);

    // generate 2D projected curvature map
    float map_min_x = 0.f, map_max_x = 4.f, map_min_y = -1.5f, map_max_y = 1.5f, resolution = 0.05;
    uint map_size_x = (map_max_x-map_min_x)/resolution, map_size_y = (map_max_y-map_min_y)/resolution;

    // std_msgs::Float64MultiArray costmap_msg; // should of used a sensor_msgs::Image
    // costmap_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // costmap_msg.layout.dim[0].size = map_size_x;
    // costmap_msg.layout.dim[0].stride = map_size_x*map_size_y;
    // costmap_msg.layout.dim[0].label = "x";
    // costmap_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // costmap_msg.layout.dim[1].size = map_size_y;
    // costmap_msg.layout.dim[1].stride = map_size_y;
    // costmap_msg.layout.dim[1].label = "y";
    // costmap_msg.data.clear();

    // assert(mainCloud->size() == normalCloud->size());
    // std::vector<double> costmap[map_size_x*map_size_y];
    // float curvature_gain = 20;
    // float normal_gain = 5;
    // Eigen::Vector3f upVec(0,0,1);
    // double min_cost = INFINITY, max_cost = -INFINITY;
    // for(size_t pointIdx = 0; pointIdx < normalCloud->size(); pointIdx++)
    // {
    //     for (uint d1=0; d1<map_size_y; d1++)
    //     {
    //         for (uint d0=0; d0<map_size_x; d0++)
    //         {
    //             PointT thisPt = mainCloud->at(pointIdx);
    //             if (thisPt.x > d0*resolution+map_min_x && thisPt.x < (d0+1)*resolution+map_min_x)
    //             {
    //                 if (thisPt.y > d1*resolution+map_min_y && thisPt.y < (d1+1)*resolution+map_min_y)
    //                 {
    //                     Normal thisNm = normalCloud->at(pointIdx);
    //                     Eigen::Vector3f nmVec(thisNm.normal_x, thisNm.normal_y, thisNm.normal_z);
    //                     double cost = 0;
    //                     cost += fabs(normal_gain*(1/upVec.dot(nmVec) - 1));
    //                     cost += fabs(curvature_gain*thisNm.curvature);
    //                     // cost += fabs(curvature_gain*(cbrt(thisNm.curvature)));
    //                     if (cost > max_cost) { max_cost = cost; }
    //                     if (cost < min_cost) { min_cost = cost; }
    //                     costmap[d1*map_size_x + d0].push_back(cost);
    //                     // ROS_INFO("cost %f", costmap[d1*map_size_x + d0][0]);
    //                 }
    //             }
    //         }
    //     }
    // }
    // // ROS_INFO("Costmap min %f max %f", min_cost, max_cost);
    // for(uint i=0; i < map_size_x*map_size_y; i++)
    // {
    //     double cost = 0;
    //     for (uint j=0; j<costmap[i].size(); j++)
    //     {
    //         cost += costmap[i][j];
    //     }
    //     cost = cost/(double)costmap[i].size();
    //     // ROS_INFO("cost %f sum %f size %f",cost,(double)std::accumulate(costmap[i].begin(), costmap[i].end(), 0),(double)costmap[i].size());
    //     if (costmap[i].size()>0)
    //         costmap_msg.data.push_back(cost);
    //     else
    //         costmap_msg.data.push_back((double)0.0);
    // }

    // // pub costmap (2d)
    // {
    //     // auto getInterceptInAnotherRange = [](auto x, auto Xstart, auto Xend, auto Ystart, auto Yend) { return (Yend-Ystart)/(Xend-Xstart)*x + (Ystart*Xend-Yend*Xstart)/(Xend-Xstart); };
    //     visualization_msgs::Marker cost_msg;
    //     cost_msg.header.frame_id = params_.fixed_frame_id;
    //     cost_msg.header.stamp = stamp_;
    //     cost_msg.type = cost_msg.POINTS;
    //     cost_msg.action = cost_msg.ADD;
    //     cost_msg.scale.x = resolution;
    //     cost_msg.scale.y = resolution;
    //     cost_msg.scale.z = resolution;
    //     cost_msg.lifetime = ros::Duration(0.0);
    //     for (uint d1=0; d1<map_size_y; d1++)
    //     {
    //         for (uint d0=0; d0<map_size_x; d0++)
    //         {
    //             geometry_msgs::Point pt_msg;
    //             pt_msg.x = d0*resolution+map_min_x;
    //             pt_msg.y = d1*resolution+map_min_y;
    //             pt_msg.z = 0;
    //             cost_msg.points.push_back(pt_msg);

    //             double cost = costmap_msg.data[d0+d1*map_size_x];
    //             // ROS_INFO("Cost %f x %f y %f", costmap_msg.data[d0+d1*map_size_x], d0*resolution+map_min_x, d1*resolution+map_min_y);
    //             max_cost  = 10.0;
    //             cost = std::min(max_cost, std::max(min_cost, cost));
    //             double min_color_val = 0.f;
    //             double max_color_val = 1.f;
    //             auto a=min_cost, b=max_cost, c=min_color_val, d=max_color_val;
    //             double color_ratio = (d-c)/(b-a)*cost + (c*b-d*a)/(b-a);
    //             std_msgs::ColorRGBA rgb_msg;
    //             RGBColor color = getRGBColor(color_ratio); 
    //             rgb_msg.r = color.r;
    //             rgb_msg.g = color.g;
    //             rgb_msg.b = color.b;
    //             rgb_msg.a = 0.1;
    //             cost_msg.colors.push_back(rgb_msg);
    //         }
    //     }
    //     costmap_pub_.publish(cost_msg);
    //     ros::spinOnce();
    // }

    // // pub costmap 3d projection
    // {
    //     float scale = 0.005;
    //     visualization_msgs::Marker cost_msg;
    //     cost_msg.header.frame_id = params_.fixed_frame_id;
    //     cost_msg.header.stamp = stamp_;
    //     cost_msg.type = cost_msg.POINTS;
    //     cost_msg.action = cost_msg.ADD;
    //     cost_msg.scale.x = scale;
    //     cost_msg.scale.y = scale;
    //     cost_msg.scale.z = scale;
    //     cost_msg.lifetime = ros::Duration(0.0);
    //     for(size_t pointIdx = 0; pointIdx < mainCloud->size(); pointIdx++)
    //     {
    //         for (uint d1=0; d1<map_size_y; d1++)
    //         {
    //             for (uint d0=0; d0<map_size_x; d0++)
    //             {
    //                 PointT thisPt = mainCloud->at(pointIdx);
    //                 if (thisPt.x > d0*resolution+map_min_x && thisPt.x < (d0+1)*resolution+map_min_x)
    //                 {
    //                     if (thisPt.y > d1*resolution+map_min_y && thisPt.y < (d1+1)*resolution+map_min_y)
    //                     {
    //                         geometry_msgs::Point pt_msg;
    //                         pt_msg.x = thisPt.x;
    //                         pt_msg.y = thisPt.y;
    //                         pt_msg.z = thisPt.z;
    //                         cost_msg.points.push_back(pt_msg);

    //                         double cost = costmap_msg.data[d0+d1*map_size_x];
    //                         // ROS_INFO("Cost %f x %f y %f", costmap_msg.data[d0+d1*map_size_x], d0*resolution+map_min_x, d1*resolution+map_min_y);
    //                         max_cost  = 10.0;
    //                         cost = std::min(max_cost, std::max(min_cost, cost));
    //                         double min_color_val = 0.f;
    //                         double max_color_val = 1.f;
    //                         auto a=min_cost, b=max_cost, c=min_color_val, d=max_color_val;
    //                         double color_ratio = (d-c)/(b-a)*cost + (c*b-d*a)/(b-a);
    //                         std_msgs::ColorRGBA rgb_msg;
    //                         RGBColor color = getRGBColor(color_ratio); 
    //                         rgb_msg.r = color.r;
    //                         rgb_msg.g = color.g;
    //                         rgb_msg.b = color.b;
    //                         rgb_msg.a = 1.;
    //                         cost_msg.colors.push_back(rgb_msg);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     costmap3d_pub_.publish(cost_msg);
    //     ros::spinOnce();
    // }

    // instead of binning, just calc cost of each point
    std::vector<double> costs(mainCloud->size());
    Eigen::Vector3f upVec(0,0,1);
    double min_cost = INFINITY, max_cost = -INFINITY;
    float curvature_gain = params_.costmap.curv_gain;
    float normal_gain = params_.costmap.normal_gain;
    {
        for(size_t pointIdx = 0; pointIdx < mainCloud->size(); pointIdx++)
        {
            PointT thisPt = mainCloud->at(pointIdx);
            Normal thisNm = normalCloud->at(pointIdx);
            Eigen::Vector3f nmVec(thisNm.normal_x, thisNm.normal_y, thisNm.normal_z);

            // calc cost
            double cost = 0;
            cost += normal_gain*fabs((1/upVec.dot(nmVec) - 1));
            cost += curvature_gain*fabs(thisNm.curvature);
            // cost += fabs(curvature_gain*(cbrt(thisNm.curvature)));
            if (cost > max_cost) { max_cost = cost; }
            if (cost < min_cost) { min_cost = cost; }
            costs[pointIdx] = cost;
        }
    }
    {
        float scale = 0.01;
        visualization_msgs::Marker cost_msg;
        cost_msg.header.frame_id = params_.fixed_frame_id;
        cost_msg.header.stamp = stamp_;
        cost_msg.type = cost_msg.POINTS;
        cost_msg.action = cost_msg.ADD;
        cost_msg.scale.x = scale;
        cost_msg.scale.y = scale;
        cost_msg.scale.z = scale;
        cost_msg.lifetime = ros::Duration(0.0);

        max_cost  = 10.0;
        double min_color_val = 0.f;
        double max_color_val = 1.f;
        auto a=min_cost, b=max_cost, c=min_color_val, d=max_color_val;

        for(size_t pointIdx = 0; pointIdx < mainCloud->size(); pointIdx++)
        {
            PointT thisPt = mainCloud->at(pointIdx);

            // calc color for viz
            double cost = std::min(max_cost, std::max(min_cost, costs[pointIdx]));
            double color_ratio = (d-c)/(b-a)*cost + (c*b-d*a)/(b-a);
            std_msgs::ColorRGBA rgb_msg;
            RGBColor color = getRGBColor(color_ratio);

            // construct marker msg
            geometry_msgs::Point pt_msg;
            pt_msg.x = thisPt.x;
            pt_msg.y = thisPt.y;
            pt_msg.z = thisPt.z;
            cost_msg.points.push_back(pt_msg);
            rgb_msg.r = color.r;
            rgb_msg.g = color.g;
            rgb_msg.b = color.b;
            rgb_msg.a = 1.;
            cost_msg.colors.push_back(rgb_msg);

            // ROS_INFO("point %d x %f y %f z %f cost %f colrat %f", pointIdx, thisPt.x, thisPt.y, thisPt.z, cost, color_ratio);
        }
        costmap3d_pub_.publish(cost_msg);
        ros::spinOnce();
    }
    {
        float scale = 0.01;
        visualization_msgs::Marker cost_msg;
        cost_msg.header.frame_id = params_.fixed_frame_id;
        cost_msg.header.stamp = stamp_;
        cost_msg.type = cost_msg.POINTS;
        cost_msg.action = cost_msg.ADD;
        cost_msg.scale.x = scale;
        cost_msg.scale.y = scale;
        cost_msg.scale.z = scale;
        cost_msg.lifetime = ros::Duration(0.0);

        max_cost  = 10.0;
        double min_color_val = 0.f;
        double max_color_val = 1.f;
        auto a=min_cost, b=max_cost, c=min_color_val, d=max_color_val;

        for(size_t pointIdx = 0; pointIdx < mainCloud->size(); pointIdx++)
        {
            PointT thisPt = mainCloud->at(pointIdx);

            // calc color for viz
            double cost = std::min(max_cost, std::max(min_cost, costs[pointIdx]));
            double color_ratio = (d-c)/(b-a)*cost + (c*b-d*a)/(b-a);
            std_msgs::ColorRGBA rgb_msg;
            RGBColor color = getRGBColor(color_ratio);

            // construct marker msg
            geometry_msgs::Point pt_msg;
            pt_msg.x = thisPt.x;
            pt_msg.y = thisPt.y;
            pt_msg.z = 0;
            cost_msg.points.push_back(pt_msg);
            rgb_msg.r = color.r;
            rgb_msg.g = color.g;
            rgb_msg.b = color.b;
            rgb_msg.a = 1.;
            cost_msg.colors.push_back(rgb_msg);
        }
        costmap_pub_.publish(cost_msg);
        ros::spinOnce();
    }

    busy = false;
}


void cfgCb(curvy_terrain_mapper::CurvyTerrainMapperConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure requested.");

    // n.param("pub_viz", params_.pub_viz_, true);
    params_.preanalysis.dsFlag = config.groups.preanalysis.dsFlag;
    params_.preanalysis.dsResolution = config.groups.preanalysis.dsResolution;
    params_.preanalysis.neNeighMethod = config.groups.preanalysis.neNeighMethod;
    params_.preanalysis.neSearchNeighbours = config.groups.preanalysis.neSearchNeighbours;
    params_.preanalysis.neSearchRadius = config.groups.preanalysis.neSearchRadius;
    params_.preanalysis.gpFlag = config.groups.preanalysis.gpFlag;
    params_.preanalysis.gpAngle = config.groups.preanalysis.gpAngle;
    params_.preanalysis.pfActive = config.groups.preanalysis.pfActive;
    params_.preanalysis.pfAngle = config.groups.preanalysis.pfAngle;
    params_.preanalysis.fsActive = config.groups.preanalysis.fsActive;
    params_.preanalysis.fsAngle = config.groups.preanalysis.fsAngle;
    params_.preanalysis.fsRange = config.groups.preanalysis.fsRange;
    params_.preanalysis.rob_x = config.groups.preanalysis.rob_x;
    params_.preanalysis.rob_y = config.groups.preanalysis.rob_y;
    params_.preanalysis.rob_z = config.groups.preanalysis.rob_z;
    params_.preanalysis.robAngle = config.groups.preanalysis.robAngle;
    params_.preanalysis.dsMethod = config.groups.preanalysis.dsMethod;
    params_.preanalysis.neMethod = config.groups.preanalysis.neMethod;
    params_.segmentationmode = config.segmentationmode;
    params_.regiongrowing.minClustSize = config.groups.region_growing.minClustSize;
    params_.regiongrowing.noNeigh = config.groups.region_growing.noNeigh;
    params_.regiongrowing.smoothFlag = config.groups.region_growing.smoothFlag;
    params_.regiongrowing.smoothThresh = config.groups.region_growing.smoothThresh;
    params_.regiongrowing.resFlag = config.groups.region_growing.resFlag;
    params_.regiongrowing.resThresh = config.groups.region_growing.resThresh;
    params_.regiongrowing.curvFlag = config.groups.region_growing.curvFlag;
    params_.regiongrowing.curvThresh = config.groups.region_growing.curvThresh;
    params_.regiongrowing.updateFlag = config.groups.region_growing.updateFlag;
    params_.regiongrowing.pointUpdateFlag = config.groups.region_growing.pointUpdateFlag;
    params_.regiongrowing.updateInterval = config.groups.region_growing.updateInterval;
    params_.costmap.normal_gain = config.groups.costmap.normal_gain;
    params_.costmap.curv_gain = config.groups.costmap.curv_gain;
}

int main (int argc, char *argv[])
{  
    ros::init(argc, argv, "curvy_terrain_mapper");
    ros::NodeHandle n("~");
    ros::Subscriber input_sub = n.subscribe("input_cloud",1,&inputCB);
    dynamic_reconfigure::Server<curvy_terrain_mapper::CurvyTerrainMapperConfig> cfg_server_;
    cfg_server_.setCallback(boost::bind(&cfgCb, _1, _2));

    n.param("fixed_frame_id", params_.fixed_frame_id, std::string("map"));
    // n.param("pub_viz", params_.pub_viz_, true);
    n.param("preanalysis/dsFlag", params_.preanalysis.dsFlag, true);
    n.param("preanalysis/dsResolution", params_.preanalysis.dsResolution, 0.01);
    n.param("preanalysis/neNeighMethod", params_.preanalysis.neNeighMethod, 0);
    n.param("preanalysis/neSearchNeighbours", params_.preanalysis.neSearchNeighbours, 24);
    n.param("preanalysis/neSearchRadius", params_.preanalysis.neSearchRadius, 0.2);
    n.param("preanalysis/gpFlag", params_.preanalysis.gpFlag, false);
    n.param("preanalysis/gpAngle", params_.preanalysis.gpAngle, 25.0);
    n.param("preanalysis/pfActive", params_.preanalysis.pfActive, false);
    n.param("preanalysis/pfAngle", params_.preanalysis.pfAngle, 20.0);
    n.param("preanalysis/fsActive", params_.preanalysis.fsActive, false);
    n.param("preanalysis/fsAngle", params_.preanalysis.fsAngle, 30.0);
    n.param("preanalysis/fsRange", params_.preanalysis.fsRange, 0.05);
    n.param("preanalysis/rob_x", params_.preanalysis.rob_x, 0.0);
    n.param("preanalysis/rob_y", params_.preanalysis.rob_y, 0.0);
    n.param("preanalysis/rob_z", params_.preanalysis.rob_z, 0.0);
    n.param("preanalysis/robAngle", params_.preanalysis.robAngle, 0.0);
    n.param("preanalysis/dsMethod", params_.preanalysis.dsMethod, false);
    n.param("preanalysis/neMethod", params_.preanalysis.neMethod, 0);
    n.param("segmentationmode", params_.segmentationmode, 0);
    n.param("regiongrowing/minClustSize", params_.regiongrowing.minClustSize, 30);
    n.param("regiongrowing/noNeigh", params_.regiongrowing.noNeigh, 24);
    n.param("regiongrowing/smoothFlag", params_.regiongrowing.smoothFlag, false);
    n.param("regiongrowing/smoothThresh", params_.regiongrowing.smoothThresh, 50.0);
    n.param("regiongrowing/resFlag", params_.regiongrowing.resFlag, true);
    n.param("regiongrowing/resThresh", params_.regiongrowing.resThresh, 0.08);
    n.param("regiongrowing/curvFlag", params_.regiongrowing.curvFlag, false);
    n.param("regiongrowing/curvThresh", params_.regiongrowing.curvThresh, 0.1);
    n.param("regiongrowing/updateFlag", params_.regiongrowing.updateFlag, true);
    n.param("regiongrowing/pointUpdateFlag", params_.regiongrowing.pointUpdateFlag, true);
    n.param("regiongrowing/updateInterval", params_.regiongrowing.updateInterval, 100);
    n.param("costmap/normal_gain", params_.costmap.normal_gain, 4.0);
    n.param("costmap/curv_gain", params_.costmap.curv_gain, 30.0);

    // config_ = YAML::LoadFile(config_filepath_);
    // params_.fixed_frame_id = config_["ros"]["fixed_frame_id"].as<std::string>();

    main_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("main_cloud",1);
    normal_cloud_pub_ = n.advertise<visualization_msgs::Marker>("normal_cloud",1);
    curvature_pub_ = n.advertise<visualization_msgs::Marker>("curvature",1);
    costmap_pub_ = n.advertise<visualization_msgs::Marker>("costmap",1);
    costmap3d_pub_ = n.advertise<visualization_msgs::Marker>("costmap3d",1);
    seg_regions_pub_ = n.advertise<visualization_msgs::Marker>("segmented_regions", 1);

    ros::spin();
}

