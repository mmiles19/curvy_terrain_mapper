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
typedef pcl::PointXYZI PointTI;
typedef pcl::PointXYZINormal PointTIN;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;
typedef pcl::PointCloud<PointTI> PointCloudI;
typedef pcl::PointCloud<PointTIN> PointCloudIN;


class CurvyTerrainMapper
{
private:

    CurvyTerrainMapperParams params_;

    // bool busy;
    boost::mutex mutex_;

    // std::string config_filepath_;
    // YAML::Node config_;
    // std::string fixed_frame_id_;
    ros::Time stamp_;
    std::string frame_;
    ros::Publisher main_cloud_pub_;
    ros::Publisher normal_cloud_pub_;
    ros::Publisher curvature_pub_;
    // ros::Publisher costmap_pub_;
    // ros::Publisher costmap3d_pub_;
    ros::Publisher seg_regions_pub_;
    ros::Publisher costcloud_pub_;
    ros::Publisher costnormcloud_pub_;
    tf::TransformListener listener;

    ros::NodeHandle m_nh, m_private_nh;
    ros::Subscriber input_sub;
    dynamic_reconfigure::Server<curvy_terrain_mapper::CurvyTerrainMapperConfig> cfg_server_;

public:

    CurvyTerrainMapper(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_nh(nh),
        m_private_nh(pnh)//,
        // busy(false)
    {

        input_sub = m_private_nh.subscribe("input_cloud", 1 ,&CurvyTerrainMapper::inputCB, this);

        cfg_server_.setCallback(boost::bind(&CurvyTerrainMapper::cfgCb, this, _1, _2));

        m_private_nh.param("fixed_frame_id", params_.fixed_frame_id, std::string("map"));
        // m_private_nh.param("pub_viz", params_.pub_viz_, true);
        m_private_nh.param("preanalysis/dsFlag", params_.preanalysis.dsFlag, true);
        m_private_nh.param("preanalysis/dsResolution", params_.preanalysis.dsResolution, 0.01);
        m_private_nh.param("preanalysis/neNeighMethod", params_.preanalysis.neNeighMethod, 0);
        m_private_nh.param("preanalysis/neSearchNeighbours", params_.preanalysis.neSearchNeighbours, 24);
        m_private_nh.param("preanalysis/neSearchRadius", params_.preanalysis.neSearchRadius, 0.2);
        m_private_nh.param("preanalysis/gpFlag", params_.preanalysis.gpFlag, false);
        m_private_nh.param("preanalysis/gpAngle", params_.preanalysis.gpAngle, 25.0);
        m_private_nh.param("preanalysis/pfActive", params_.preanalysis.pfActive, false);
        m_private_nh.param("preanalysis/pfAngle", params_.preanalysis.pfAngle, 20.0);
        m_private_nh.param("preanalysis/fsActive", params_.preanalysis.fsActive, false);
        m_private_nh.param("preanalysis/fsAngle", params_.preanalysis.fsAngle, 30.0);
        m_private_nh.param("preanalysis/fsRange", params_.preanalysis.fsRange, 0.05);
        m_private_nh.param("preanalysis/rob_x", params_.preanalysis.rob_x, 0.0);
        m_private_nh.param("preanalysis/rob_y", params_.preanalysis.rob_y, 0.0);
        m_private_nh.param("preanalysis/rob_z", params_.preanalysis.rob_z, 0.0);
        m_private_nh.param("preanalysis/robAngle", params_.preanalysis.robAngle, 0.0);
        m_private_nh.param("preanalysis/dsMethod", params_.preanalysis.dsMethod, false);
        m_private_nh.param("preanalysis/neMethod", params_.preanalysis.neMethod, 0);
        m_private_nh.param("segmentationmode", params_.segmentationmode, 0);
        m_private_nh.param("regiongrowing/enable", params_.regiongrowing.enable, false);
        m_private_nh.param("regiongrowing/minClustSize", params_.regiongrowing.minClustSize, 30);
        m_private_nh.param("regiongrowing/noNeigh", params_.regiongrowing.noNeigh, 24);
        m_private_nh.param("regiongrowing/smoothFlag", params_.regiongrowing.smoothFlag, false);
        m_private_nh.param("regiongrowing/smoothThresh", params_.regiongrowing.smoothThresh, 50.0);
        m_private_nh.param("regiongrowing/resFlag", params_.regiongrowing.resFlag, true);
        m_private_nh.param("regiongrowing/resThresh", params_.regiongrowing.resThresh, 0.08);
        m_private_nh.param("regiongrowing/curvFlag", params_.regiongrowing.curvFlag, false);
        m_private_nh.param("regiongrowing/curvThresh", params_.regiongrowing.curvThresh, 0.1);
        m_private_nh.param("regiongrowing/updateFlag", params_.regiongrowing.updateFlag, true);
        m_private_nh.param("regiongrowing/pointUpdateFlag", params_.regiongrowing.pointUpdateFlag, true);
        m_private_nh.param("regiongrowing/updateInterval", params_.regiongrowing.updateInterval, 100);
        m_private_nh.param("costmap/normal_gain", params_.costmap.normal_gain, 4.0);
        m_private_nh.param("costmap/curv_gain", params_.costmap.curv_gain, 30.0);
        m_private_nh.param("costmap/max_saturation_cost", params_.costmap.max_saturation_cost, 1.0);
        m_private_nh.param("costmap/min_saturation_cost", params_.costmap.min_saturation_cost, 0.0);
        m_private_nh.param("costmap/set_max_saturation_cost_to_max_cost", params_.costmap.set_max_saturation_cost_to_max_cost, true);
        m_private_nh.param("costmap/set_min_saturation_cost_to_min_cost", params_.costmap.set_min_saturation_cost_to_min_cost, true);

        // config_ = YAML::LoadFile(config_filepath_);
        // params_.fixed_frame_id = config_["ros"]["fixed_frame_id"].as<std::string>();

        main_cloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("main_cloud",1);
        normal_cloud_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("normal_cloud",1);
        curvature_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("curvature",1);
        // costmap_pub_ = n.advertise<visualization_msgs::Marker>("costmap",1);
        // costmap3d_pub_ = n.advertise<visualization_msgs::Marker>("costmap3d",1);
        seg_regions_pub_ = m_private_nh.advertise<visualization_msgs::Marker>("segmented_regions", 1);
        costcloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("cost_cloud",1);
        costnormcloud_pub_ = m_private_nh.advertise<sensor_msgs::PointCloud2>("cost_norm_cloud",1);
    }

    inline float rescale(float initial, float min_initial, float max_initial, float min_final=0.0f, float max_final=1.0f)
    {
        float ratio = (max_final-min_final)/(max_initial-min_initial)*initial + (min_final*max_initial-max_final*min_initial)/(max_initial-min_initial);
        return ratio;
    }

    inline void cfgCb(curvy_terrain_mapper::CurvyTerrainMapperConfig &config, uint32_t level)
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
        params_.regiongrowing.enable = config.groups.region_growing.enable;
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
        params_.costmap.max_saturation_cost = config.groups.costmap.max_saturation_cost;
        params_.costmap.min_saturation_cost = config.groups.costmap.min_saturation_cost;
        params_.costmap.set_max_saturation_cost_to_max_cost = config.groups.costmap.set_max_saturation_cost_to_max_cost;
        params_.costmap.set_min_saturation_cost_to_min_cost = config.groups.costmap.set_min_saturation_cost_to_min_cost;
    }

    inline void inputCB(const sensor_msgs::PointCloud2& input_msg)
    {
        // ROS_INFO("Got input cloud.");

        if(!ros::ok()){ return; }
        // if(busy){ return; }
        // uint count;

        // busy = true;

        boost::mutex::scoped_lock lock(mutex_);

        stamp_ = input_msg.header.stamp;
        frame_ = input_msg.header.frame_id;

    // Loading input point cloud //

        double loadS = pcl::getTime();

        PointCloudT::Ptr mainCloud;
        mainCloud.reset (new PointCloudT);

        pcl::fromROSMsg(input_msg, *mainCloud);

    // Starting preanalysis //

        ROS_INFO("---");
        ROS_INFO("Received point cloud with %d points",mainCloud->width*mainCloud->height);

        if (mainCloud->width*mainCloud->height==0)
        {
            ROS_WARN("Received empty cloud. Skipping.");
            // busy = false;
            return;
        }

        ROS_INFO("Starting preanalysis");
        double preAS = pcl::getTime();

        Preanalysis pre;
        pre.loadConfig(params_.preanalysis);
        NormalCloud::Ptr mainNormals;
        mainNormals.reset(new NormalCloud);
        PointCloudT::Ptr floorCloud;
        floorCloud.reset (new PointCloudT);
        NormalCloud::Ptr floorNormals;
        floorNormals.reset(new NormalCloud);
        PointCloudC coloredNormalCloud;

        Eigen::Matrix4d T_fixed_input_mat = Eigen::Matrix4d::Identity();
        if (input_msg.header.frame_id != params_.fixed_frame_id)
        {
            if (!getTransformFromTree(listener, params_.fixed_frame_id, input_msg.header.frame_id, &T_fixed_input_mat, stamp_))
            {
                // busy = false;
                return;
            }
        }
        // Eigen::Matrix4d invTransformCloud = transformCloud.inverse();

        // transform mainCloud (default identity),
        // downsample (default true, 0.01m res),
        // normal estimation and init mainNormals with output,
        // filter ghost/shadow points (nonexistent points at discontinuities due to sensor noise),
        // extract floor and init floorCloud with output,
        // init prepNorMap with xyz's of mainCloud points and rgb's of mainCloud normals
        pre.run(mainCloud, mainNormals, coloredNormalCloud, floorCloud, floorNormals, T_fixed_input_mat);
        double preAE = pcl::getTime();
        ROS_INFO("Preanalysis took: %f",preAE-preAS);

        // pubMainCloud(&main_cloud_pub_, *mainCloud, params_.fixed_frame_id, stamp_);
        pubNormalCloud(&normal_cloud_pub_, *mainCloud, *mainNormals, params_.fixed_frame_id, stamp_);
        pubCurvature(&curvature_pub_, *mainCloud, *mainNormals, params_.fixed_frame_id, stamp_);

        PointCloudT::Ptr currentCloud = mainCloud;
        NormalCloud::Ptr currentNormals = mainNormals;

    // Starting segmentation //

        if (params_.regiongrowing.enable)
        {
          ROS_INFO("Starting segmentation");
          double segS = pcl::getTime();
          regions segRegions;
          int segMode = params_.segmentationmode;
          PointCloudT::Ptr segCloud = mainCloud;
          NormalCloud::Ptr segNormals = mainNormals;
          // PointCloudT::Ptr segCloud = floorCloud;
          // NormalCloud::Ptr segNormals = floorNormals;
          switch (segMode)
          {
              case 0:
              {
                  ROS_INFO("Using Region Growing algorithm");
                  RegionGrowing reGrow;
                  reGrow.loadConfig(params_.regiongrowing);
                  reGrow.setInputCloud(segCloud);
                  reGrow.setNormalCloud(segNormals);
                  // extract and init segRegions with smooth regions
                  reGrow.run(segRegions);
                  break;
              }
              // case 1:
              // {
              //     ROS_INFO("Using Voxel SAC algorithm");
              //     voxSAC voxelSAC;
              //     voxelSAC.setInputCloud(segCloud);
              //     voxelSAC.setNormalCloud(segNormals);
              //     voxelSAC.run(segRegions);
              //     break;
              // }
              // case 2:
              // {
              //     ROS_INFO("Using Split & Merge algorithm");
              //     splitMerge sam;
              //     sam.setInputCloud(segCloud);
              //     sam.setNormalCloud(segNormals);
              //     sam.splitProcess();
              //     sam.mergeProcess(segRegions);
              //     break;
              // }
              default:
              {
                ROS_WARN("Unsupported segmentation mode.");
              }
          }
          double segE = pcl::getTime();
          ROS_INFO("Segmentation found %d regions",segRegions.size());
          ROS_INFO("Segmentation took: %f",segE-segS);

          // pubRegions(&seg_regions_pub_,segRegions, params_.fixed_frame_id, stamp_);

          if (segRegions.size()==0)
          {
            //   busy = false;
              return;
          }

          segmentPatch largestPatch;
          for (uint i=0; i<segRegions.size(); i++)
          {
              if (segRegions.at(i).segmentCloud.size() > largestPatch.segmentCloud.size())
              {
                  largestPatch = segRegions.at(i);
              }
          }
          ROS_INFO("Largest patch has %d points",largestPatch.segmentCloud.size());

          currentCloud.reset(new PointCloudT(largestPatch.segmentCloud));
          currentNormals.reset(new NormalCloud(largestPatch.normalCloud));
        }

        // init costmap
        ROS_INFO("Starting costmap calculation");
        double cmS = pcl::getTime();

        // float map_min_x = 0.f, map_max_x = 4.f, map_min_y = -1.5f, map_max_y = 1.5f, resolution = 0.05;
        // uint map_size_x = (map_max_x-map_min_x)/resolution, map_size_y = (map_max_y-map_min_y)/resolution;

        // calc cost of each point
        Eigen::Vector3f upVec(0,0,1);
        float min_cost = INFINITY, max_cost = -INFINITY;
        // double min_cost = 0.0, max_cost = 1.0;
        float curvature_gain = params_.costmap.curv_gain;
        float normal_gain = params_.costmap.normal_gain;

        // PointCloudT::Ptr costCloud = mainCloud;
        // NormalCloud::Ptr costNormals = mainNormals;
        // // PointCloudT::Ptr costCloud = floorCloud;
        // // NormalCloud::Ptr costNormals = floorNormals;
        // // PointCloudT::Ptr costCloud; costCloud.reset(new PointCloudT(largestPatch.segmentCloud));
        // // NormalCloud::Ptr costNormals; costNormals.reset(new NormalCloud(largestPatch.normalCloud));
        // ROS_INFO("costCloud has %d points",costNormals->size());
        // // assert(costCloud->size()==costNormals->size());
        // std::vector<double> costs(costCloud->size());
        // {
        //     for(size_t pointIdx = 0; pointIdx < costCloud->size(); pointIdx++)
        //     {
        //         PointT thisPt = costCloud->at(pointIdx);
        //         Normal thisNm = costNormals->at(pointIdx);
        //         Eigen::Vector3f nmVec(thisNm.normal_x, thisNm.normal_y, thisNm.normal_z);

        //         // calc cost
        //         double cost = 0;
        //         cost += normal_gain*pow(fabs((1/upVec.dot(nmVec) - 1)),2);
        //         cost += curvature_gain*fabs(thisNm.curvature);
        //         // cost += fabs(curvature_gain*(cbrt(thisNm.curvature)));
        //         if (cost > max_cost) { max_cost = cost; }
        //         if (cost < min_cost) { min_cost = cost; }
        //         costs[pointIdx] = cost;
        //     }
        // }

        PointCloudIN costCloud;
        {
            for(uint i=0; i<currentCloud->size(); i++)
            {
                PointT pt = currentCloud->points[i];
                Normal nm = currentNormals->points[i];

                Eigen::Vector3f nmVec(nm.normal_x, nm.normal_y, nm.normal_z);

                double cost = 0;
                cost += normal_gain*pow(1-fabs(upVec.dot(nmVec)),3);
                cost += curvature_gain*fabs(nm.curvature);
                // cost += fabs(curvature_gain*(cbrt(it->curvature)));

                if (cost > max_cost) { max_cost = cost; }
                if (cost < min_cost) { min_cost = cost; }

                // if (cost > max_cost) { cost = max_cost; }
                // if (cost < min_cost) { cost = min_cost; }

                PointTIN newpt;
                newpt.x = pt.x;
                newpt.y = pt.y;
                newpt.z = pt.z;
                newpt.intensity = cost;
                newpt.normal_x = nm.normal_x;
                newpt.normal_y = nm.normal_y;
                newpt.normal_z = nm.normal_z;
                newpt.curvature = nm.curvature;
                costCloud.push_back(newpt);
            }
        }

        PointCloudIN costNormCloud;
        {
            for(uint i=0; i<costCloud.size(); i++)
            {
                PointTIN pt = costCloud.points[i];

                PointTIN newpt;
                newpt.x = pt.x;
                newpt.y = pt.y;
                newpt.z = pt.z;

                if (params_.costmap.set_max_saturation_cost_to_max_cost) params_.costmap.max_saturation_cost = max_cost;
                if (params_.costmap.set_min_saturation_cost_to_min_cost) params_.costmap.min_saturation_cost = min_cost;
                newpt.intensity = std::min(std::max(rescale(pt.intensity, params_.costmap.min_saturation_cost, params_.costmap.max_saturation_cost,0.0f,1.0f),0.0f),1.0f);
                if (!std::isfinite(newpt.intensity) || newpt.intensity<0.0 || newpt.intensity>1.0)
                    continue;

                newpt.normal_x = pt.normal_x;
                newpt.normal_y = pt.normal_y;
                newpt.normal_z = pt.normal_z;
                newpt.curvature = pt.curvature;
                costNormCloud.push_back(newpt);
            }
        }

        double cmE = pcl::getTime();
        ROS_INFO("Costmap gen took: %f s. Publishing costmap(s) of size %d...",cmE-cmS,costCloud.width*costCloud.height);
        // pub costmap pc2
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cost_cloud;
        // {
        //     for(size_t pointIdx = 0; pointIdx < costCloud->size(); pointIdx++)
        //     {
        //         PointT thisPt = costCloud->at(pointIdx);

        //         pcl::PointXYZI cloud_pt;
        //         cloud_pt.x = thisPt.x;
        //         cloud_pt.y = thisPt.y;
        //         cloud_pt.z = thisPt.z;
        //         cloud_pt.intensity = cost[pointIdx];
        //         cost_cloud->push_back(cloud_pt);
        //     }
        //     sensor_msgs::PointCloud2 msg;
        //     pcl::toROSMsg(cost_cloud, msg);
        //     costcloud_pub_.publish(msg);
        //     ros::spinOnce();
        // }

        // pub cost cloud
        // {
        //     sensor_msgs::PointCloud2 costcloud_msg;
        //     pcl::toROSMsg (costCloud, costcloud_msg);
        //     costcloud_msg.header.frame_id = params_.fixed_frame_id;
        //     costcloud_msg.header.stamp = stamp_;
        //     costcloud_pub_.publish(costcloud_msg);
        //     // ros::spinOnce();
        // }
        // pub cost norm cloud
        {
            sensor_msgs::PointCloud2 costcloud_msg;
            pcl::toROSMsg (costNormCloud, costcloud_msg);
            costcloud_msg.header.frame_id = params_.fixed_frame_id;
            costcloud_msg.header.stamp = stamp_;
            costnormcloud_pub_.publish(costcloud_msg);
            // ros::spinOnce();
        }
        // // pub 3d costmap viz
        // {
        //     float scale = 0.01;
        //     visualization_msgs::Marker cost_msg;
        //     cost_msg.header.frame_id = params_.fixed_frame_id;
        //     cost_msg.header.stamp = stamp_;
        //     cost_msg.type = cost_msg.POINTS;
        //     cost_msg.action = cost_msg.ADD;
        //     cost_msg.scale.x = scale;
        //     cost_msg.scale.y = scale;
        //     cost_msg.scale.z = scale;
        //     cost_msg.lifetime = ros::Duration(0.0);

        //     // max_cost  = 10.0;
        //     double min_color_val = 0.f;
        //     double max_color_val = 1.f;
        //     auto a=min_initial, b=max_initial, c=min_final, d=max_final;

        //     for(size_t pointIdx = 0; pointIdx < costCloud->size(); pointIdx++)
        //     {
        //         PointT thisPt = costCloud->at(pointIdx);

        //         // calc color for viz
        //         double cost = std::min(max_cost, std::max(min_cost, costs[pointIdx]));
        //         double color_ratio = (max_final-min_final)/(max_initial-min_initial)*initial + (min_final*max_initial-max_final*min_initial)/(max_initial-min_initial);
        //         std_msgs::ColorRGBA rgb_msg;
        //         RGBColor color = getRGBColor(color_ratio);

        //         // construct marker msg
        //         geometry_msgs::Point pt_msg;
        //         pt_msg.x = thisPt.x;
        //         pt_msg.y = thisPt.y;
        //         pt_msg.z = thisPt.z;
        //         cost_msg.points.push_back(pt_msg);
        //         rgb_msg.r = color.r;
        //         rgb_msg.g = color.g;
        //         rgb_msg.b = color.b;
        //         rgb_msg.a = 1.;
        //         cost_msg.colors.push_back(rgb_msg);

        //         // ROS_INFO("point %d x %f y %f z %f cost %f colrat %f", pointIdx, thisPt.x, thisPt.y, thisPt.z, cost, color_ratio);
        //     }
        //     costmap3d_pub_.publish(cost_msg);
        //     ros::spinOnce();
        // }
        // // pub 2d costmap
        // {
        //     float scale = 0.01;
        //     visualization_msgs::Marker cost_msg;
        //     cost_msg.header.frame_id = params_.fixed_frame_id;
        //     cost_msg.header.stamp = stamp_;
        //     cost_msg.type = cost_msg.POINTS;
        //     cost_msg.action = cost_msg.ADD;
        //     cost_msg.scale.x = scale;
        //     cost_msg.scale.y = scale;
        //     cost_msg.scale.z = scale;
        //     cost_msg.lifetime = ros::Duration(0.0);

        //     max_cost  = 10.0;
        //     double min_color_val = 0.f;
        //     double max_color_val = 1.f;
        //     auto a=min_cost, b=max_cost, c=min_color_val, d=max_color_val;

        //     for(size_t pointIdx = 0; pointIdx < costCloud->size(); pointIdx++)
        //     {
        //         PointT thisPt = costCloud->at(pointIdx);

        //         // calc color for viz
        //         double cost = std::min(max_cost, std::max(min_cost, costs[pointIdx]));
        //         double color_ratio = (d-c)/(b-a)*cost + (c*b-d*a)/(b-a);
        //         std_msgs::ColorRGBA rgb_msg;
        //         RGBColor color = getRGBColor(color_ratio);

        //         // construct marker msg
        //         geometry_msgs::Point pt_msg;
        //         pt_msg.x = thisPt.x;
        //         pt_msg.y = thisPt.y;
        //         pt_msg.z = 0;
        //         cost_msg.points.push_back(pt_msg);
        //         rgb_msg.r = color.r;
        //         rgb_msg.g = color.g;
        //         rgb_msg.b = color.b;
        //         rgb_msg.a = 1.;
        //         cost_msg.colors.push_back(rgb_msg);
        //     }
        //     costmap_pub_.publish(cost_msg);
        //     ros::spinOnce();
        // }

        ROS_INFO("Done publishing clouds");

        // busy = false;
    }
};

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "curvy_terrain_mapper");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    CurvyTerrainMapper ctm(n,np);

    ros::spin();
}
