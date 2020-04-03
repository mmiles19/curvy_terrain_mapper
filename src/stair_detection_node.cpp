// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <curvy_terrain_mapper/preanalysis.h>
#include <curvy_terrain_mapper/regions.h>
#include <curvy_terrain_mapper/regiongrowing.h>
#include <curvy_terrain_mapper/voxSAC.h>
#include <curvy_terrain_mapper/splitmerge.h>
#include <curvy_terrain_mapper/planeshape.h>
#include <curvy_terrain_mapper/recognition.h>
#include <curvy_terrain_mapper/StairVector.h>
#include <curvy_terrain_mapper/prediction.h>

#include <ros/ros.h>
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

bool busy;

std::string config_filepath_;
YAML::Node config_;
std::string fixed_frame_id_;
ros::Publisher main_cloud_pub_;
ros::Publisher normal_cloud_pub_;
ros::Publisher seg_regions_pub_;
ros::Publisher rise_regions_pub_;
ros::Publisher tread_regions_pub_;
ros::Publisher tread_region_nums_pub_;
ros::Publisher rise_cloud_pub_;
ros::Publisher tread_cloud_pub_;
ros::Publisher rail_cloud_pub_;
ros::Publisher seg_stairs_pub_;
ros::Publisher stair_parts_pub_;
ros::Publisher nearest_step_pose_pub_;

struct RGBColor{ uint r, g, b; };
inline RGBColor getRGBColor(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 6);

    //find the distance to the start of the closest region
    int x = normalized % 256;

    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
    case 0: red = 255;      grn = x;        blu = 0;       break;//red
    case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
    case 2: red = 0;        grn = 255;      blu = x;       break;//green
    case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
    case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
    }

    RGBColor out;
    out.r = red;
    out.g = grn;
    out.b = blu;
    return out;
}

inline void pubPose(ros::Publisher* pub, geometry_msgs::PoseStamped step_pose)
{
    if (sqrt(pow(step_pose.pose.position.x,2)+pow(step_pose.pose.position.y,2)+pow(step_pose.pose.position.z,2)) < .05)
        return;
        
    pub->publish(step_pose);
    ros::spinOnce();
}

inline void pubTCloud(ros::Publisher* pub, PointCloudT cloud)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    pub->publish(cloud_msg);
    ros::spinOnce();
}

inline void pubCCloud(ros::Publisher* pub, PointCloudC cloud)
{
    visualization_msgs::Marker cloud_msg;
    cloud_msg.header.frame_id = fixed_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.type = cloud_msg.POINTS;
    cloud_msg.action = cloud_msg.ADD;
    cloud_msg.scale.x = 0.005;
    cloud_msg.scale.y = 0.005;
    cloud_msg.scale.z = 0.005;
    cloud_msg.lifetime = ros::Duration(0.0);
    for(uint i=0; i<cloud.size(); i++)
    {
        PointTC pt = cloud.points[i];

        geometry_msgs::Point pt_msg;
        pt_msg.x = pt.x;
        pt_msg.y = pt.y;
        pt_msg.z = pt.z;
        cloud_msg.points.push_back(pt_msg);

        std_msgs::ColorRGBA rgb_msg;
        rgb_msg.r = pt.r/255.0;
        rgb_msg.g = pt.g/255.0;
        rgb_msg.b = pt.b/255.0;
        rgb_msg.a = 1.0;
        cloud_msg.colors.push_back(rgb_msg);
    }
    pub->publish(cloud_msg);
    ros::spinOnce();
}

inline void pubCCloud(ros::Publisher* pub, PointCloudT tcloud, NormalCloud ncloud)
{
    if (tcloud.size()!=ncloud.size())
    {
        ROS_ERROR("Invalid cloud inputs");
    }

    visualization_msgs::Marker cloud_msg;
    cloud_msg.header.frame_id = fixed_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.type = cloud_msg.POINTS;
    cloud_msg.action = cloud_msg.ADD;
    cloud_msg.scale.x = 0.005;
    cloud_msg.scale.y = 0.005;
    cloud_msg.scale.z = 0.005;
    cloud_msg.lifetime = ros::Duration(0.0);
    for(uint i=0; i<tcloud.size(); i++)
    {
        PointT pt = tcloud.points[i];
        Normal nm = ncloud.points[i];

        geometry_msgs::Point pt_msg;
        pt_msg.x = pt.x;
        pt_msg.y = pt.y;
        pt_msg.z = pt.z;
        cloud_msg.points.push_back(pt_msg);

        std_msgs::ColorRGBA rgb_msg;
        rgb_msg.r = nm.normal_x;
        rgb_msg.g = nm.normal_y;
        rgb_msg.b = nm.normal_z;
        rgb_msg.a = 1.0;
        cloud_msg.colors.push_back(rgb_msg);
    }
    pub->publish(cloud_msg);
    ros::spinOnce();
}

inline void pubCCloud(ros::Publisher* pub, regions segments)
{
    // auto pal = palettes.at("inferno").rescale(0.0, 1.0);
    visualization_msgs::Marker cloud_msg;
    cloud_msg.header.frame_id = fixed_frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.type = cloud_msg.POINTS;
    cloud_msg.action = cloud_msg.ADD;
    cloud_msg.scale.x = 0.005;
    cloud_msg.scale.y = 0.005;
    cloud_msg.scale.z = 0.005;
    cloud_msg.lifetime = ros::Duration(0.0);
    for(uint j=0; j<segments.size(); j++)
    {
        RGBColor color = getRGBColor((float)(j)/(float)(segments.size())); 

        PointCloudT cloud = segments.at(j).segmentCloud;
        for(uint i=0; i<cloud.size(); i++)
        {
            PointT pt = cloud.points[i];
            geometry_msgs::Point pt_msg;
            pt_msg.x = pt.x;
            pt_msg.y = pt.y;
            pt_msg.z = pt.z;
            cloud_msg.points.push_back(pt_msg);

            std_msgs::ColorRGBA rgb_msg;
            rgb_msg.r = (float)(color.r)/255.0;
            rgb_msg.g = (float)(color.g)/255.0;
            rgb_msg.b = (float)(color.b)/255.0;
            rgb_msg.a = 1.0;
            cloud_msg.colors.push_back(rgb_msg);
        }
    }
    pub->publish(cloud_msg);
    ros::spinOnce();
}

inline void pubLabels(ros::Publisher* pub, regions segments)
{
    // {
    // visualization_msgs::MarkerArray mkr_arr_msg;

    //     visualization_msgs::Marker mkr_msg;

    //     mkr_msg.action = mkr_msg.DELETEALL;
    //     mkr_msg.color.a = 1.0;
    //     mkr_msg.lifetime = ros::Duration(0.0);

    //     mkr_arr_msg.markers.push_back(mkr_msg);
        
    // pub->publish(mkr_arr_msg);
    // ros::spinOnce();
    // }

    {
    visualization_msgs::MarkerArray mkr_arr_msg;
    for(uint i=0; i<segments.size(); i++)
    {
        visualization_msgs::Marker mkr_msg;

        mkr_msg.header.frame_id = fixed_frame_id_;
        mkr_msg.header.stamp = ros::Time::now();
        mkr_msg.type = mkr_msg.TEXT_VIEW_FACING;
        mkr_msg.action = mkr_msg.ADD;
        mkr_msg.scale.z = 0.1;
        mkr_msg.lifetime = ros::Duration(0.0);
        mkr_msg.id = i;
        mkr_msg.color.r = mkr_msg.color.g = mkr_msg.color.b = mkr_msg.color.a = 1;
        mkr_msg.pose.position.x = segments.at(i).segmentCentroid[0];
        mkr_msg.pose.position.y = segments.at(i).segmentCentroid[1];
        mkr_msg.pose.position.z = segments.at(i).segmentCentroid[2];
        mkr_msg.text = std::to_string(i);

        mkr_arr_msg.markers.push_back(mkr_msg);
    }
    pub->publish(mkr_arr_msg);
    ros::spinOnce();
    }
}

// inline void getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, Eigen::Transformd* tform)
inline void getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, Eigen::Matrix4d* tform_mat)
{
    tf::TransformListener listener;
    tf::StampedTransform tform_msg;
    try
    {
        listener.waitForTransform(parent_frame_id, child_frame_id, ros::Time::now(), ros::Duration(1));
        listener.lookupTransform(parent_frame_id, child_frame_id, ros::Time(0), tform_msg);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Affine3d tform;
    tf::transformTFToEigen(tf::Transform(tform_msg),tform);
    (*tform_mat) = tform.matrix();
}

inline void inputCB(const sensor_msgs::PointCloud2& input_msg)
{
    if(!ros::ok()){ return; }
    if(busy){ return; }

    busy = true;

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
    pre.loadConfig(config_["preanalysis"]);
    NormalCloud::Ptr prepNomalCloud;
    prepNomalCloud.reset(new NormalCloud);
    PointCloudT floorPC;
    PointCloudC prepNorMap;

    Eigen::Matrix4d T_fixed_input_mat = Eigen::Matrix4d::Identity();
    if (input_msg.header.frame_id != fixed_frame_id_)
        getTransformFromTree(fixed_frame_id_, input_msg.header.frame_id, &T_fixed_input_mat);
    // Eigen::Matrix4d invTransformCloud = transformCloud.inverse();

    // transform mainCloud (default identity), 
    // downsample (default true, 0.01m res), 
    // normal estimation and init prepNomalCloud with output, 
    // filter ghost/shadow points (nonexistent points at discontinuities due to sensor noise), 
    // extract floor and init floorPC with output, 
    // init prepNorMap with xyz's of mainCloud points and rgb's of mainCloud normals
    pre.run(mainCloud, prepNomalCloud, prepNorMap, floorPC, T_fixed_input_mat);
    double preAE = pcl::getTime();
    ROS_INFO("Preanalysis took: %f",preAE-preAS);

    pubTCloud(&main_cloud_pub_, *mainCloud);
    pubCCloud(&normal_cloud_pub_, *mainCloud, *prepNomalCloud);

// Starting segmentation //

    ROS_INFO("Starting segmentation");
    int mode = 0;
    double segS = pcl::getTime();
    regions segRegions;
    if(mode == 0)
    {
        ROS_INFO("Using Region Growing algorithm");
        RegionGrowing reGrow;
        reGrow.loadConfig(config_["regiongrowing"]);
        reGrow.setInputCloud(mainCloud);
        reGrow.setNormalCloud(prepNomalCloud);
        // extract and init segRegions with smooth regions
        reGrow.run(segRegions);
    }
    if(mode == 1)
    {
        ROS_INFO("Using Voxel SAC algorithm");
        voxSAC voxelSAC;
        voxelSAC.setInputCloud(mainCloud);
        voxelSAC.setNormalCloud(prepNomalCloud);
        voxelSAC.run(segRegions);
    }
    if(mode == 2)
    {
        ROS_INFO("Using Split & Merge algorithm");
        splitMerge sam;
        sam.setInputCloud(mainCloud);
        sam.setNormalCloud(prepNomalCloud);
        sam.splitProcess();
        sam.mergeProcess(segRegions);
    }
    double segE = pcl::getTime();
    ROS_INFO("Segmentation found %d regions",segRegions.size());
    ROS_INFO("Segmentation took: %f",segE-segS);

    pubCCloud(&seg_regions_pub_,segRegions);

// Starting plane finder - plane extraction //

    ROS_INFO("Starting plane finder");

    double pfS = pcl::getTime();
    planeshape psProc;
    psProc.loadConfig(config_["planeshape"]);
    regions stairTreads;
    regions stairRisers;
    psProc.setInputRegions(segRegions);
    // extract treads and risers regions from segRegions
    psProc.filterSc(stairTreads, stairRisers);

    double pfE = pcl::getTime();
    ROS_INFO("Plane filter took: %f",pfE-pfS);

    // pubCCloud(&rise_regions_pub_,stairRisers);
    // pubCCloud(&tread_regions_pub_,stairTreads);
    // pubLabels(&tread_region_nums_pub_,stairTreads); 

// eigen based stair detection/prediction

    ROS_INFO("Starting prediction");
    prediction pred;
    pred.setTreadRegions(stairTreads);
    // // pred.setRiseRegions(stairRisers);
    pred.setFixedTform(T_fixed_input_mat.cast<float>());
    pred.setFixedFrame(fixed_frame_id_);
    pred.run();
    stairTreads.clear();
    pred.getFilteredRegions(stairTreads);
    geometry_msgs::PoseStamped step_pose;
    pred.getNearestStepPose(step_pose);

    pubPose(&nearest_step_pose_pub_,step_pose);
    pubCCloud(&rise_regions_pub_,stairRisers);
    pubCCloud(&tread_regions_pub_,stairTreads);
    pubLabels(&tread_region_nums_pub_,stairTreads);

    busy = false; 
    // return;

// Starting graph-based stair detection //

    ROS_INFO("Starting graph-based detection");
    StairVector detectedStairs;

    double refS = pcl::getTime();
    recognition stairDetect;
    stairDetect.loadConfig(config_["recognition"]);
    stairDetect.setInputRegions(segRegions);
    stairDetect.setStairTreadRegions(stairTreads);
    stairDetect.setStairRiseRegions(stairRisers);
    // filter
    // // filter rise regions
    // // // filter: horizontal distance
    // // // filter: normal angle difference
    // // // filter: height initializations
    // // // filter: vertical distance
    // // // filter: inlination
    // // // find (extend)
    // // // check
    // // filter tread regions
    // // // filter: vertical distance
    // // // filter: direction initilizations
    // // // filter: depth initilizations
    // // // filter: horizontal distance
    // // // filter: inlination
    // // // find (extend)
    // // // check
    // sort
    // either simple search
    // // finalize
    // or extended search
    // // finalize
    stairDetect.run(detectedStairs);
    double refE = pcl::getTime();

    ROS_INFO("There are treads: %d",stairTreads.size());
    ROS_INFO("There are risers: %d",stairRisers.size());

    ROS_INFO("Refinement took: %f",refE-refS);
    ROS_INFO("Total time  took: %f",refE-loadS);

    if (detectedStairs.size() > 0)
    {
        pubTCloud(&rise_cloud_pub_,detectedStairs.at(0).stairRiseCloud); 
        pubTCloud(&tread_cloud_pub_,detectedStairs.at(0).stairTreadCloud);
        pubTCloud(&rail_cloud_pub_,detectedStairs.at(0).stairRailCloud);
    }

// Printing out the results //

    bool colorByPart = true;

    PointCloudC resultCloud;

    bool addBackGround = true;
    if(addBackGround)
    {
    	for(size_t pointID = 0; pointID < mainCloud->size(); pointID ++)
    	{
    		PointTC backPoint;
    		backPoint.x = mainCloud->points[pointID].x;
    		backPoint.y = mainCloud->points[pointID].y;
    		backPoint.z = mainCloud->points[pointID].z;
    		backPoint.r=255;
    		backPoint.g=255;
    		backPoint.b=255;
    		resultCloud.push_back(backPoint);
    	}
    }

    ROS_INFO("Detected stairways: %d",detectedStairs.size());
    if(detectedStairs.size()>0)
    {
		for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
		{
			Stairs stairCoefficients;
			stairCoefficients = detectedStairs.at(stairCoeffIdx);

			float slope = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

            ROS_INFO("-");
			ROS_INFO("Step depth:   %f",round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2))));
			ROS_INFO("Step height:  %f",round(1000*stairCoefficients.dir[2]));
			ROS_INFO("Step width:   %f",round(1000*stairCoefficients.width));
			ROS_INFO("Slope is:     %f",round(100*slope/M_PI*180));
			ROS_INFO("Amount of stair parts: %d",stairCoefficients.size());

			float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
			float xStairDist = stairCoefficients.pos[0];
			float yStairDist = stairCoefficients.pos[1];

			Eigen::Vector2f sepDist;
			sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
			sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

            ROS_INFO("-");
	        ROS_INFO("Dist in X is: %f",round(1000*(stairCoefficients.pos[0])));
	        ROS_INFO("Dist in Y is: %f",round(1000*stairCoefficients.pos[1]));

	        ROS_INFO("Dist par is:  %f",round(1000*sepDist[0]));
	        ROS_INFO("Dist ort is:  %f",round(1000*sepDist[1]));
			ROS_INFO("Anchor point is: %d",stairCoefficients.anchPoint);

			ROS_INFO("Angle is:     %f",round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180));

			if(colorByPart)
				resultCloud += detectedStairs.getColoredParts(stairCoeffIdx);
			else
				resultCloud += detectedStairs.getColoredCloud(stairCoeffIdx);
        
        }
        
    } // if(detectedStairs.size()>0)

    if(detectedStairs.size()>0)
    {
        uint stairCoeffIdx = 0;

        Stairs stairCoefficients = detectedStairs.at(stairCoeffIdx);
        regions stairParts = stairCoefficients.stairParts;
        std::vector<int> planeLabels = stairCoefficients.planeLabels;
        ROS_INFO(stairCoefficients.str().c_str());

        pubCCloud(&stair_parts_pub_,stairParts);
        pubCCloud(&seg_stairs_pub_,resultCloud);
    } // if(detectedStairs.size()>0)

    // busy = false;
}

int main (int argc, char *argv[])
{  
    ros::init(argc, argv, "stair_detection_node");
    ros::NodeHandle n;
    ros::Subscriber input_sub = n.subscribe("input_cloud",1,&inputCB);

    if(argc < 2){ 
		ROS_ERROR("Usage: %s <config_filepath>",argv[0]);
        return 0;
	}
    else
    {
        config_filepath_ = argv[1]; 
    }

    config_ = YAML::LoadFile(config_filepath_);
    fixed_frame_id_ = config_["ros"]["fixed_frame_id"].as<std::string>();

    main_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("main_cloud",1);
    normal_cloud_pub_ = n.advertise<visualization_msgs::Marker>("normal_cloud",1);
    seg_regions_pub_ = n.advertise<visualization_msgs::Marker>("segmented_regions", 1);
    rise_regions_pub_ = n.advertise<visualization_msgs::Marker>("rise_regions", 1);
    tread_regions_pub_ = n.advertise<visualization_msgs::Marker>("tread_regions", 1);
    tread_region_nums_pub_ = n.advertise<visualization_msgs::MarkerArray>("tread_region_nums",1);
    rise_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("rise_cloud", 1);
    tread_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("tread_cloud", 1);
    rail_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("rail_cloud", 1);
    seg_stairs_pub_ = n.advertise<visualization_msgs::Marker>("segmented_stairs", 1);
    stair_parts_pub_ = n.advertise<visualization_msgs::Marker>("stair_parts",1);
    nearest_step_pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("nearest_step_pose",1);

    ros::spin();
}

