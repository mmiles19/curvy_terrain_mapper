#ifndef PREDICTION
#define PREDICTION

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <curvy_terrain_mapper/regions.h>
#include <curvy_terrain_mapper/StairVector.h>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <numeric>
#include <cmath>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;

class DerivedAnalysisResult {
	public:
  
    AnalysisResult analysis_result_;
    uint width_idx_;
    uint trav_idx_;
    uint normal_idx_;
    float ratio_trav_width_;
    float ratio_normal_width_;
    float dot_normal_up_;
    Eigen::Vector3f stair_dir_;

    inline DerivedAnalysisResult(AnalysisResult ana_result)
    { 
		  width_idx_ = 2;
		  trav_idx_ = 1;
		  normal_idx_ = 0;

      analysis_result_ = ana_result;

      ratio_trav_width_ = analysis_result_.eigen_values(trav_idx_)/analysis_result_.eigen_values(width_idx_);
		  ratio_normal_width_ = analysis_result_.eigen_values(normal_idx_)/analysis_result_.eigen_values(width_idx_);
      
		}
};

// template<
//   typename T, //real type
//   typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type
// >
// class Histogram {
// public:

//   uint nbins;
//   uint ndim;
//   std::vector< std::vector< T > > data;

//   inline Histogram(uint number_of_bins, uint bin_size)
//   {
//     nbins = number_of_bins;
//     ndim = bin_size;
//     data = std::vector<std::vector<T>>(nbins, std::vector<T>(ndim));
//   }

//   inline std::vector<T> at(int i)
//   {
//     return data.at(i);
//   }

//   inline void sort(){ std::sort(data.begin(), data.end()); }
  
//   inline T getMedian()
//   {
//     size_t size = getSize();
//     if (size == 0)
//     {
//       return 0;  // Undefined, really.
//     }
//     else
//     {
//       sort();
//       if (size % 2 == 0)
//       {
//         return (data[size / 2 - 1] + data[size / 2]) / 2;
//       }
//       else 
//       {
//         return data[size / 2];
//       }
//     }
//   }

//   inline T getSum(){ return std::accumulate(data.begin(), data.end(), 0); }

//   inline size_t getSize(
	// segmentPatch fusedTreads(treads.at(0));){ return data.size(); }

//   inline T getAvg(){ return getSum()/getSize(); }
// };

class prediction {
public:

	// prediction();

  // regions stairRiseRegions;
  regions treads_;
  std::vector<DerivedAnalysisResult> result_set_;
  std::vector<uint> filtered_idxs_;
  Eigen::Matrix4f T_fixed_input_mat_;
  std::string fixed_frame_id_;
  Eigen::Vector3f stair_dir_;
  Eigen::Vector3f nearest_step_;

  void loadConfig(YAML::Node);
  // inline void setRiseRegions(regions risers){	stairRiseRegions = risers; }
  inline void setTreadRegions(regions treads){ treads_ = treads; }
  inline void setFixedFrame(std::string frame){ fixed_frame_id_ = frame; }
  inline void setFixedTform(Eigen::Matrix4f T_fixed_input_mat){ T_fixed_input_mat_ = T_fixed_input_mat; }
  void analyse();
  void filterByTravWidthEigValRatio(float,float);
  void filterByNormalWidthEigValRatio(float,float);
  void filterByNormalUpEigVec(float);
  void extractStairDir();
  void getFilteredRegions(regions&);
  void getNearestStepPose(geometry_msgs::PoseStamped&);
  void print();
  
  void run();

};

#endif // RECOGNITION

