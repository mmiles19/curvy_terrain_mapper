#include <curvy_terrain_mapper/prediction.h>

// prediction::prediction()
// {
	
// }

void prediction::print()
{
	if (filtered_idxs_.empty()){ return; }

	for (uint i=0; i<filtered_idxs_.size(); i++)
	{
		uint idx = filtered_idxs_[i];
		DerivedAnalysisResult der_ana_result = result_set_.at(idx);
		AnalysisResult ana_result = der_ana_result.analysis_result_;
		ROS_INFO(
			// "------"
			"\nStair %d"
			"\ncovariance: [[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]]" // row-wise
			"\ncentroid x: %f y: %f z: %f"
			"\neigen_vectors: [[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]]" // vector-/column-wise
			"\neigen_values: %f %f %f"
			"\ncoefficient: %f %f %f %f"
			"\ncurvature: %f"
			"\ntrav/width eigval rat: %f"
			"\nnormal/width eigval rat: %f"
			"\nnormal up dot: %f"
			// "\n--> %f"
			"\n------"
			,idx
			,ana_result.covariance(0,0),ana_result.covariance(0,1),ana_result.covariance(0,2)
			,ana_result.covariance(1,0),ana_result.covariance(1,1),ana_result.covariance(1,2)
			,ana_result.covariance(2,0),ana_result.covariance(2,1),ana_result.covariance(2,2)
			,ana_result.centroid(0),ana_result.centroid(1),ana_result.centroid(2)
			,ana_result.eigen_vectors(0,0),ana_result.eigen_vectors(1,0),ana_result.eigen_vectors(2,0)
			,ana_result.eigen_vectors(0,1),ana_result.eigen_vectors(1,1),ana_result.eigen_vectors(2,1)
			,ana_result.eigen_vectors(0,2),ana_result.eigen_vectors(1,2),ana_result.eigen_vectors(2,2)
			,ana_result.eigen_values(0),ana_result.eigen_values(1),ana_result.eigen_values(2)
			,ana_result.coefficient(0),ana_result.coefficient(1),ana_result.coefficient(2),ana_result.coefficient(3)
			,ana_result.curvature
			,der_ana_result.ratio_trav_width_
			,der_ana_result.ratio_normal_width_
			,der_ana_result.dot_normal_up_
			// ,
		);
	}
}


// void prediction::clusterPatches()
// {
// 	treads_.generateCenterCloud();
// 	PointCloudT::Ptr cloud = new(PointCloudT(treads_.centerCloud));

// 	// Creating the KdTree object for the search method of the extraction
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
// 	tree->setInputCloud (cloud);

// 	std::vector<pcl::PointIndices> cluster_indices;
// 	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 	ec.setClusterTolerance (0.05); 
// 	ec.setMinClusterSize (10);
// 	ec.setMaxClusterSize (1000);
// 	ec.setSearchMethod (tree);
// 	ec.setInputCloud (cloud);
// 	ec.extract (cluster_indices);

// 	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
// 	{
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
// 		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
// 			cloud_cluster->points.push_back (cloud->points[*pit]); //*
// 		cloud_cluster->width = cloud_cluster->points.size ();
// 		cloud_cluster->height = 1;
// 		cloud_cluster->is_dense = true;
// 		clusters.push_back(cloud_cluster);
// 	}
// }

// void prediction::extractPlanes()
// {
// 	cloud = _cloud;

// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
// 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// 	// Create the segmentation object
// 	pcl::SACSegmentation<PointT> seg;
// 	// Optional
// 	seg.setOptimizeCoefficients (true);
// 	// Mandatory
// 	seg.setModelType (pcl::SACMODEL_PLANE);
// 	seg.setMethodType (pcl::SAC_RANSAC);
// 	seg.setDistanceThreshold (0.05);

// 	seg.setInputCloud (cloud);
// 	seg.segment (*inliers, *coefficients);

// 	if (inliers->indices.size () == 0)
// 	{
// 		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
// 		return (-1);
// 	}

// 	// std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
// 	// 									<< coefficients->values[1] << " "
// 	// 									<< coefficients->values[2] << " " 
// 	// 									<< coefficients->values[3] << std::endl;

// 	// std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
// 	// for (std::size_t i = 0; i < inliers->indices.size (); ++i)
// 	// 	std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
// 	// 											<< cloud->points[inliers->indices[i]].y << " "
// 	// 											<< cloud->points[inliers->indices[i]].z << std::endl;

// 	_inliers = inliers;
// 	_coefficients = coefficients;
// }

void prediction::loadConfig(YAML::Node config)
{
	// angleMargin = config["angleMargin"].as<float>();
}

void prediction::analyse()
{
	for (uint i=0; i<treads_.size(); i++)
	{	
		AnalysisResult ana_result = treads_.at(i).analyse_without_indices();

		// this assumes that the eigen values/vectors are sorted from low to high
		DerivedAnalysisResult der_ana_result(ana_result);

		result_set_.push_back(der_ana_result);
		filtered_idxs_.push_back(i);
	}
}

void prediction::filterByTravWidthEigValRatio(float min, float max)
{
	if (filtered_idxs_.empty()){ return; }
	
	auto filtered_idxs_copy = filtered_idxs_;
	filtered_idxs_copy.clear();
	
	for (uint i=0; i<filtered_idxs_.size(); i++)
	{
		float ratio = result_set_.at(filtered_idxs_[i]).ratio_trav_width_;
		if ((ratio > min) && (ratio < max))
		{
			filtered_idxs_copy.push_back(i);
		}
	}

	filtered_idxs_ = filtered_idxs_copy;
}

void prediction::filterByNormalWidthEigValRatio(float min, float max)
{
	if (filtered_idxs_.empty()){ return; }
	
	auto filtered_idxs_copy = filtered_idxs_;
	filtered_idxs_copy.clear();
	
	for (uint i=0; i<filtered_idxs_.size(); i++)
	{
		float ratio = result_set_.at(filtered_idxs_[i]).ratio_normal_width_;
		if ((ratio > min) && (ratio < max))
		{
			filtered_idxs_copy.push_back(i);
		}
	}

	filtered_idxs_ = filtered_idxs_copy;
}

void prediction::filterByNormalUpEigVec(float tol)
{
	if (filtered_idxs_.empty()){ return; }
	
	auto filtered_idxs_copy = filtered_idxs_;
	filtered_idxs_copy.clear();
	
	for (uint i=0; i<filtered_idxs_.size(); i++)
	{
		Eigen::Vector3f normal = result_set_.at(filtered_idxs_[i]).analysis_result_.eigen_vectors.col(result_set_.at(filtered_idxs_[i]).normal_idx_);
		Eigen::Vector3f up(0,0,1);
		float proj = normal.dot(up);
		result_set_.at(filtered_idxs_[i]).dot_normal_up_ = proj;
		if ((fabs(proj) > tol) /*&& (ratio < 1.0)*/)
		{
			filtered_idxs_copy.push_back(i);
		}
	}

	filtered_idxs_ = filtered_idxs_copy;
}

void prediction::extractStairDir()
{
	if (filtered_idxs_.size()<2){ return; }
	
	// auto filtered_idxs_copy = filtered_idxs_;
	// filtered_idxs_copy.clear();

	// std::vector<float> weights;
	Eigen::Vector3f weighted_dir(0,0,0);
	std::vector<Eigen::Vector3f> diff_vecs;
	for (uint i=0; i<filtered_idxs_.size()-1; i++)
	{
		for (uint j=i+1; j<filtered_idxs_.size(); j++)
		{
			// if (i==j){ continue; }
			Eigen::Vector4f point0 = treads_.at(i).segmentCentroid;
			Eigen::Vector4f point1 = treads_.at(j).segmentCentroid;
			Eigen::Vector3f diff_vec = point0.head(3)-point1.head(3);
			diff_vec = (diff_vec.sum()>0 ? 1 : -1)*diff_vec;
			diff_vec.normalize();
			diff_vecs.push_back(diff_vec);

			Eigen::Vector3f point0_input = Eigen::Vector4f(T_fixed_input_mat_*point0).head(3);
			Eigen::Vector3f point1_input = Eigen::Vector4f(T_fixed_input_mat_*point1).head(3);
			float strong_nearness_factor = 1.0; // loosely corresponds to norm at which weight saturates
			float weight = std::min(float(10.0),float(pow(10,strong_nearness_factor/point0_input.norm())*pow(10,strong_nearness_factor/point1_input.norm())));
			weighted_dir += diff_vec*weight;

			ROS_INFO("diff %d->%d %f %f %f",i,j,diff_vec[0],diff_vec[1],diff_vec[2]);
		}
	}
	weighted_dir.normalize();
	stair_dir_ = weighted_dir;
	ROS_INFO("weighted dir %f %f %f",weighted_dir[0],weighted_dir[1],weighted_dir[2]);

	std::vector<Eigen::Vector3f> dist_vecs;
	std::vector<float> dist_norms;
	Eigen::Vector3f nearest_step;
	float min_norm = INFINITY;
	for (uint j=0; j<filtered_idxs_.size(); j++)
	{
		Eigen::Vector4f stairctr = treads_.at(j).segmentCentroid;
		Eigen::Vector4f stairctr_from_input = T_fixed_input_mat_*stairctr;
		Eigen::Vector3f stairctr_head = stairctr_from_input.head(3);
		float stairctr_head_norm = stairctr_head.norm();
		dist_vecs.push_back(stairctr_head);
		dist_norms.push_back(stairctr_head_norm);
		// ROS_INFO("dist %d norm %f vec %f %f %f",j,stairctr_head_norm,stairctr_head[0],stairctr_head[1],stairctr_head[2]);

		if (stairctr_head_norm < min_norm)
		{
			min_norm = stairctr_head_norm;
			nearest_step = stairctr.head(3);
		}
	}
	nearest_step_ = nearest_step;
	// ROS_INFO("nearest step %f %f %f",nearest_step[0],nearest_step[1],nearest_step[2]);

	// std::vector<uint> dist_idx_vec;
	// iota(dist_idx_vec.begin(), dist_idx_vec.end(), 0);
  	// stable_sort(dist_idx_vec.begin(), dist_idx_vec.end(), [&dist_norms](size_t i1, size_t i2) {return dist_norms[i1] < dist_norms[i2];} );

  	// stable_sort(dist_vecs.begin(), dist_vecs.end(), [&dist_vecs](size_t i1, size_t i2) {return dist_vecs[i1].norm() < dist_vecs[i2].norm();} );
	
	// std::vector<float> weights;
	// for (std::vector<float>::iterator it=dist_norms.begin(); it!=dist_norms.end(); it++)
	// {
	// 	weights.push_back(pow(10,*it));
	// }
	// float weights_sum = std::accumulate(weights.begin(), weights.end(),0);
	// for (std::vector<float>::iterator it=weights.begin(); it!=weights.end(); it++)
	// {
	// 	(*it) = (*it)/weights_sum;
	// }
	// Eigen::Vector3f weighted_dir(0,0,0);
	// for (uint j=0; j<filtered_idxs_.size(); j++)
	// {
	// 	weighted_dir += dist_vecs[j]*weights[j];
	// }
	// weighted_dir.normalize();
	// ROS_INFO("weighted dir %f %f %f",weighted_dir[0],weighted_dir[1],weighted_dir[2]);

	// Eigen::Vector3f median_vec;
	// std::vector<uint> median_idx_vec;
	// medianVec(vecs, &median_vec, &median_idx_vec);

	// std::vector<float> x_vec;
	// std::vector<float> y_vec;
	// std::vector<float> z_vec;
	// for (uint i=0; i<diff_vecs.size(); i++)
	// {	
	// 	x_vec.push_back(diff_vecs[i][0]);
	// 	y_vec.push_back(diff_vecs[i][1]);
	// 	z_vec.push_back(diff_vecs[i][2]);
	// }

	// std::sort(x_vec.begin(),x_vec.end());
	// std::sort(y_vec.begin(),y_vec.end());
	// std::sort(z_vec.begin(),z_vec.end());

	// float x_med = ( x_vec.size() % 2 == 0 ? (x_vec[x_vec.size() / 2 - 1] + x_vec[x_vec.size() / 2]) / 2 : x_vec[x_vec.size() / 2] );
	// float y_med = ( y_vec.size() % 2 == 0 ? (y_vec[y_vec.size() / 2 - 1] + y_vec[y_vec.size() / 2]) / 2 : y_vec[y_vec.size() / 2] );
	// float z_med = ( z_vec.size() % 2 == 0 ? (z_vec[z_vec.size() / 2 - 1] + z_vec[z_vec.size() / 2]) / 2 : z_vec[z_vec.size() / 2] ); 
	// Eigen::Vector3f median_vec(x_med, y_med, z_med);
	// median_vec.normalize();
	// ROS_INFO("median vec %f %f %f",median_vec[0],median_vec[1],median_vec[2]);

	// stair_dir_ = median_vec;

	// float x_mean = std::accumulate(x_vec.begin(),x_vec.end(),0.0)/float(x_vec.size());
	// float y_mean = std::accumulate(y_vec.begin(),y_vec.end(),0.0)/float(y_vec.size());
	// float z_mean = std::accumulate(z_vec.begin(),z_vec.end(),0.0)/float(z_vec.size()); 
	// Eigen::Vector3f mean_vec(x_mean, y_mean, z_mean);
	// ROS_INFO("mean vec %f %f %f",mean_vec[0],mean_vec[1],mean_vec[2]);



	// filtered_idxs_ = filtered_idxs_copy;
}

void prediction::getFilteredRegions(regions& filtered_treads)
{
	for (uint i=0; i<filtered_idxs_.size(); i++)
	{
		filtered_treads.push_back(treads_.at(i));
	}
}

void prediction::getNearestStepPose(geometry_msgs::PoseStamped& output)
{
	float roll = 0;
	// float pitch = atan(stair_dir_[2]/sqrt(pow(stair_dir_[0],2)+pow(stair_dir_[1],2)));
	float pitch = -asin(stair_dir_[2]);
	float yaw = atan2(stair_dir_[1],stair_dir_[0]);
	Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
	q.normalize();

	ROS_INFO("pitch %f yaw %f",pitch,yaw);

	output.header.frame_id = fixed_frame_id_;
	output.header.stamp = ros::Time::now();
	output.pose.position.x = nearest_step_[0];
	output.pose.position.y = nearest_step_[1];
	output.pose.position.z = nearest_step_[2];
	output.pose.orientation.x = q.x();
	output.pose.orientation.y = q.y();
	output.pose.orientation.z = q.z();
	output.pose.orientation.w = q.w();
}

// void prediction::medianVec(std::vector<Eigen::Vector3f>& vecs, Eigen::Vector3f* median_vec, std::vector<uint>* index_vec)
// {
// 	std::vector<float> x_vec;
// 	std::vector<float> y_vec;
// 	std::vector<float> z_vec;
// 	for (uint i=0; i<vecs.size(); i++)
// 	{	
// 		x_vec.push_back(vecs[i][0]);
// 		y_vec.push_back(vecs[i][1]);
// 		z_vec.push_back(vecs[i][2]);
// 	}

// 	std::sort(x_vec.begin(),x_vec.end());
// 	std::sort(y_vec.begin(),y_vec.end());
// 	std::sort(z_vec.begin(),z_vec.end());

// 	float x_med = ( x_vec.size() % 2 == 0 ? (x_vec[x_vec.size() / 2 - 1] + x_vec[x_vec.size() / 2]) / 2 : x_vec[x_vec.size() / 2] );
// 	float y_med = ( y_vec.size() % 2 == 0 ? (y_vec[y_vec.size() / 2 - 1] + y_vec[y_vec.size() / 2]) / 2 : y_vec[y_vec.size() / 2] );
// 	float z_med = ( z_vec.size() % 2 == 0 ? (z_vec[z_vec.size() / 2 - 1] + z_vec[z_vec.size() / 2]) / 2 : z_vec[z_vec.size() / 2] ); 
// 	median_vec = new Eigen::Vector3f(x_med, y_med, z_med);

// 	// for(std::vector<float>::iterator it = x_vec.begin(); it!= x_vec.end(); it++)
// 	// {
// 	// 	it = std::find(x_vec.begin(), x_vec.end(), 22);
// 	// }
// }

void prediction::run()
{
	// treads_ with z normals
	// Histogram<double> hist(10,10);		
	
	// PointCloudT fusedTreads = treads_.at(0).segmentCloud;
	// pcl::PointIndices::Ptr treadsIdxs (new pcl::PointIndices);	
	// treadsIdxs->indices.push_back(0);	
	// for (uint i=1; i<treads_.size(); i++)
	// {	
	// 	fusedTreads += treads_.at(i).segmentCloud;	
	// 	treadsIdxs->indices.push_back(i);
	// }
	
	analyse();

	// filterByTravWidthEigValRatio(0.0,0.1);
	// filterByNormalWidthEigValRatio(0.0,0.01);
	// filterByNormalUpEigVec(0.98);

	extractStairDir();

	print();

}


