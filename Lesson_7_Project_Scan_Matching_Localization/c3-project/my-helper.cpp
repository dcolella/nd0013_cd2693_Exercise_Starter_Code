#include <pcl/registration/icp.h>


Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // align source with starting pose
    Eigen::Matrix4d initTransform = transform2D(startingPose.theta, startingPose.position.x, startingPose.position.y);
    PointCloudT::Ptr transformSource (new PointCloudT); 
    pcl::transformPointCloud (*source,* transformSource, initTransform);

    pcl::console::TicToc time;
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations (iterations);
	icp.setInputSource (transformSource);
	icp.setInputTarget (target);
	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
	icp.align (*cloud_icp);

	if (icp.hasConverged ())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
		transformation_matrix = icp.getFinalTransformation ().cast<double>();
		transformation_matrix =  transformation_matrix * initTransform;
		return transformation_matrix;
	}
	cout << "WARNING: ICP did not converge" << endl;

    return transformation_matrix;

}