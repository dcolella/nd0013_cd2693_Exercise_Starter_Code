#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc


#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <optional>
#include <chrono>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
std::chrono::time_point<std::chrono::system_clock> lastPredictionTime;

vector<ControlState> cs;

bool refresh_view = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box; //test
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

enum ScanMatchAlgo{ Off, Icp, Ndt, Hybrid, SpeedAdapt, Interpolation};

Eigen::Matrix4d getTransformWithICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Eigen::Matrix4d initTransform, int iterations, auto logger){

	// Defining a rotation matrix and translation vector
  	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  	// align source with starting pose
  	//Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
  	PointCloudT::Ptr transformSource (new PointCloudT); 
  	pcl::transformPointCloud (*source, *transformSource, initTransform);
  		
	pcl::console::TicToc time;
  	time.tic ();
  	pcl::IterativeClosestPoint<PointT, PointT> icp;
  	icp.setMaximumIterations (iterations);
  	icp.setInputSource (transformSource);
  	icp.setInputTarget (target);
	icp.setMaxCorrespondenceDistance (0.6);
	icp.setTransformationEpsilon(1e-5);
	icp.setEuclideanFitnessEpsilon(1e-4);
	
	

  	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  	icp.align (*cloud_icp);
  	

  	if (icp.hasConverged ())
  	{
  		transformation_matrix = icp.getFinalTransformation ().cast<double>();
  		transformation_matrix =  transformation_matrix * initTransform;

  		return transformation_matrix;
  	}
	else
  		//cout << "WARNING: ICP did not converge" << endl;
		logger.warn("ICP did not converge");
  	return transformation_matrix;

}




Eigen::Matrix4d getTransformWithNDT(PointCloudT::Ptr mapCloud, typename pcl::PointCloud<PointT>::Ptr cloudFiltered, Eigen::Matrix4d init_guess , int iterations, auto logger){
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// Setting minimum transformation difference for termination condition.
  	//ndt.setTransformationEpsilon (.0001);
	ndt.setTransformationEpsilon (0.01);
  	// Setting maximum step size for More-Thuente line search.
  	ndt.setStepSize (0.1);
  	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
  	ndt.setResolution (0.5);
  	ndt.setInputTarget (mapCloud);
	ndt.setMaximumIterations (iterations);
	ndt.setInputSource (cloudFiltered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt (new pcl::PointCloud<pcl::PointXYZ>);
  	ndt.align (*cloud_ndt, init_guess.cast<float>());
	//Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);

	Eigen::Matrix4d transformation_matrix;

	if (ndt.hasConverged()) {
		transformation_matrix = ndt.getFinalTransformation ().cast<double>();
    }
	else {
		//std::cout << "[WARNING] NDT did not converge" << std::endl;
		logger.warn("NDT did not converge");
        return init_guess.cast<double>();
	}

	

	return transformation_matrix;

}


Eigen::Matrix4d hybridScanMatch(
    PointCloudT::Ptr mapCloud,
    PointCloudT::Ptr scanCloud,
    Eigen::Matrix4d init_guess,
	auto logger,
    double voxel_size = 0.2,       // downsampling
    int ndt_iterations = 35,
    int icp_iterations = 50)
{
    // --- 1. Downsample scan for speed and stability ---
    PointCloudT::Ptr filteredScan(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(scanCloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*filteredScan);

    // --- 2. Coarse alignment with NDT ---
    Eigen::Matrix4d ndt_transform = getTransformWithNDT(mapCloud, filteredScan, init_guess, ndt_iterations, logger);

    // --- 3. Refine with ICP ---
    Eigen::Matrix4d icp_transform = getTransformWithICP(mapCloud, filteredScan, ndt_transform, icp_iterations, logger);

    // Optional: Check convergence / fitness score if you modify getTransformWithICP to return it
    // For now, fallback to NDT if ICP did not improve anything
    // (Your getTransformWithICP already prints warning if ICP fails)

    return icp_transform; // final aligned transform
}



/**
	Get current vehicle Speed in Meter per Seconds.
*/
double getVehicleSpeedMs(const carla::SharedPtr<carla::client::Vehicle>& vehicle){
	carla::geom::Vector3D vel = vehicle->GetVelocity();
	return std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
}

/**
	Get current vehicle Yaw rate in Radiant per Second.
*/
double getVehicleYawRateRadS(const carla::SharedPtr<carla::client::Vehicle>& vehicle){
	carla::geom::Vector3D ang = vehicle->GetAngularVelocity();
	return ang.z * M_PI / 180.0;
}

void printTime(std::chrono::time_point<std::chrono::system_clock> time, std::string label, auto logger){
	auto dur = time.time_since_epoch(); // duration since clock’s epoch
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

	//std::cout << label << ": " << ms << " ms since steady_clock epoch\n";
	logger.info("{}: {} ms since steady_clock epoch", label.c_str(), ms);
}

Pose predictPoseFromSpeedAndYawRate(const Pose current, double speed, double yaw_rate, std::chrono::time_point<std::chrono::system_clock> &lastPredictedPoseTime){

	auto now = std::chrono::system_clock::now();
	double dt = std::chrono::duration<double>(now - lastPredictedPoseTime).count();
	lastPredictedPoseTime = now;

	Pose predicted = current;

    double yaw   = current.rotation.yaw;
    double pitch = current.rotation.pitch;   // not used
    double roll  = current.rotation.roll;    // not used

    // ---- Predict orientation ----
    double newYaw = yaw + yaw_rate * dt;

    // ---- Predict translation ----
    predicted.position.x += speed * std::cos(newYaw) * dt;
    predicted.position.y += speed * std::sin(newYaw) * dt;
    predicted.position.z  = current.position.z;   // ignore Z motion for cars

    // ---- Store updated rotation ----
    predicted.rotation.yaw   = newYaw;
    predicted.rotation.pitch = pitch;
    predicted.rotation.roll  = roll;

    return predicted;

}

void printPose(Pose pose, std::string label, auto logger){
	//printf("%s Position: < %6.3f, %6.3f, %6.3f >\n", label.c_str(), pose.position.x, pose.position.y, pose.position.z);
	//printf("%s Rotation: < %6.3f, %6.3f, %6.3f >\n\n", label.c_str(), pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll);

	logger.info("{} Position: < {:6.3f}, {:6.3f}, {:6.3f} ",  label.c_str(),  pose.position.x, pose.position.y, pose.position.z);
	logger.info("{} Rotation: < {:6.3f}, {:6.3f}, {:6.3f} ",  label.c_str(), pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll);

}

Eigen::Matrix4d getLidarOffSetTransform(){
	return  transform3D(
		0, 0, 0,
		-0.5, 0, 1.8
	);
}

int main(){

	//auto logger = spdlog::basic_logger_mt("basic_logger", "dclogs.txt", true);
	//logger->set_level(spdlog::level::debug); 

	// Create a console sink (colored)
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
	console_sink->set_level(spdlog::level::info); // console log level

	// Create a file sink (truncate file each run)
	auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("dclogs.txt", true);
	file_sink->set_level(spdlog::level::info);    // file log level

	spdlog::logger logger("multi_logger", {console_sink, file_sink});
    logger.set_level(spdlog::level::info); // global log level

	/*
	logger->info("Application started");
    logger->warn("Hellow World!");
	logger->debug("Debug Hellow World!");
    logger->error("Oops, Something went wrong!");
	*/

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions

	double points_per_second = 500000;
	double rotation_frequency = 60;
	int channels = 32;
	double range = 30.0;

	double points_per_rotation = points_per_second / rotation_frequency;
	double points_per_channel_per_rotation = points_per_rotation / channels;
	double horizontal_angular_res_deg = 360 * channels / points_per_rotation;

	string upper_fov_str = "15";
	string lower_fov_str = "-25";
	string channels_str = std::to_string(channels);
	string range_str = std::to_string(range);
	string rotation_frequency_str = std::to_string(rotation_frequency);
	string points_per_second_str = std::to_string(points_per_second);

	
	lidar_bp.SetAttribute("upper_fov", upper_fov_str);
    lidar_bp.SetAttribute("lower_fov", lower_fov_str);
    lidar_bp.SetAttribute("channels", channels_str);
    lidar_bp.SetAttribute("range", range_str);
	lidar_bp.SetAttribute("rotation_frequency", rotation_frequency_str);
	lidar_bp.SetAttribute("points_per_second",points_per_second_str);

	logger.info("lidar attribute upper_fov: {}", upper_fov_str);
	logger.info("lidar attribute lower_fov: {}", lower_fov_str);
	logger.info("lidar attribute channels: {}", channels_str);
	logger.info("lidar attribute range: {}",range_str);
	logger.info("lidar attribute rotation_frequency: {}", rotation_frequency_str);
	logger.info("lidar attribute points_per_second: {}", points_per_second_str);
	logger.info("lidar points_per_rotation: {}", points_per_second_str);
	logger.info("lidar points_per_channel_per_rotation: {}", points_per_channel_per_rotation);
	logger.info("lidar horizontal_angular_res_deg: {:.3f}", horizontal_angular_res_deg);

	
	
	
/*
	lidar_bp.SetAttribute("channels", "32");               
	lidar_bp.SetAttribute("upper_fov", "15");               
	lidar_bp.SetAttribute("lower_fov", "-25");             
	lidar_bp.SetAttribute("range", "30");                  
	lidar_bp.SetAttribute("rotation_frequency", "60");     
	lidar_bp.SetAttribute("points_per_second", "500000"); 
	*/
	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	bool first_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime, endTime;
	

	pcl::console::TicToc scan_processing_timer;

	

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));
	

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	//typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr lastCloudFiltered (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &scan_processing_timer, &lastScanTime, &scanCloud](auto data){

		if(new_scan){
			scan_processing_timer.tic();
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
					pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	lastPredictionTime = std::chrono::system_clock::now();

	double total_processing_time = 0;
	double avg_processing_elapsed_time = 0;
	double processing_elapsed_time = 0;
	
	int scan_count = 0;


	
	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		printPose(truePose, "True Pose", logger);
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();		
		
		
		
		if(!new_scan){
			scan_count++;
	
			new_scan = true;

			// TODO: (Filter scan using voxel filter)
			pcl::VoxelGrid<PointT> vg;
			vg.setInputCloud(scanCloud);
			typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

			Eigen::Matrix4d transform;

			double vehicle_speed = getVehicleSpeedMs(vehicle);
			double vehicle_yaw_rate = getVehicleYawRateRadS(vehicle);

			//std::cout << "speed:" << vehicle_speed << endl;
			//std::cout << "yaw rate:" << vehicle_yaw_rate << endl;
			logger.info("vehicle speed: {:.4f}", vehicle_speed);
			logger.info("vehicle yaw rate: {:.4f}", vehicle_yaw_rate);

			printPose(truePose, "truePose.", logger);	


			
			if (vehicle_speed > 10000000) {			
				
				double filterRes = 0.2;
				
				vg.setLeafSize(filterRes, filterRes, filterRes);
				
				vg.filter(*cloudFiltered);


				//pose = predictPoseFromSpeedAndYawRate(pose, vehicle_speed, vehicle_yaw_rate, lastPredictionTime);

				// *** Applying Lidar offset
				//transform = getLidarOffSetTransform() * transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
				
				
				transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
				
				transform = getTransformWithICP(mapCloud, cloudFiltered, getTransformWithNDT(mapCloud, cloudFiltered, transform, 50, logger), 50, logger);
				
				pose = getPose(transform);
				//std::cout << "Predicted pose using motion model." << std::endl;
				logger.info("Predicted pose using motion model.");
				printPose(pose, "predictedWithMotionModel.", logger);
			} else {
				
				double filterRes = 0.2;
				
				vg.setLeafSize(filterRes, filterRes, filterRes);
				
				vg.filter(*cloudFiltered);


				
				// *** Applying Lidar offset
				//transform = getLidarOffSetTransform() * transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);

				transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
				//transform = getTransformWithNDT(mapCloud, cloudFiltered, transform, 50, logger);
				//transform = getTransformWithICP(mapCloud, cloudFiltered, transform, 50, logger);
				if (lastCloudFiltered->empty()) {
					transform = getTransformWithICP(mapCloud, cloudFiltered, transform, 50, logger);
					*lastCloudFiltered = *cloudFiltered;
					pcl::copyPointCloud(*cloudFiltered, *lastCloudFiltered);
					std::cout << "lastCloudFiltered empty, using mapCloud for matching." << std::endl;

				}else{
					transform = getTransformWithICP(lastCloudFiltered, cloudFiltered, transform, 50, logger);
					pcl::copyPointCloud(*cloudFiltered, *lastCloudFiltered);
					std::cout << "Using lastCloudFiltered scan for matching." << std::endl;

				}
				
				
				
				pose = getPose(transform);
				//std::cout << "Speed is zero, skipping motion model prediction." << std::endl;
				logger.info("Skipping motion model prediction.");
				printPose(pose, "predictedWithoutMotionModel.", logger);
				//if (lastPredictionTime == std::chrono::time_point<std::chrono::system_clock>()) {
				
				//}
				
			}
			
			
			
			// TODO: Find pose transform by using ICP or NDT matching
			//pose = ....
			
			processing_elapsed_time = scan_processing_timer.toc ();

			avg_processing_elapsed_time = (total_processing_time + processing_elapsed_time) / scan_count;

			logger.info("Processing Time: {} ms", processing_elapsed_time);
			logger.info("Average Processing Time: {} ms", avg_processing_elapsed_time);


			
			/*
			Eigen::Matrix4d init_guess = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,pose.position.x, pose.position.y, pose.position.z);

			// Hybrid NDT → ICP
			Eigen::Matrix4d transform = hybridScanMatch(mapCloud, scanCloud, init_guess);
			pose = getPose(transform); // update your pose
			*/

			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr transformed_scan (new PointCloudT);
			pcl::transformPointCloud (*cloudFiltered, *transformed_scan, transform);

			viewer->removePointCloud("scan");
			// TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );

			//std::cout << "Pose Error: " << poseError << " meters." << std::endl;
			logger.info("Pose Error: {} meters.", poseError);

			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			//**displaying lidar params and processing performances */
			viewer->removeShape("ppr");
			viewer->addText("Points per Rotations: "+to_string(points_per_rotation), 900, 200, 32, 1.0, 1.0, 1.0, "ppr",0);

			viewer->removeShape("avgpt");
			viewer->addText("Average Processing Time per scan: "+to_string(avg_processing_elapsed_time), 900, 150, 32, 1.0, 1.0, 1.0, "avgpt",0);
			

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}

			pclCloud.points.clear();
		}

		
		
  	}
	return 0;
}