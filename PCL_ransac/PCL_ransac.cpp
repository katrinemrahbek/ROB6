#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include "royale.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <thread>
#include <mutex>

#include "sample_utils/PlatformResources.hpp"

using namespace sample_utils;

#define VISUALIZE 0
#define DOWNSAMPLE 0

typedef pcl::PointXYZRGB PointT;

//used to store result from each ply file the algorithm is run on
struct test_result
{
	std::vector<float> radi;
	std::string folder_name;
	float radius;
	float std_dev;
	float fps;
};

struct path_leaf_string
{
	std::string operator()(const boost::filesystem::directory_entry& entry) const
	{
		return entry.path().leaf().string();
	}
};

void read_directory(const std::string& name, std::vector<std::string> &v)
{
	boost::filesystem::path p(name);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(v), path_leaf_string());
}

float calcRadius(pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);

	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0.05, 0.3);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);

	// SACMODEL_CYLINDER - used to determine cylinder models. The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);

#if VISUALIZE==1
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(*cropped_cloud);

	for (int i = 0; i < cropped_cloud->size(); i++)
		cropped_cloud->at(i).g = 255;

	pcl::visualization::PCLVisualizer viewer("data viewer");

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cloud_rgb(cloud);
	viewer.addPointCloud<PointT>(cloud, cloud_rgb, "sample cloud");
	//viewer.addPointCloud<PointT>(cloud, "sample cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cropped_cloud_rgb(cropped_cloud);
	viewer.addPointCloud<PointT>(cropped_cloud, cropped_cloud_rgb, "cropped cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	//coefficients_cylinder->values[0] = 0;
	//coefficients_cylinder->values[1] = 0;
	//coefficients_cylinder->values[2] = 0;
	//if (coefficients_cylinder->values[5] < 0)
	//	coefficients_cylinder->values[5] *= -1;
	viewer.addCylinder(*coefficients_cylinder, "cylinder");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif

	return coefficients_cylinder->values[6];
}

//Corrected sample standard deviation
float std_dev(std::vector<float> data)
{
	float mean = 0.0;
	for (int i = 0; i < data.size(); i++)
		mean += data[i] / (float)data.size();

	float sumMmeanSq = 0.0;
	for (int i = 0; i < data.size(); i++)
		sumMmeanSq += pow(data[i] - mean, 2);

	float std_dev = sqrt((1.0 / (float)(data.size() - 1))*sumMmeanSq);
	return std_dev;
}

//not better than calcRadius
float calcRadiusPlaneRemoval(pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients), coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices), inliers_plane(new pcl::PointIndices);

	pcl::PointCloud<PointT>::Ptr planeless_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);

	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Find plane in cloud, remove it, find cylinder in remaining cloud
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0.05, 0.3);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_plane, *coefficients_plane);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(true);
	extract.filter(*planeless_cloud);

	if (planeless_cloud->size() == 0)
		std::cout << "no plane found \n";
	else
	{
		for (int i = 0; i < planeless_cloud->size(); i++)
		{
			planeless_cloud->at(i).b = 255;
			planeless_cloud->at(i).x += 1.0;
		}
	}

	ne.setInputCloud(planeless_cloud);
	ne.compute(*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0.05, 0.3);
	seg.setInputCloud(planeless_cloud);
	seg.setInputNormals(cloud_normals);

	// SACMODEL_CYLINDER - used to determine cylinder models. The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
#if VISUALIZE

	extract.setInputCloud(planeless_cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(*cropped_cloud);

	for (int i = 0; i < cropped_cloud->size(); i++)
	{
		cropped_cloud->at(i).g = 255;
		cropped_cloud->at(i).x += 1.0;
	}


	pcl::visualization::PCLVisualizer viewer("data viewer");

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cloud_rgb(cloud);
	viewer.addPointCloud<PointT>(cloud, cloud_rgb, "sample cloud");
	//viewer.addPointCloud<PointT>(cloud, "sample cloud");

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> plane_cloud_rgb(planeless_cloud);
	viewer.addPointCloud<PointT>(planeless_cloud, plane_cloud_rgb, "plane cloud");

	pcl::visualization::PointCloudColorHandlerRGBField<PointT> cropped_cloud_rgb(cropped_cloud);
	viewer.addPointCloud<PointT>(cropped_cloud, cropped_cloud_rgb, "cropped cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	//coefficients_cylinder->values[0] = 0;
	//coefficients_cylinder->values[1] = 0;
	//coefficients_cylinder->values[2] = 0;
	//if (coefficients_cylinder->values[5] < 0)
	//	coefficients_cylinder->values[5] *= -1;
	viewer.addCylinder(*coefficients_cylinder, "cylinder");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
	return coefficients_cylinder->values[6];
}

//calculates test_result for each ply file in given folder.
test_result calcRadiusOnFolder(std::string folder_path)
{
	std::vector<std::string> files;
	read_directory(folder_path, files);

	pcl::PLYReader reader;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
	std::vector<float> radi;

	auto timing_start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < files.size(); i++)
	{
		// Read in the cloud data
		reader.read(folder_path + "/" + files[i], *cloud);
		//std::cout << "file " << i << " of " << files.size() << ": " << files[i] << " with " << cloud->size() << " points, ";
		if (cloud->size() == 0)
			continue;

		pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);

	#if DOWNSAMPLE==1
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.02f, 0.02f, 0.02f);
		sor.filter(*filtered_cloud);
		
		for (int i = 0; i < filtered_cloud->size(); i++)
		{
			if (filtered_cloud->at(i).z < 1.5)
				filtered_cloud->at(i).r = 255;
				cropped_cloud->push_back(filtered_cloud->at(i));
		}
		if (cropped_cloud->size() == 0)
			continue;
		float radius = calcRadius(cropped_cloud);
	#else
		for (int i = 0; i < cloud->size(); i++)
		{
			if (cloud->at(i).z < 1.5 && cloud->at(i).z != 0)
			{
				cloud->at(i).r = 255;
				cropped_cloud->push_back(cloud->at(i));
			}
		}
		//std::cout << "cropped size: " << cropped_cloud->size() << ", ";
		float radius = calcRadius(cropped_cloud);
	#endif
		//std::cout << "Got radius: " << radius << std::endl;
		radi.push_back(radius);
	}
	auto timing_stop = std::chrono::high_resolution_clock::now();

	float avg_radius = 0;
	for (auto r : radi)
	{
		//std::cout << r << ",";
		avg_radius += r / radi.size();
	}
	float standard_dev = std_dev(radi);

	test_result tr;
	tr.fps = files.size() / ((std::chrono::duration_cast<std::chrono::milliseconds>(timing_stop - timing_start).count())/1000.0);
	tr.radi = radi;
	tr.std_dev= standard_dev;
	tr.radius = avg_radius;
	tr.folder_name = folder_path;

	return tr;
}

int main(int argc, char** argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	std::cout << "reading data sets from: " << argv[1] << "\n";
	std::string data_set_folder_path = argv[1];//"Data/dry_pvc_400/";

	std::vector<std::string> files;
	read_directory(data_set_folder_path, files);
	std::sort(files.begin(), files.end(), [](std::string a, std::string b) {return a<b;});

	std::vector<test_result> test_results;
	for(auto file : files)
	{	
		std::string current_folder =  data_set_folder_path + "/" + file;
		std::cout << "working on: " << current_folder << "\n";
		auto tr = calcRadiusOnFolder(current_folder);
		test_results.push_back(tr);
	}

	for(auto r: test_results)
	{
		std::cout << r.folder_name << ", r: " << r.radius << ", std_dev: " << r.std_dev << ", fps: " << r.fps << "\n";
	}

	// no downsampling:
	//running on ply_files
	//std.dev for calcRadiusPlaneRemoval: 0.0180283, mean radius = 0.214116
	//std.dev for calcRadius: 0.00974209, mean radius = 0.207412
	//running on ply_files_tube
	//std.dev for calcRadiusPlaneRemoval: 0.000551955, mean radius = 0.202051, fps = 2.58699
	//std.dev for calcRadius: 0.00233632, mean radius = 0.201664, fps = 2.85144


	// downsampled:
	//running on ply_files
	//std.dev for calcRadiusPlaneRemoval: 0.0214987, mean radius = 0.256655
	//std.dev for calcRadius: 0.0204836, mean radius = 0.253375
	//running on ply_files_tube
	//std.dev for calcRadiusPlaneRemoval: 0.00668299., mean radius = 0.197018, fps = 3.53607
	//std.dev for calcRadius: 0.00241367, mean radius = 0.20197, fps = 3.78573

	return (0);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DepthDataToPCL(const royale::DepthData* data)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	cloud->height = data->height;
	cloud->width = data->width;
	//cloud.header.stamp = data->timeStamp;
	
	for (auto point : data->points)
	{
		pcl::PointXYZI p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		p.intensity = point.grayValue;
		if(p.z < 5.0)
			cloud->push_back(p);
	}
	return cloud;
}

class MyListener : public royale::IDepthDataListener
{
	struct MyFrameData
	{
		std::vector<uint32_t> exposureTimes;
		std::vector<std::string> asciiFrame;
	};

	pcl::visualization::PCLVisualizer::Ptr viewer;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
	std::mutex mut;

public:

	void spin()
	{
		while (!viewer->wasStopped())
		{
			{
				std::unique_lock<std::mutex> lck(mut);
				viewer->spinOnce(1);
			}
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	/**
	 * This callback is called for each depth frame that is captured.  In a mixed-mode use case
	 * (a use case with multiple streams), each callback refers to data from a single stream.
	 */
	void onNewData(const royale::DepthData *data) override
	{
		{
			//std::cout << "waiting for lock \n";
			{
				std::unique_lock<std::mutex> lck(mut);
				cloud = DepthDataToPCL(data);
			}

			if (!viewer->updatePointCloud<pcl::PointXYZI>(cloud, "sample cloud"))
			{
				//std::cout << "failed to update point cloud, is it first time? \n";
				viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
			}
		}		
	}

	/**
	 * Creates a listener which will have callbacks from two sources - the Royale framework, when a
	 * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
	 */
	explicit MyListener(const royale::Vector<royale::StreamId> &streamIds) :
		m_streamIds(streamIds)
	{
		viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("data viewer"));
		viewer->addCoordinateSystem(0.1);
		viewer->initCameraParameters();
	}

private:
	const royale::Vector<royale::StreamId> m_streamIds;
	std::map<royale::StreamId, MyFrameData> m_receivedData;
};

int VisualizerFromCameraWithPCLmain(int argc, char **argv)
{
	// Windows requires that the application allocate these, not the DLL.  We expect typical
	// Royale applications to be using a GUI toolkit such as Qt, which has its own equivalent of this
	// PlatformResources class (automatically set up by the toolkit).
	PlatformResources resources;

	// This is the data listener which will receive callbacks.  It's declared
	// before the cameraDevice so that, if this function exits with a 'return'
	// statement while the camera is still capturing, it will still be in scope
	// until the cameraDevice's destructor implicitly deregisters the listener.
	std::unique_ptr<MyListener> listener;

	// this represents the main camera device object
	std::unique_ptr<royale::ICameraDevice> cameraDevice;

	royale::CameraManager manager;

	auto camlist = manager.getConnectedCameraList();
	cout << "Detected " << camlist.size() << " camera(s)." << endl;
	if (!camlist.empty())
	{
		cout << "CamID for first device: " << camlist.at(0).c_str() << " with a length of (" << camlist.at(0).length() << ")" << endl;
		cameraDevice = manager.createCamera(camlist[0]);
	}

	// the camera device is now available and CameraManager can be deallocated here

	if (cameraDevice == nullptr)
	{
		cerr << "Cannot create the camera device" << endl;
		return 1;
	}

	// IMPORTANT: call the initialize method before working with the camera device
	if (cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
	{
		cerr << "Cannot initialize the camera device" << endl;
		return 1;
	}

	royale::Vector<royale::String> useCases;
	auto status = cameraDevice->getUseCases(useCases);
	for (auto stat : useCases)
		std::cout << stat << "\n";
	std::cout << "\n";

	if (status != royale::CameraStatus::SUCCESS || useCases.empty())
	{
		cerr << "No use cases are available" << endl;
		cerr << "getUseCases() returned: " << getErrorString(status) << endl;
		return 1;
	}

	// choose a use case
	auto selectedUseCaseIdx = 0u;
	std::cout << "Choosing useCase " << selectedUseCaseIdx << "\n";

	// set an operation mode
	if (cameraDevice->setUseCase(useCases.at(selectedUseCaseIdx)) != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error setting use case" << endl;
		return 1;
	}

	// Retrieve the IDs of the different streams
	royale::Vector<royale::StreamId> streamIds;
	if (cameraDevice->getStreams(streamIds) != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error retrieving streams" << endl;
		return 1;
	}

	// register a data listener
	listener.reset(new MyListener(streamIds));
	if (cameraDevice->registerDataListener(listener.get()) != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error registering data listener" << endl;
		return 1;
	}

	// start capture mode
	if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error starting the capturing" << endl;
		return 1;
	}

	while (true)
	{	
		listener->spin();
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	// stop capture mode
	if (cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error stopping the capturing" << endl;
		return 1;
	}

	return 0;
}
