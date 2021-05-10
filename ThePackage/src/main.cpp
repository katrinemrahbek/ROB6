#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <list>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include "royale.hpp"
#include "royale/IPlaybackStopListener.hpp"
#include "royale/IReplay.hpp"
#include "sample_utils/PlatformResources.hpp"

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

#include "matplotlibcpp.h"

#define MAX_Z_DISTANCE 0.75
#define EXPOSURE_SETTING 1000
#define USECASE_SETTING 1
#define VISUALIZE 1

namespace plt = matplotlibcpp;

//globals to communicate and protect between threads
std::mutex picoMut;
typedef pcl::PointXYZI PointT;

int setupCamera(std::shared_ptr<royale::ICameraDevice> &cameraDevice, int selectedUseCase, int exposure)
{
	royale::CameraManager manager;

	auto camlist = manager.getConnectedCameraList();
	//std::cout << "Detected " << camlist.size() << " camera(s)." << std::endl;
	if (!camlist.empty())
	{
		std::cout << "CamID for first device: " << camlist.at(0).c_str() << " with a length of (" << camlist.at(0).length() << ")" << std::endl;
		cameraDevice = manager.createCamera(camlist[0]);
	}
	//std::cout << "created camera\n";

	if (cameraDevice == nullptr)
	{
		std::cerr << "Cannot create the camera device" << std::endl;
		return 1;
	}
	//std::cout << "before initialize\n";

	// IMPORTANT: call the initialize method before working with the camera device
	if (cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
	{
		std::cerr << "Cannot initialize the camera device" << std::endl;
		return 1;
	}
	//std::cout << "after cam initialize\n";


	/* available usecases from the pico flex
	0	MODE_9_5FPS_2000
	1	MODE_9_10FPS_1000
	2	MODE_9_15FPS_700
	3	MODE_9_25FPS_450
	4	MODE_5_35FPS_600
	5	MODE_5_45FPS_500
	6	MODE_MIXED_30_5
	7	MODE_MIXED_50_5
	8	Low_Noise_Extended
	9	Fast_Acquisition
		*/

	royale::Vector<royale::String> useCases;
	auto status = cameraDevice->getUseCases(useCases);
	//std::cout << "got use cases\n";
	//for (auto stat : useCases)
	//	std::cout << stat << "\n";
	//std::cout << "\n";

	if (status != royale::CameraStatus::SUCCESS || useCases.empty())
	{
		std::cerr << "No use cases are available" << std::endl;
		std::cerr << "getUseCases() returned: " << getErrorString(status) << std::endl;
		return 1;
	}
	// choose a use case
	auto selectedUseCaseIdx = selectedUseCase;
	if (cameraDevice->setUseCase(useCases.at(selectedUseCaseIdx)) != royale::CameraStatus::SUCCESS)
	{
		std::cerr << "Error setting use case" << std::endl;
		return 1;
	}
	//std::cout << "before setexposuremode\n";

	if (cameraDevice->setExposureMode(royale::ExposureMode::MANUAL) != royale::CameraStatus::SUCCESS)
	{
		std::cout << "failed to set fixed exposure mode\n";
		return 1;
	}
    if (cameraDevice->setExposureTime(exposure) != royale::CameraStatus::SUCCESS)
    {
        std::cout << "failed to set exposure: " << exposure << "\n";
    }
	uint16_t maxFrameRate;
	cameraDevice->getFrameRate(maxFrameRate);
	std::cout << "Framerate: " << maxFrameRate << "\n";
	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthDataToPCL(royale::DepthData* data)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->height = data->height;
	cloud->width = data->width;
	//cloud.header.stamp = data->timeStamp;

	for (auto point : data->points)
	{
		if(point.z < MAX_Z_DISTANCE && point.z != 0)
		{
			pcl::PointXYZ p;
			p.x = point.x;
			p.y = point.y;
			p.z = point.z;
			cloud->push_back(p);
		}
	}
	return cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DepthDataToPCLI(royale::DepthData* data)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	cloud->height = data->height;
	cloud->width = data->width;
	//cloud.header.stamp = data->timeStamp;

	for (auto point : data->points)
	{
		if(point.z < MAX_Z_DISTANCE && point.z != 0)
		{
			pcl::PointXYZI p;
			p.x = point.x;
			p.y = point.y;
			p.z = point.z;
			p.intensity = point.grayValue;
			cloud->push_back(p);
		}
	}
	return cloud;
}

class cameraListener : public royale::IDepthDataListener
{
public:
	bool hasData;
	royale::DepthData data;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	cameraListener() : hasData(false)
	{
		viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("data viewer"));
		viewer->addCoordinateSystem(0.1);
		viewer->initCameraParameters();
	}

	void writePLY(const std::string &filename, royale::DepthData *data)
	{
		// For an explanation of the PLY file format please have a look at
		// https://en.wikipedia.org/wiki/PLY_(file_format)

		std::ofstream outputFile;
		std::stringstream stringStream;

		outputFile.open(filename, std::ofstream::out);

		if (outputFile.fail())
		{
			std::cerr << "Outputfile " << filename << " could not be opened!" << std::endl;
			return;
		}
		else
		{
			// if the file was opened successfully write the PLY header
			stringStream << "ply" << std::endl;
			stringStream << "format ascii 1.0" << std::endl;
			stringStream << "comment Generated by pico_recorder" << std::endl;
			stringStream << "element vertex " << data->points.size() << std::endl;
			stringStream << "property float x" << std::endl;
			stringStream << "property float y" << std::endl;
			stringStream << "property float z" << std::endl;
			stringStream << "element face 0" << std::endl;
			stringStream << "property list uchar int vertex_index" << std::endl;
			stringStream << "end_header" << std::endl;

			// output XYZ coordinates into one line
			for (size_t i = 0; i < data->points.size(); ++i)
			{
				stringStream << data->points.at(i).x << " " << data->points.at(i).y << " " << data->points.at(i).z << std::endl;
			}
			// output stringstream to file and close it
			outputFile << stringStream.str();
			outputFile.close();
		}
	}
    void writePLY(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr data)
	{
		// For an explanation of the PLY file format please have a look at
		// https://en.wikipedia.org/wiki/PLY_(file_format)

		std::ofstream outputFile;
		std::stringstream stringStream;

		outputFile.open(filename, std::ofstream::out);

		if (outputFile.fail())
		{
			std::cerr << "Outputfile " << filename << " could not be opened!" << std::endl;
			return;
		}
		else
		{
			// if the file was opened successfully write the PLY header
			stringStream << "ply" << std::endl;
			stringStream << "format ascii 1.0" << std::endl;
			stringStream << "comment Generated by pico_recorder" << std::endl;
			stringStream << "element vertex " << data->points.size() << std::endl;
			stringStream << "property float x" << std::endl;
			stringStream << "property float y" << std::endl;
			stringStream << "property float z" << std::endl;
			stringStream << "element face 0" << std::endl;
			stringStream << "property list uchar int vertex_index" << std::endl;
			stringStream << "end_header" << std::endl;
			// output XYZ coordinates into one line
			for (size_t i = 0; i < data->points.size(); ++i)
			{
				stringStream << data->points.at(i).x << " " << data->points.at(i).y << " " << data->points.at(i).z << std::endl;
			}
			// output stringstream to file and close it
			outputFile << stringStream.str();
			outputFile.close();
		}
	}

	void onNewData(const royale::DepthData *data) override
	{
		picoMut.lock();
		this->data = *data;
		hasData = true;

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = DepthDataToPCLI(&this->data);
		//if(cloud->size() == 0)
            	//	return;
		if (!viewer->updatePointCloud<pcl::PointXYZI>(cloud, "sample cloud"))
			viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
		picoMut.unlock();
	}

	float calcRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
	{
        	if(cloud_in->size() == 0)
            		return -1;
		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		pcl::ExtractIndices<PointT> extract;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

		pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);

		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud_in);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);

		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setOptimizeCoefficients(true);
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0.05, 0.3);
		seg.setInputCloud(cloud_in);
		seg.setInputNormals(cloud_normals);

		//initial guess for coeffiecients, 0 position, axis in Z direction. don't guess for radius
		coefficients_cylinder->values.push_back(0);
		coefficients_cylinder->values.push_back(0);
		coefficients_cylinder->values.push_back(0);
		coefficients_cylinder->values.push_back(0);
		coefficients_cylinder->values.push_back(0);
		coefficients_cylinder->values.push_back(1.0);

		// SACMODEL_CYLINDER - used to determine cylinder models. The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
		if(coefficients_cylinder->values.size() != 7)
		{
			std::cout << "coefficients_cylinder->values.size() != 7 is true, failed to segment? \n";
			return -1;
		}

	#if VISUALIZE==1
		extract.setInputCloud(cloud_in);
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		extract.filter(*cropped_cloud);

		//for (int i = 0; i < cropped_cloud->size(); i++)
		//	cropped_cloud->at(i).g = 255;

		//pcl::visualization::PCLVisualizer viewer("data viewer");

		//pcl::visualization::PointCloudColorHandlerRGBField<PointT> cloud_rgb(cloud);
		//viewer->addPointCloud<PointT>(cloud, cloud_rgb, "sample cloud");
		//viewer.addPointCloud<PointT>(cloud, "sample cloud");
		//pcl::visualization::PointCloudColorHandlerRGBField<PointT> cropped_cloud_rgb(cropped_cloud);
		//if(!viewer->updatePointCloud<PointT>(cropped_cloud, cropped_cloud_rgb, "cropped cloud"))
		//{
		//	viewer->addPointCloud<PointT>(cropped_cloud, cropped_cloud_rgb, "cropped cloud");
		//}
		//viewer.addCoordinateSystem(1.0);
		//viewer.initCameraParameters();
		//coefficients_cylinder->values[0] = 0;
		//coefficients_cylinder->values[1] = 0;
		//coefficients_cylinder->values[2] = 0;
		//if (coefficients_cylinder->values[5] < 0)
		//	coefficients_cylinder->values[5] *= -1;
		float b = -coefficients_cylinder->values[2]/coefficients_cylinder->values[5];
		for(int i = 0; i<3; i++)
        {
            coefficients_cylinder->values[i] = coefficients_cylinder->values[i] + b*coefficients_cylinder->values[3+i];
            if(coefficients_cylinder->values[5] < 0)
                coefficients_cylinder->values[3+i] *= -1;
        }
		viewer->removeShape("cylinder");
		viewer->addCylinder(*coefficients_cylinder, "cylinder");
		/*
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		*/
	#endif

		return abs(coefficients_cylinder->values[6]);

	}
};


//Corrected sample standard deviation
std::tuple<float, float> std_dev(std::vector<float> data)
{
    if(data.size() < 2)
        return std::tuple<float, float>(0,0);
	float mean = 0.0;
	for (int i = 0; i < data.size(); i++)
		mean += data[i] / (float)data.size();

	float sumMmeanSq = 0.0;
	for (int i = 0; i < data.size(); i++)
		sumMmeanSq += pow(data[i] - mean, 2);

	float std_dev = sqrt((1.0 / (float)(data.size() - 1))*sumMmeanSq);
	return std::tuple<float, float>(mean, std_dev);
}

int main(int argc, char *argv[])
{
	std::unique_ptr<cameraListener> listener;
	std::shared_ptr<royale::ICameraDevice> cameraDevice;
	auto result = setupCamera(cameraDevice, USECASE_SETTING, EXPOSURE_SETTING);
	if(result != 0)
	{
		std::cout << "failed to setup camera" << "\n";
		return result;
	}
	listener.reset(new cameraListener());
	if (cameraDevice->registerDataListener(listener.get()) != royale::CameraStatus::SUCCESS)
	{
		std::cerr << "Error registering data listener" << std::endl;
		return 1;
	}
    // start capture mode
    if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
    {
        std::cerr << "Error starting the capturing" << std::endl;
        return 1;
    }

	#define NUM_DATA_POINTS 50
	std::list<float> radii;
	plt::plot();
	plt::xlabel("#measurement");
	plt::ylabel("radius in mm");
    while (true)
    {
        picoMut.lock();
        if(listener->hasData)
        {
            listener->hasData = false;
            //auto data = listener->cloud;
			/*
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = listener->cloud;
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::copyPointCloud(*cloud, *cloud_copy); 
			*/
			listener->viewer->spinOnce(10);
			auto cloud_copy = DepthDataToPCLI(&listener->data);
			picoMut.unlock();
			std::cout << "copyed cloud\n";
            float r = listener->calcRadius(cloud_copy);
			radii.push_back(r);
			if(radii.size()> NUM_DATA_POINTS && radii.size()> 0)
				radii.pop_front();
			std::vector<float> data;
			for(auto it = radii.begin(); it != radii.end(); it++)
				data.push_back(*it);

			plt::clf();
			plt::plot(data);

			while(data.size() > 25)
				data.pop_back();
			auto statistics = std_dev(data);

			auto mean = std::get<0>(statistics);
			auto stddev = std::get<1>(statistics);
			std::vector<float> meanY = {mean, mean};
			std::vector<float> meanX = {NUM_DATA_POINTS / 2, NUM_DATA_POINTS};
			plt::plot(meanX, meanY);

			std::string title = "Mean: " + std::to_string(mean) + ", std_dev: " + std::to_string(stddev);

			plt::title(title);
			plt::xlim(0,50);
			plt::ylim(0.04, 0.3);
			plt::pause(0.00000001);

			std::cout << "got radius: " << r << "\n";
        }
        else
        {
            picoMut.unlock();
            boost::this_thread::sleep(boost::posix_time::microseconds(1000));
        }
    }

	if (cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
	{
		std::cerr << "Error stopping the capturing" << std::endl;
		return 1;
	}
	return(0);
}
