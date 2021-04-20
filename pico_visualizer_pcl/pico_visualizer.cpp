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


typedef pcl::PointXYZRGB PointT;

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

int main(int argc, char **argv)
{
	// Windows requires that the application allocate these, not the DLL.  We expect typical
	// Royale applications to be using a GUI toolkit such as Qt, which has its own equivalent of this
	// PlatformResources class (automatically set up by the toolkit).
	sample_utils::PlatformResources resources;

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
