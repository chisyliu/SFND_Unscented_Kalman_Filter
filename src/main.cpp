/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include <fstream>

int main(int argc, char **argv)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition(x_pos - 26, 0, 15.0, x_pos + 25, 0, 0, 0, 0, 1);
	bool logNIS = false;

	Highway highway(viewer, logNIS);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec * sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000 / frame_per_sec);
		frame_count++;
		time_us = 1000000 * frame_count / frame_per_sec;
	}

	std::cout << "Program finished!" << std::endl;
	if (logNIS)
	{
		ofstream nis_lidar_log;
		ofstream nis_radar_log;
		nis_lidar_log.open("../results/nis_lidar.csv");
		nis_radar_log.open("../results/nis_radar.csv");
		for (int i = 0; i < highway.traffic[0].ukf.NIS_lidar_.size(); i++)
		{
			nis_lidar_log << highway.traffic[0].ukf.NIS_lidar_[i] << ","
						  << highway.traffic[1].ukf.NIS_lidar_[i] << ","
						  << highway.traffic[2].ukf.NIS_lidar_[i] << std::endl;
		}
		for (int i = 0; i < highway.traffic[0].ukf.NIS_radar_.size(); i++)
		{
			nis_radar_log << highway.traffic[0].ukf.NIS_radar_[i] << ","
						  << highway.traffic[1].ukf.NIS_radar_[i] << ","
						  << highway.traffic[2].ukf.NIS_radar_[i] << std::endl;
		}
		nis_lidar_log.close();
		nis_radar_log.close();
		std::cout << "NISs logged!" << std::endl;
	}
}