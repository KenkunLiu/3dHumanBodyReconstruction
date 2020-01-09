#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("matched_pcd.pcd", *cloud) == -1) //  open pcd file
	{
		PCL_ERROR("Couldn't read pcd file \n");
		return(-1);
	}
	std::cout << "Loaded " << cloud->width*cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

	pcl::visualization::CloudViewer viewer("My First Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{

	}
}