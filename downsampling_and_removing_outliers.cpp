#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <time.h>

typedef pcl::PointXYZRGB PointT;

int  main1(int argc, char** argv)
{
	clock_t start, finish;
	double totaltime;


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	std::string filename;
	std::string save_filename;
	std::string init_prefix = "original1/face";
	std::string post_prefix = "processed1/processed_face";
	std::string suffix = ".pcd";
	

	for (int i = 2; i <= 6; i++)
	{
		start = clock();
		filename = init_prefix + std::to_string(i) + suffix;
		if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1)  //open pcd file
		{
			PCL_ERROR("Couldn't read the pcd file\n");
			return(-1);
		}
		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		cout << "load pcd data : " << totaltime << "seconds!" << endl;

		// Downsampling
		pcl::VoxelGrid<PointT> voxel_grid;
		voxel_grid.setLeafSize(2, 2, 2);
		voxel_grid.setInputCloud(cloud);
		pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);
		voxel_grid.filter(*cloud_down);
		std::cout << "\ndown size the point cloud from " << cloud->size() << "to" << cloud_down->size() << endl;

		// Remove outliers
		pcl::PointCloud<PointT>::Ptr cloud_inliner(new pcl::PointCloud<PointT>);
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(cloud_down);
		sor.setMeanK(50);
		sor.setStddevMulThresh(0.5);
		sor.filter(*cloud_inliner);
		save_filename = post_prefix + std::to_string(i) + suffix;
		pcl::io::savePCDFileBinaryCompressed(save_filename, *cloud_inliner);
	}

	return (0);
}