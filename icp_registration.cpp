// This code could iteratively registrate as many point clouds as possible

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <time.h>

typedef pcl::PointXYZRGB PointT;

int  main(int argc, char** argv)
{
	clock_t start, finish;
	double totaltime;

	pcl::PointCloud<PointT>::Ptr cloud_next(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_model(new pcl::PointCloud<PointT>);

	std::string filename;
	std::string init_prefix = "processed1/processed_face";
	std::string suffix = ".pcd";

	start = clock();
	if (pcl::io::loadPCDFile<PointT>("processed1/processed_face2.pcd", *cloud_model) == -1)
	{
		PCL_ERROR("Couldn't read pcd file\n");
		return(-1);
	}
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "load pcd data : " << totaltime << "seconds!" << endl;

	for (int n = 2; n <= 4; n++)
	{
		start = clock();
		filename = init_prefix + std::to_string(n) + suffix;
		pcl::PointCloud<PointT>::Ptr cloud_next(new pcl::PointCloud<PointT>);
		if (pcl::io::loadPCDFile<PointT>(filename, *cloud_next) == -1)
		{
			PCL_ERROR("Couldn't read pcd file\n");
			return(-1);
		}
		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		cout << "\nload pcd data : " << totaltime << "seconds!" << endl;

		////call icp api
		start = clock();
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputCloud(cloud_next);
		icp.setInputTarget(cloud_model);
		pcl::PointCloud<PointT> *Final = new pcl::PointCloud<PointT>;
		icp.align(*Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;

		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		cout << "\n"<<n << "  time call icp process : " << totaltime << "seconds!" << endl;

		// construct current point
		for (int i = 0; i< Final->points.size(); i++)
		{
			PointT basic_point;
			basic_point.x = Final->points[i].x;
			basic_point.y = Final->points[i].y;
			basic_point.z = Final->points[i].z;
			basic_point.r = Final->points[i].r;
			basic_point.g = Final->points[i].g;
			basic_point.b = Final->points[i].b;
			cloud_model->points.push_back(basic_point);
		}
		delete Final;
	}

	// save the point cloud
	pcl::io::savePCDFileBinaryCompressed("matched_pcd.pcd", *cloud_model);
	pcl::visualization::CloudViewer viewer("My Cloud Viewer");
	viewer.showCloud(cloud_model);
	while (!viewer.wasStopped())
	{

	}
	return (0);
}