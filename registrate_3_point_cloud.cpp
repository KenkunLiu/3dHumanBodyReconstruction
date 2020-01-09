#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int  main(int argc, char** argv)
{
	clock_t start, finish;
	double totaltime;


	pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr my_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr my_cloud2(new pcl::PointCloud<PointT>);

	start = clock();
	if (pcl::io::loadPCDFile<PointT>("face2.pcd", *cloud1) == -1) // open point cloud 1
	{
		PCL_ERROR("Couldn't read pcd file\n");
		return(-1);
	}
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "load pcd data : " << totaltime << "seconds!" << endl;

	start = clock();
	if (pcl::io::loadPCDFile<PointT>("face3.pcd", *cloud2) == -1)// open point cloud 2
	{
		PCL_ERROR("Couldn't read pcd file\n");
		return(-1);
	}
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\nload pcd data : " << totaltime << "seconds!" << endl;

	start = clock();
	if (pcl::io::loadPCDFile<PointT>("face4.pcd", *cloud3) == -1)// open point cloud 3
	{
		PCL_ERROR("Couldn't read pcd file\n");
		return(-1);
	}
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\nload pcd data : " << totaltime << "seconds!" << endl;

	// Downsampling
	pcl::VoxelGrid<PointT> voxel_grid1;
	voxel_grid1.setLeafSize(5, 5, 5);
	voxel_grid1.setInputCloud(cloud1);
	pcl::PointCloud<PointT>::Ptr cloud1_down(new pcl::PointCloud<PointT>);
	voxel_grid1.filter(*cloud1_down);
	std::cout << "\n down size the point cloud from " << cloud1->size() << "to" << cloud1_down->size() << endl;

	// Remove outliers
	pcl::PointCloud<PointT>::Ptr cloud1_inliner(new pcl::PointCloud<PointT>);
	pcl::StatisticalOutlierRemoval<PointT> sor1;
	sor1.setInputCloud(cloud1_down);
	sor1.setMeanK(50);
	sor1.setStddevMulThresh(0.5);
	sor1.filter(*cloud1_inliner);


	// Downsampling
	pcl::VoxelGrid<PointT> voxel_grid2;
	voxel_grid2.setLeafSize(5, 5, 5);
	voxel_grid2.setInputCloud(cloud2);
	pcl::PointCloud<PointT>::Ptr cloud2_down(new pcl::PointCloud<PointT>);
	voxel_grid2.filter(*cloud2_down);
	std::cout << "\n down size the point cloud from " << cloud2->size() << "to" << cloud2_down->size() << endl;

	// Remove outliers
	pcl::PointCloud<PointT>::Ptr cloud2_inliner(new pcl::PointCloud<PointT>);
	pcl::StatisticalOutlierRemoval<PointT> sor2;
	sor2.setInputCloud(cloud2_down);
	sor2.setMeanK(50);
	sor2.setStddevMulThresh(0.5);
	sor2.filter(*cloud2_inliner);

	// Downsampling
	pcl::VoxelGrid<PointT> voxel_grid3;
	voxel_grid3.setLeafSize(5, 5, 5);
	voxel_grid3.setInputCloud(cloud3);
	pcl::PointCloud<PointT>::Ptr cloud3_down(new pcl::PointCloud<PointT>);
	voxel_grid3.filter(*cloud3_down);
	std::cout << "\n down size the point cloud from " << cloud3->size() << "to" << cloud3_down->size() << endl;

	// Remove outliers
	pcl::PointCloud<PointT>::Ptr cloud3_inliner(new pcl::PointCloud<PointT>);
	pcl::StatisticalOutlierRemoval<PointT> sor3;
	sor3.setInputCloud(cloud3_down);
	sor3.setMeanK(50);
	sor3.setStddevMulThresh(0.5);
	sor3.filter(*cloud3_inliner);



	////call icp api
	start = clock();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputCloud(cloud1_inliner);
	icp.setInputTarget(cloud2_inliner);
	pcl::PointCloud<PointT> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\n first time call icp process : " << totaltime << "seconds!" << endl;

	// construct a registrated point cloud
	for (int i = 0; i< Final.points.size(); i++)
	{
		PointT basic_point;
		basic_point.x = Final.points[i].x;
		basic_point.y = Final.points[i].y;
		basic_point.z = Final.points[i].z;
		//basic_point.r = Final.points[i].r;
		//basic_point.g = Final.points[i].g;
		//basic_point.b = Final.points[i].b;

		my_cloud->points.push_back(basic_point);
	}

	//call icp api another time
	start = clock();
	icp.setInputCloud(cloud3_inliner);
	icp.setInputTarget(my_cloud);
	pcl::PointCloud<PointT> Final2;
	icp.align(Final2);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\n second time call icp process : " << totaltime << "seconds!" << endl;

	// construct final point cloud
	for (int i = 0; i< Final2.points.size(); i++)
	{
		PointT basic_point;
		basic_point.x = Final2.points[i].x;
		basic_point.y = Final2.points[i].y;
		basic_point.z = Final2.points[i].z;
		//basic_point.r = Final2.points[i].r;
		//basic_point.g = Final2.points[i].g;
		//basic_point.b = Final2.points[i].b;

		my_cloud2->points.push_back(basic_point);
	}
	pcl::io::savePCDFileBinaryCompressed("matched_3.pcd", *my_cloud2);
	std::cout << "\nThe size of the final point cloud is " << my_cloud2->size() << endl;
	pcl::visualization::CloudViewer viewer("My First Cloud Viewer");
	viewer.showCloud(my_cloud2);
	while (!viewer.wasStopped())
	{

	}
	return (0);
}