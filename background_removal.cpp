#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 1);
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;
	// load pcd file
	reader.read("filename", *cloud);

	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// Threshold
	seg.setDistanceThreshold(25);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}


	// extract the background
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;

	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	writer.write<pcl::PointXYZRGB>("filename", *cloud_filtered, false);

	// viewer
	pcl::visualization::CloudViewer viewer("Filtered");
	viewer.showCloud(cloud_filtered);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	while (!viewer.wasStopped()) {

	}
	return (0);
}

