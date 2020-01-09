#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <vector> 
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp> 
#include <OpenNI.h> 

typedef unsigned char uint8_t;
using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

void CheckOpenNIError(Status result, string status)
{
	if (result != STATUS_OK)
		cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}

int main(int argc, char **argv)
{
	Status result = STATUS_OK;
	int i, j;
	float x = 0.0, y = 0.0, z = 0.0, xx = 0.0;
	IplImage *test2;

	//point cloud 
	PointCloud<PointXYZRGB> cloud;

	//opencv image
	Mat cvBGRImg;
	Mat cvDepthImg;

	//OpenNI2 image  
	VideoFrameRef oniDepthImg;
	VideoFrameRef oniColorImg;

	namedWindow("depth");
	namedWindow("image");

	char key = 0;

	// initialize OpenNI  
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	// open device    
	Device device;
	result = device.open(openni::ANY_DEVICE);
	CheckOpenNIError(result, "open device");


	// create depth stream   
	VideoStream oniDepthStream;
	result = oniDepthStream.create(device, openni::SENSOR_DEPTH);
	CheckOpenNIError(result, "create depth stream");

	// set depth video mode  
	VideoMode modeDepth;
	modeDepth.setResolution(640, 480);
	modeDepth.setFps(30);
	modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode(modeDepth);
	// start depth stream  
	result = oniDepthStream.start();
	CheckOpenNIError(result, "start depth stream");

	// create color stream  
	VideoStream oniColorStream;
	result = oniColorStream.create(device, openni::SENSOR_COLOR);
	CheckOpenNIError(result, "create color stream");
	// set color video mode  
	VideoMode modeColor;
	modeColor.setResolution(640, 480);
	modeColor.setFps(30);
	modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
	oniColorStream.setVideoMode(modeColor);
	// start color stream  
	result = oniColorStream.start();
	CheckOpenNIError(result, "start color stream");

	while (true)
	{
		// read frame  
		if (oniColorStream.readFrame(&oniColorImg) == STATUS_OK)
		{
			// convert data into OpenCV type  
			Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());
			cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
			imshow("image", cvBGRImg);
		}

		if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK)
		{
			Mat cvRawImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
			cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
			imshow("depth", cvDepthImg);
		}
		// quit
		if (cv::waitKey(1) == 'q')
			break;
		// capture  depth and color data   
		if (cv::waitKey(1) == 'c')
		{
			//get data
			DepthPixel *pDepth = (DepthPixel*)oniDepthImg.getData();
			//create point cloud
			cloud.width = oniDepthImg.getWidth();
			cloud.height = oniDepthImg.getHeight();
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);
			test2 = &IplImage(cvBGRImg);

			for (i = 0; i<oniDepthImg.getHeight(); i++)
			{
				for (j = 0; j<oniDepthImg.getWidth(); j++)
				{
					float k = i;
					float m = j;
					xx = pDepth[i*oniDepthImg.getWidth() + j];
					CoordinateConverter::convertDepthToWorld(oniDepthStream, m, k, xx, &x, &y, &z);
					cloud[i*cloud.width + j].x = x;
					cloud[i*cloud.width + j].y = y;
					cloud[i*cloud.width + j].z = xx;
					cloud[i*cloud.width + j].b = (uint8_t)test2->imageData[i*test2->widthStep + j * 3 + 0];
					cloud[i*cloud.width + j].g = (uint8_t)test2->imageData[i*test2->widthStep + j * 3 + 1];
					cloud[i*cloud.width + j].r = (uint8_t)test2->imageData[i*test2->widthStep + j * 3 + 2];
				}
			}

			pcl::io::savePCDFileBinaryCompressed("face.pcd", cloud);
			cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << endl;
			imwrite("face_color.jpg", cvBGRImg);
			imwrite("face_depth.jpg", cvDepthImg);
		}
	}
}