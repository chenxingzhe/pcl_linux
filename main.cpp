
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
using namespace openni;
static pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer");
int threshold_value1 = 125;
int threshold_value2 = 240;
char* trackbar_value1 = "H_low Value";
char* trackbar_value2 = "H_high Value";
void Threshold_Demo(int, void*)
{}
bool svalue=false;
Mat pick;
char key=0;
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	Mat pic(480,640,CV_64FC1);
	Mat pic1(480,640,CV_64FC1);
	
	if (!viewer.wasStopped())
	{
		if(svalue)
			return;
		
               
		
		for(int i=0;i<480;i++)
			for(int j=0;j<640;j++)
			{
				//cout<< cloud->points[i*640+j].x<<" "<<cloud->points[i*640+j].y<<" "<<cloud->points[i*640+j].z<<endl;
				pic.at<double>(i,j)=cloud->points[i*640+j].z;
				pic1.at<double>(i,j)=cloud->points[i*640+j].y;
			}
			double m1,m2;
			minMaxIdx(pic, &m1, &m2);
		       //cout<<m1<<" "<<m2<<endl;
			pic=pic-m1;
			pic.convertTo(pic,CV_8U,255/(m2-m1));
			minMaxIdx(pic1, &m1, &m2);
			pic1=pic1-m1;
			pic1.convertTo(pic1,CV_8U,255/(m2-m1));
			//flip(pic,pic,1);
			//flip(pic1,pic1,1);
			Mat pic2;
			inRange(pic1, threshold_value1, threshold_value2,pic1);
			//inRange(pic, 60, 240,pic1);
			//inRange(pic, 0, 60,pic2);
			//pic=pic1+pic2;


			cv::imshow("z",pic);
			cv::imshow("y",pic1);
			pic1=pic1/255;
			pick=pic1;
			
			key = cv::waitKey(20);		
			viewer.showCloud(cloud);
			svalue=true;
	}
}


void CheckOpenNIError(Status result, string status)
{
	if (result != STATUS_OK)
		cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}
string num2str(int i){
	stringstream s;
	s << i;
	return s.str();
}
int main(int argc, char** argv)
{
	//pcl test
	//static pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer");
	cv::namedWindow("BarValueThres");
	const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
	Eigen::Quaternionf ori1(0,0,0,0);
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
		boost::bind(&cloud_cb_,_1);
	interface->registerCallback(f);
	interface->start();

	//【1】
	// initialize OpenNI2
	while (key != 27&&!viewer.wasStopped())
	{
		// read frame
		if(svalue==false)
		{
			//Sleep(1);
			continue;
		}
		createTrackbar(trackbar_value1,
			"BarValueThres", &threshold_value1,
			255, Threshold_Demo);

		createTrackbar(trackbar_value2,
			"BarValueThres", &threshold_value2,
			255, Threshold_Demo);

		
			svalue=false;
			//imwrite((num2str(index++) + ".jpg").c_str(), cvDepthImg);
		//【6】
		key = cv::waitKey(20);
	}
	interface->stop();
	//cv destroy
	cv::destroyWindow("depth");
	cv::destroyWindow("image");
	cv::destroyWindow("fusion");

	//OpenNI2 destroy
	return 0;
}



