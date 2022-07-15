#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>
// #include "depthmap2pointcloud.h"

struct PointCloud
	{
		Eigen::VectorXd x;
		Eigen::VectorXd y;
		Eigen::VectorXd z;
	};

//namespace depthmap2pcloud
class depthmap2pcloud
{
	//void depthmap2pointcloud_node::init()
	public:
	depthmap2pcloud() // default constructor
	{
		
	}
	~depthmap2pcloud() // default destructor
	{
		
	}
	Eigen::MatrixXd opencv2eigen(cv::Mat depthmap)
	{
		Eigen::MatrixXd container;
		cv::cv2eigen(depthmap, container);
		
		return container;
	}
	
	//void aggregatePointClouds(Eigen::MatrixXd& cloud1, Eigen::MatrixXd& cloud2, Eigen::MatrixXd Combined)
	//{
	
	
	//}
	
	
	
	PointCloud convert2pointcloud(cv::Mat& depthImage)
	{
		// This is simply a conversion from spherical coordinates to cartesian coordinates
		unsigned long int width = depthImage.cols;
		unsigned long int height = depthImage.rows;
		
		PointCloud pcloud;
		
		float originx = 0;
		float originy = (float) height/2;
		
		float dx = 360.0 / (float) width; // this is the resolution in the x direction
		float dy = 90.0 / (float) height; // this is the resolution in the y direction
		
		Eigen::MatrixXd Yidx = Eigen::VectorXd::LinSpaced(height, 0.0, height - 1).replicate(1, width);
		std::cout<<originx<<"  "<<originy<<"  "<<dx<<"  "<<dy<<'\n';
		
		Eigen::MatrixXd Xidx = Eigen::VectorXd::LinSpaced(width, 0.0, width - 1).replicate(1, height);
		Xidx.transposeInPlace();
		
		Eigen::MatrixXd image = opencv2eigen(depthImage);
		image = image.array()/512.0;
		// cout << Xidx.rows << '  ' << Xidx.cols;
		
		Eigen::MatrixXd x, y, z; // to hold the cartesian coordinates
		
		x = image.array() * ((Xidx.array() - originx) * (dx * M_PI / 180.0)).cos() * ((Yidx.array() - originy) * (dy * M_PI / 180.0 )).cos();
		y = image.array() * ((Xidx.array() - originx) * (dx * M_PI / 180.0)).sin() * ((Yidx.array() - originy) * (dy * M_PI / 180.0 )).cos();
		z = -1.0 * image.array() * ((Yidx.array() - originy) * (dy * M_PI / 180.0)).sin();
		Eigen::VectorXd x_flat;
		x_flat = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
		Eigen::VectorXd y_flat;
		y_flat = Eigen::Map<const Eigen::VectorXd>(y.data(), y.size());
		Eigen::VectorXd z_flat;
		z_flat = Eigen::Map<const Eigen::VectorXd>(z.data(), z.size());
		// x.resize(width * height, 1); // DO NOT USE RESIZE
		// y.resize(width * height, 1);
		// z.resize(width * height, 1);
		pcloud.x = x_flat;
		pcloud.y = y_flat;
		pcloud.z = z_flat;
		
		return pcloud;
	}
};