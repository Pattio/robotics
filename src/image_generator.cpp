#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
image_transport::Publisher image_pub_;

// Instrinsics matrix
cv::Matx33d K;

std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
std::vector<std::string> images = {"1.png", "2.png", "3.png", "4.png", "5.png", "6.png"};

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imwrite("test.jpg", cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
 }

 void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
 	K = cv::Matx33d(msg->K[0], 0, msg->K[2], 0, msg->K[4], msg->K[5], 0, 0,  1);
 }

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_generator");
	ros::NodeHandle nh;

	// Get instrinsics matrix from camera info
	// sensor_msgs::CameraInfo cameraInfo = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", ros::Duration(10)));
	// std::std::cout << "wow"  << std::std::endl;;
// 
	image_transport::ImageTransport it_(nh);
	// image_transport::Subscriber image_sub_ = it_.subscribe("/image", 1, imageCb);
	ros::Subscriber info_sub = nh.subscribe("/camera_info", 1, cameraInfoCallback);
	// image_pub_ = it_.advertise("/image_converter/output_video", 1);

	cv::namedWindow(OPENCV_WINDOW);
	while(ros::ok()) {
		if(K(0,0) != 0) break;
		ros::spinOnce();
	}


}