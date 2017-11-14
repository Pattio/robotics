#define CERES_FOUND 1
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

#include <iostream>

int main(int argc, char *argv[])
{
    // Images vector
    /*std::vector<cv::String> images = {
        "1.png", "2.png", "3.png", "4.png", "5.png", "6.png", "7.png", "8.png", "9.png",
        "10.png", "11.png", "12.png", "13.png", "14.png", "15.png", "16.png", "17.png",
        "18.png", "19.png", "20.png", "21.png", "22.png", "23.png", "24.png", "25.png",
        "26.png", "27.png", "28.png", "29.png", "30.png", "31.png", "32.png", "33.png",
        "34.png", "35.png", "36.png"
    };*/

    std::vector<cv::String> images = {
        "1.png", "2.png", "30.png", "29.png"
    };

    /*std::vector<cv::String> templeImages = {
        "temple0001.png", "temple0002.png", "temple0003.png", "temple0004.png", "temple0005.png"
    };*/

    // Instrinsics matrix from ros /camera info
    cv::Matx33d instrinsicsMatrix = cv::Matx33d(228.50368107873834, 0, 160.0, 0, 274.7477419454622, 100.0, 0, 0,  1);
    //cv::Matx33d instrinsicsTemple = cv::Matx33d(1520.4, 0, 302.32, 0, 1525.9, 246.87, 0, 0, 1);
    std::vector<cv::Mat> cameraRotations, cameraTranslations, points3D;

    // Perform reconstruction to get 3d points from 2d images
    cv::sfm::reconstruct(images, cameraRotations, cameraTranslations, instrinsicsMatrix, points3D, true);

    // Create 3d visualizer window
    cv::viz::Viz3d window("Visualizer");
    window.setWindowSize(cv::Size(1000, 1000));


    // Create points cloud and push 3D points into it
    std::vector<cv::Vec3f> pointCloud;
    for(int i = 0; i < points3D.size(); i++) {
        pointCloud.push_back(cv::Vec3f(points3D[i]));
    }

    // Add cameras
    std::vector<cv::Affine3d> cameras;
    for(int i = 0; i< cameraRotations.size(); i++) {
        cameras.push_back(cv::Affine3d(cameraRotations[i], cameraTranslations[i]));
    }

    // Add point cloud to window
    if(pointCloud.size() > 0) {
        std::cout << "Adding 3D point cloud" << std::endl;
        cv::viz::WCloud cloud3D(pointCloud, cv::viz::Color::red());
        window.showWidget("PointCloud", cloud3D);
    } else {
        std::cout << "No points created, can't create cloud" << std::endl;
        exit(-1);
    }

    // Add cameras to window
    if(cameras.size() > 0) {
        std::cout << "Adding cameras" << std::endl;
        window.showWidget("Cameras", cv::viz::WTrajectory(cameras, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::green()));
        window.showWidget("CamerasFrames", cv::viz::WTrajectoryFrustums(cameras, instrinsicsMatrix, 0.1, cv::viz::Color::yellow()));
        window.setViewerPose(cameras[0]);
    } else {
        std::cout << "No cameras detected, exiting" << std::endl;
        exit(-1);
    }

    window.spin();
    return 0;
}



