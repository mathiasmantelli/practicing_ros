#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

struct Lidar_Point{
    float x, y, z, intensity;
};

int LoadImage(const std::string image_path, cv::Mat& img){
    img = cv::imread(image_path, cv::IMREAD_COLOR);
    if(img.empty())
    {
        return -1;
    }
    return 1;
}

void DefineProjectionMatrix(cv::Mat& intrinsic_matrix, cv::Mat& rotation_matrix, cv::Mat& translation_matrix, cv::Mat& distortion_matrix){
    /*Creating the intrinsic matrix*/
    float focal_len_m, pixel_dim_m;
    int cam_size_u_px, cam_size_v_px;
    focal_len_m = 0.00605f;
    pixel_dim_m = 0.00000375f;
    cam_size_u_px = 1280;
    cam_size_v_px = 960;
    intrinsic_matrix.at<double>(0,0) = focal_len_m/pixel_dim_m;
    intrinsic_matrix.at<double>(1,0) = 0;
    intrinsic_matrix.at<double>(2,0) = 0;

    intrinsic_matrix.at<double>(0,1) = 0;
    intrinsic_matrix.at<double>(1,1) = focal_len_m/pixel_dim_m;
    intrinsic_matrix.at<double>(2,1) = 0;

    intrinsic_matrix.at<double>(0,2) = cam_size_u_px/2.0;
    intrinsic_matrix.at<double>(1,2) = cam_size_v_px/2.0;
    intrinsic_matrix.at<double>(2,2) = 1;

    /*Creating rotation Matrix*/
    float cam_roll_rad, cam_pitch_rad, cam_yaw_rad;
    cam_roll_rad = -0.012;
    cam_pitch_rad = 0.02;
    cam_yaw_rad = 0.0;
    cv::Mat R(3, 3, cv::DataType<double>::type);
    R.at<double>(0,0) = cam_roll_rad;
    R.at<double>(1,0) = 0;
    R.at<double>(2,0) = 0;

    R.at<double>(0,1) = 0;
    R.at<double>(1,1) = cam_pitch_rad;
    R.at<double>(2,1) = 0;

    R.at<double>(0,2) = 0;
    R.at<double>(1,2) = 0;
    R.at<double>(2,2) = cam_yaw_rad;

    cv::Rodrigues(R, rotation_matrix);

    /*Creating the translation matrix*/
    float cam_x_m, cam_y_m, cam_z_m;
    cam_x_m = 1.95;
    cam_y_m = 0.0;
    cam_z_m = 1.29;
    translation_matrix.at<double>(0) = cam_x_m;
    translation_matrix.at<double>(1) = cam_y_m;
    translation_matrix.at<double>(2) = cam_z_m;    

    /*Creating the distortion matrix*/
    distortion_matrix.at<double>(0) = 0;
    distortion_matrix.at<double>(1) = 0;
    distortion_matrix.at<double>(2) = 0;
    distortion_matrix.at<double>(3) = 0;
    distortion_matrix.at<double>(4) = 0;
}

int ReadPointcloudCSV(const std::string pointcloud_path, std::vector<Lidar_Point>& pointcloud, float& average, float& std_dev){
    std::string line, word, temp; 
    std::fstream pointcloud_file(pointcloud_path, std::ios::in);

    std::vector<float> row_values; 
    std::vector<std::vector<float>> multiple_rows_values;     
    if(pointcloud_file.is_open()){
        while(getline(pointcloud_file, line)){
            row_values.clear();
            std::stringstream s(line);
            while(getline(s, word, ',')){
                row_values.push_back(std::stof(word));
            }
            multiple_rows_values.push_back(row_values);
        }
        Lidar_Point new_point; 
        new_point.x = 0;
        new_point.y = 0;
        new_point.z = 0;
        new_point.intensity = 0;
        double sum = 0; 
        for(int i = 0; i < multiple_rows_values[0].size(); i++){
            new_point.x = multiple_rows_values[0][i];
            new_point.y = multiple_rows_values[1][i];
            new_point.z = multiple_rows_values[2][i];
            new_point.intensity = multiple_rows_values[3][i];
            sum += new_point.intensity;
            pointcloud.push_back(new_point);
        }   
        float variance = 0; 
        average = sum/multiple_rows_values[0].size();
        for(int i = 0; i < multiple_rows_values[0].size(); i++){
            variance += std::pow((multiple_rows_values[3][i] - average),2);
        }
        std_dev = std::sqrt(variance/multiple_rows_values[0].size());
        std::cout << "Average: " << average << " | Std Dev: " << std_dev << std::endl;     
        return 1; 
    }else{
        return -1; 
    }
}

cv::Mat ProjectPoints(const std::vector<Lidar_Point> pointcloud, const cv::Mat img, const cv::Mat intrinsic_matrix, 
                      const cv::Mat rotation_matrix, const cv::Mat translation_matrix, const cv::Mat distortion_matrix, const float average, const float std_dev){
    
    /*Preparing the 3D points*/
    cv::Mat pointcloud_projected;
    img.copyTo(pointcloud_projected);
    std::vector<cv::Point3f> points; 
    for(auto my_point : pointcloud)
        points.emplace_back(my_point.x, my_point.y, my_point.z);
    

    std::cout << "Intrisic matrix: " << intrinsic_matrix << std::endl << std::endl;
    std::cout << "Rotation vector: " << rotation_matrix << std::endl << std::endl;
    std::cout << "Translation vector: " << translation_matrix << std::endl << std::endl;
    std::cout << "Distortion coef: " << distortion_matrix << std::endl << std::endl;

    /*Projecting the 3D points into the 2D image*/
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points, rotation_matrix, translation_matrix, intrinsic_matrix, distortion_matrix, projectedPoints);
        
    /*Drawing the points*/
    for(int index = 0; index < projectedPoints.size() - 1; index++){
        if(projectedPoints.at(index).x > 0 && projectedPoints.at(index).x < 1280 && 
           projectedPoints.at(index).y > 0 && projectedPoints.at(index).y < 960)
            cv::circle(pointcloud_projected, cv::Point(int(projectedPoints.at(index).x), int(projectedPoints.at(index).y)), 5, cv::Scalar(0, 0,255*((pointcloud[index].intensity - average)/std_dev)), -1);      
    }

    return pointcloud_projected;
}

int main(int argc, char **argv){
    /*Defining the main matrices*/
    cv::Mat intrinsic_matrix(3,3, cv::DataType<double>::type);
    cv::Mat rotation_matrix(3, 1, cv::DataType<double>::type);
    cv::Mat translation_matrix(3, 1, cv::DataType<double>::type);
    cv::Mat distortion_matrix(5,1,cv::DataType<double>::type);
    DefineProjectionMatrix(intrinsic_matrix, rotation_matrix, translation_matrix, distortion_matrix);

    /*Reading the point cloud from a CVS file*/
    float average, std_dev;
    // std::string pointcloud_csv_file = "../../../coding_task/pointcloud.csv";
    std::string pointcloud_csv_file = "pointcloud.csv";
    std::vector<Lidar_Point> pointcloud;
    pointcloud.clear();
    int pointcloud_read_sucess = ReadPointcloudCSV(pointcloud_csv_file, pointcloud, average, std_dev);
    if(pointcloud_read_sucess){
        std::cout << "Amount of points in the point cloud: " << pointcloud.size() << std::endl;
    }else{
        std::cout << "Error while opening the CVS file at: " << pointcloud_csv_file.c_str() << std::endl;
    }

    /*Reading the camera image from a JPG file*/
    cv::Mat img;
    int image_read_success;
    // std::string image_path = "../../../coding_task/cameraimage.jpeg";
    std::string image_path = "cameraimage.jpeg";
    image_read_success = LoadImage(image_path, img);
    if(image_read_success){
        std::cout << "Image has been read" << std::endl;
    }else{
        std::cout << "Could not read the image: " << image_path << std::endl;
    }   
    /*Computing the projection*/
    cv::Mat output_projection = ProjectPoints(pointcloud, img, intrinsic_matrix, rotation_matrix, translation_matrix, distortion_matrix, average, std_dev);
    cv::imshow("Copied", output_projection);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        cv::imwrite("projected_pointcloud.png", output_projection);
    }

    return 0;
}
