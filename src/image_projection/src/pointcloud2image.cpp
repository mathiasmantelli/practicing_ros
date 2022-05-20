#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

struct lidar_point{
    float x, y, z, intensity;
};

int load_image(const std::string image_path, cv::Mat& img){
    img = cv::imread(image_path, cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return -1;
    }
    cv::imshow("Display window", img);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        cv::imwrite("projected_pointcloud.png", img);
    }
    return 1;
}

std::vector<lidar_point> read_pointcloud_csv(const std::string pointcloud_path){
    std::vector<lidar_point> pointcloud;
    std::string line, word, temp; 
    std::fstream pointcloud_file(pointcloud_path, std::ios::in);
    int rollnum, roll, count = 0;
    rollnum = 50; 
    // std::vector<std::string> row; 
    // std::vector<std::vector<std::string>> multiple_rows; 
    std::vector<float> row_values; 
    std::vector<std::vector<float>> multiple_rows_values;     
    if(pointcloud_file.is_open()){
        while(getline(pointcloud_file, line)){
            //row.clear();
            row_values.clear();
            std::stringstream s(line);
            while(getline(s, word, ',')){
                // row.push_back(word);
                row_values.push_back(std::stof(word));
            }
            // multiple_rows.push_back(row);
            multiple_rows_values.push_back(row_values);
            // std::cout << "Size row: " << row.size() << std::endl;
            //std::cout << "Size multiple row: " << multiple_rows.size() << std::endl;
        }
        /*std::cout << "Size row: " << row.size() << std::endl;
        for(int i = 0; i < rollnum; i++){
            std::cout <<"TESTing: " << multiple_rows[0][i] << " " << multiple_rows[1][i] << " " << multiple_rows[2][i] << " " << multiple_rows[3][i] << std::endl;
        }
        std::cout << std::endl;*/
    }
}
int main(int argc, char **argv){
    cv::Mat img;
    cv::Mat intrinsics_mat = cv::Mat::zeros(3,3, CV_32FC1);
    intrinsics_mat.at<float>(0,0) = 0.00605f;
    intrinsics_mat.at<float>(1,1) = 0.00605f;
    intrinsics_mat.at<float>(0,2) = (1280.0f/2.0f);
    intrinsics_mat.at<float>(1,2) = (960.0f/2.0f);

    cv::Mat extrinsicts_mat = cv::Mat::zeros(3,4, CV_32FC1);
    extrinsicts_mat.at<float>(0,0) = -0.012f;
    extrinsicts_mat.at<float>(1,1) = 0.02f;
    extrinsicts_mat.at<float>(0,3) = 1.95f;
    extrinsicts_mat.at<float>(2,3) = 1.29f;

    for(int i = 0; i < intrinsics_mat.rows; i++){
        for(int j = 0; j < intrinsics_mat.cols; j++){
            std::cout << intrinsics_mat.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    for(int i = 0; i < extrinsicts_mat.rows; i++){
        for(int j = 0; j < extrinsicts_mat.cols; j++){
            std::cout << extrinsicts_mat.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }

    std::vector<lidar_point> test = read_pointcloud_csv("../../../coding_task/pointcloud.csv");

    int image_read_success;
    std::string image_path = "../../../coding_task/cameraimage.jpeg";
    image_read_success = load_image(image_path, img);
    if(image_read_success)
        cv::imshow("Display window", img);
    return 0;
}