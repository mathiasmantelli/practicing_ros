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
    // cv::imshow("Display window", img);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        cv::imwrite("projected_pointcloud.png", img);
    }
    return 1;
}

std::tuple<cv::Mat, cv::Mat> defining_projection_matrix(){
    cv::Mat intrinsics_mat = cv::Mat::zeros(3,3, CV_32FC1);
    //(3,3, cv::DataType<double>::type);
    float focal_len_m, pixel_dim_m;
    int cam_size_u_px, cam_size_v_px;
    focal_len_m = 0.00605f;
    pixel_dim_m = 0.00000375f;
    cam_size_u_px = 1280;
    cam_size_v_px = 960;
    intrinsics_mat.at<float>(0,0) = focal_len_m; 
    intrinsics_mat.at<float>(1,1) = focal_len_m;
    intrinsics_mat.at<float>(2,2) = 1.0;
    intrinsics_mat.at<float>(0,2) = cam_size_u_px/2.0;
    intrinsics_mat.at<float>(1,2) = cam_size_v_px/2.0;

    float cam_x_m, cam_y_m, cam_z_m, cam_roll_rad, cam_pitch_rad, cam_yaw_rad;
    cam_x_m = 1.95;
    cam_y_m = 0.0;
    cam_z_m = 1.29;
    cam_roll_rad = -0.012;
    cam_pitch_rad = 0.02;
    cam_yaw_rad = 0.0;
    cv::Mat extrinsicts_mat = cv::Mat::zeros(3,4, CV_32FC1);
    extrinsicts_mat.at<float>(0,0) = cam_roll_rad;
    extrinsicts_mat.at<float>(1,1) = cam_pitch_rad;
    extrinsicts_mat.at<float>(2,2) = cam_yaw_rad;
    extrinsicts_mat.at<float>(0,3) = cam_x_m;
    extrinsicts_mat.at<float>(1,3) = cam_y_m;
    extrinsicts_mat.at<float>(2,3) = cam_z_m;

    return std::make_tuple(intrinsics_mat, extrinsicts_mat);
}

int read_pointcloud_csv(const std::string pointcloud_path, std::vector<lidar_point>& pointcloud, float max_intensity){
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
        lidar_point new_point; 
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
            if(max_intensity < new_point.intensity)
                max_intensity = new_point.intensity;
            pointcloud.push_back(new_point);
            //std::cout <<"TESTing: " << multiple_rows[0][i] << " " << multiple_rows[1][i] << " " << multiple_rows[2][i] << " " << multiple_rows[3][i] << std::endl;
        }   
        std::cout << "MAX INTENSITY : " << max_intensity << " | Average: " << sum/multiple_rows_values[0].size() << std::endl;     
        float variance = 0; 
        float average = sum/multiple_rows_values[0].size();
        for(int i = 0; i < multiple_rows_values[0].size(); i++){
            variance += std::pow((multiple_rows_values[3][i] - average),2);
        }
        float std_dev = std::sqrt(variance/multiple_rows_values[0].size());
        std::cout << "MAX INTENSITY : " << max_intensity << " | Average: " << average << " | Std Dev: " << std_dev << std::endl;     
        return 1; 
    }else{
        return -1; 
    }
}

cv::Mat pointcloud2image(const cv::Mat intrinsics_mat,const  cv::Mat extrinsicts_mat,const  std::vector<lidar_point> pointcloud,const  cv::Mat img){
    cv::Mat pointcloud_projected;
    img.copyTo(pointcloud_projected);
    cv::Mat point_3d = cv::Mat::ones(4,1, CV_32FC1);
    cv::Mat result, result2;
    point_3d.at<float>(0,0) = pointcloud[0].x;
    point_3d.at<float>(1,0) = pointcloud[0].y;
    point_3d.at<float>(2,0) = pointcloud[0].z;    
    std::cout << "POINT 3D: [" << point_3d.rows << ", " << point_3d.cols << "]" << std::endl;
    std::cout << "Point 3D[" << point_3d.at<float>(0,0) << ", "  << point_3d.at<float>(1,0) << ", " << point_3d.at<float>(2,0) << "]" << std::endl;    
    std::cout << "Intrinsics: [" << intrinsics_mat.rows << ", " << intrinsics_mat.cols << "] Extrinsic: ["  << extrinsicts_mat.rows << ", " << extrinsicts_mat.cols << "] Point3D: ["   << point_3d.rows << ", " << point_3d.cols << "]" << std::endl;
    result = extrinsicts_mat * point_3d;
    std::cout << "EXTRINSICTS RESULT: [" << result.rows << ", " << result.cols << "]" << std::endl;
    std::cout << "result[" << result.at<float>(0,0) << ", "  << result.at<float>(1,0) << ", " << result.at<float>(2,0) << "]" << std::endl;    
    result = intrinsics_mat * extrinsicts_mat * point_3d;
    std::cout << "INTRINSICTS RESULT: [" << result.rows << ", " << result.cols << "]" << std::endl;
    std::cout << "result[" << result.at<float>(0,0) << ", "  << result.at<float>(1,0) << ", " << result.at<float>(2,0) << "]" << std::endl;    
    cv::circle(pointcloud_projected, cv::Point(int(result.at<float>(0,0)), int(result.at<float>(0,1))), 5, cv::Scalar(255, 0,0), -1);
    std::cout <<" ----------------------------- " << std::endl; //
    int index = 5000;
    point_3d.at<float>(0,0) = pointcloud[index].x;
    point_3d.at<float>(1,0) = pointcloud[index].y;
    point_3d.at<float>(2,0) = pointcloud[index].z;    
    std::cout << "POINT 3D: [" << point_3d.rows << ", " << point_3d.cols << "]" << std::endl;
    std::cout << "Point 3D[" << point_3d.at<float>(0,0) << ", "  << point_3d.at<float>(1,0) << ", " << point_3d.at<float>(2,0) << "]" << std::endl;    
    std::cout << "Intrinsics: [" << intrinsics_mat.rows << ", " << intrinsics_mat.cols << "] Extrinsic: ["  << extrinsicts_mat.rows << ", " << extrinsicts_mat.cols << "] Point3D: ["   << point_3d.rows << ", " << point_3d.cols << "]" << std::endl;
    result = extrinsicts_mat * point_3d;
    std::cout << "EXTRINSICTS RESULT: [" << result.rows << ", " << result.cols << "]" << std::endl;
    std::cout << "result[" << result.at<float>(0,0) << ", "  << result.at<float>(1,0) << ", " << result.at<float>(2,0) << "]" << std::endl;    
    result2 = intrinsics_mat * extrinsicts_mat * point_3d;
    std::cout << "INTRINSICTS RESULT: [" << result2.rows << ", " << result2.cols << "]" << std::endl;
    std::cout << "result[" << result2.at<float>(0,0) << ", "  << result2.at<float>(1,0) << ", " << result2.at<float>(2,0) << "]" << std::endl;    
    cv::circle(pointcloud_projected, cv::Point(int(result.at<float>(0,0)), int(result2.at<float>(0,1))), 5, cv::Scalar(255, 0,0), -1);


    
    // for(int i = 0; i < pointcloud.size(); i++){
    //     point_3d.at<float>(0,0) = pointcloud[i].x;
    //     point_3d.at<float>(1,0) = pointcloud[i].y;
    //     point_3d.at<float>(2,0) = pointcloud[i].z;

    //     result = extrinsicts_mat * point_3d; 
    //     result = intrinsics_mat * result; 
    //     if(result.at<float>(0,0) > 0 && result.at<float>(0,0) < 1280 && result.at<float>(0,1) > 0 && result.at<float>(0,1) < 960)
    //         cv::circle(pointcloud_projected, cv::Point(int(result.at<float>(0,0)), int(result.at<float>(0,1))), 5, cv::Scalar(255, 0,0), -1);

    // }
    
    return pointcloud_projected;
}

cv::Mat projectPoints(const std::vector<lidar_point> pointcloud, const cv::Mat img, float max_intensity){
    cv::Mat pointcloud_projected;
    img.copyTo(pointcloud_projected);
    std::vector<cv::Point3f> points; 
    for(auto my_point : pointcloud){
        points.emplace_back(my_point.x, my_point.y, my_point.z);
    }

    cv::Mat intrinsicMat(3,3, cv::DataType<double>::type);
    intrinsicMat.at<double>(0,0) = 0.00605/0.00000375;
    intrinsicMat.at<double>(1,0) = 0;
    intrinsicMat.at<double>(2,0) = 0;

    intrinsicMat.at<double>(0,1) = 0;
    intrinsicMat.at<double>(1,1) = 0.00605/0.00000375;
    intrinsicMat.at<double>(2,1) = 0;

    intrinsicMat.at<double>(0,2) = 1280/2.0;
    intrinsicMat.at<double>(1,2) = 960/2.0;
    intrinsicMat.at<double>(2,2) = 1;

    cv::Mat R(3, 3, cv::DataType<double>::type);
    R.at<double>(0,0) = -0.52;
    R.at<double>(1,0) = 0;
    R.at<double>(2,0) = 0;

    R.at<double>(0,1) = 0;
    R.at<double>(1,1) = 0.02;
    R.at<double>(2,1) = 0;

    R.at<double>(0,2) = 0;
    R.at<double>(1,2) = 0;
    R.at<double>(2,2) = 0;


    cv::Mat Rvec(3, 1, cv::DataType<double>::type);
    cv::Rodrigues(R, Rvec);

    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    rvec.at<double>(0) = -0.012;
    rvec.at<double>(1) = 0.02;
    rvec.at<double>(2) = 0;


    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    tvec.at<double>(0) = 1.95;
    tvec.at<double>(1) = 0.0;
    tvec.at<double>(2) = 1.29;    

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;
    distCoeffs.at<double>(4) = 0;

    std::cout << "Intrisic matrix: " << intrinsicMat << std::endl << std::endl;
    std::cout << "Rotation vector: " << Rvec << std::endl << std::endl;
    std::cout << "Translation vector: " << tvec << std::endl << std::endl;
    std::cout << "Distortion coef: " << distCoeffs << std::endl << std::endl;

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points, Rvec, tvec, intrinsicMat, distCoeffs, projectedPoints);
        
    for(int index = 0; index < projectedPoints.size() - 1; index++){
        if(projectedPoints.at(index).x > 0 && projectedPoints.at(index).x < 1280 && 
           projectedPoints.at(index).y > 0 && projectedPoints.at(index).y < 960)
            cv::circle(pointcloud_projected, cv::Point(int(projectedPoints.at(index).x), int(projectedPoints.at(index).y)), 5, cv::Scalar(0, 0,255*(1 - (pointcloud[index].intensity - 0.00802395)/0.00547971)), -1);        //
    }
    return pointcloud_projected;
}

int main(int argc, char **argv){
    /*Defining the projection matrix (first the intrinsic and later extrinsic matrices)*/
    cv::Mat intrinsics_mat, extrinsicts_mat;
    std::tie(intrinsics_mat, extrinsicts_mat) = defining_projection_matrix();

    /*Reading the point cloud from a CVS file*/
    float max_intensity = -1;
    std::string pointcloud_csv_file = "../../../coding_task/pointcloud.csv";
    std::vector<lidar_point> pointcloud;
    pointcloud.clear();
    int pointcloud_read_sucess = read_pointcloud_csv(pointcloud_csv_file, pointcloud, max_intensity);
    if(pointcloud_read_sucess){
        std::cout << "Amount of points in the point cloud: " << pointcloud.size() << std::endl;
    }else{
        std::cout << "Error while opening the CVS file at: " << pointcloud_csv_file.c_str() << std::endl;
    }

    /*Reading the camera image from a JPG file*/
    cv::Mat img;
    int image_read_success;
    std::string image_path = "../../../coding_task/cameraimage.jpeg";
    image_read_success = load_image(image_path, img);
    // if(image_read_success)
        // cv::imshow("Display window", img);

    /*Computing the projection*/
    // cv::Mat output_projection = pointcloud2image(intrinsics_mat, extrinsicts_mat, pointcloud, img);
    cv::Mat output_projection = projectPoints(pointcloud, img, max_intensity);
    cv::imshow("Copied", output_projection);
    int k = cv::waitKey(0); // Wait for a keystroke in the window




    std::cout << "Intrinsic matrix: " << std::endl;
    for(int i = 0; i < intrinsics_mat.rows; i++){
        for(int j = 0; j < intrinsics_mat.cols; j++){
            std::cout << intrinsics_mat.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Extrinsic matrix: " << std::endl;
    for(int i = 0; i < extrinsicts_mat.rows; i++){
        for(int j = 0; j < extrinsicts_mat.cols; j++){
            std::cout << extrinsicts_mat.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}