#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include <jsoncpp/json/json.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>

/**
 * @brief 读取json文件,并把数据转换成字符串
 * @param file_path json文件的路径
 * @return json文件内容的字符串
*/
inline std::string parseJsonFile(const std::string& file_path){

    // 读取json文件
    std::ifstream jsonFile(file_path);
    if (!jsonFile.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return "";
    }

    // 读取文件内容到字符串
    std::string jsonData((std::istreambuf_iterator<char>(jsonFile)),
                        std::istreambuf_iterator<char>());
    jsonFile.close();
    
    return jsonData;

}

/**
 * @brief 把json文件中的key,转换成eigen矩阵
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return Eigen矩阵
*/
inline Eigen::MatrixXd parseJsonEigenMatrix(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 获取 matrix 数组
    Json::Value matrixArray = root[key];
    
    // 确保 matrixArray 是数组
    if (!matrixArray.isArray()) {
        std::cerr << "Invalid eigen matrix data in JSON." << std::endl;
        return Eigen::MatrixXd::Zero(0,0);
    }

    // 获取matrixArray的行数
    int rows = matrixArray.size();
    // 获取matrixArray的列数
    int cols = matrixArray[0].size();

    Eigen::MatrixXd matrix(rows, cols);

    // 读取并处理矩阵数据
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            matrix(i, j) = matrixArray[i][j].asDouble();
        }
    }

    return matrix;
}

/**
 * @brief 把json文件中的key,转换成cv::Mat
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return cv::Mat矩阵
*/
inline cv::Mat parseJsonCVMatrix(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 获取 matrix 数组
    Json::Value matrixArray = root[key];
    
    // 确保 matrixArray 是数组
    if (!matrixArray.isArray()) {
        std::cerr << "Invalid cv matrix data in JSON." << std::endl;
        return cv::Mat::zeros(0,0,CV_64FC1);
    }

    // 获取matrixArray的行数
    int rows = matrixArray.size();
    // 获取matrixArray的列数
    int cols = matrixArray[0].size();

    cv::Mat matrix = cv::Mat::zeros(rows, cols, CV_64FC1);

    // 读取并处理矩阵数据
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            matrix.at<double>(i, j) = matrixArray[i][j].asDouble();
        }
    }

    return matrix;
}

/**
 * @brief 把json文件中的key,转换成double数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return double数据
*/
inline double parseJsonDouble(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 double 数据
    if (root.isMember(key) && root[key].isDouble()) {
        double value = root[key].asDouble();
        return value;
    } else {
        std::cout << "parse Json double WRONG!! " << key << std::endl;
        return 0;
    }
}

/**
 * @brief 把json文件中的key,转换成int数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return int数据
*/
inline int parseJsonInt(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 int 数据
    if (root.isMember(key) && root[key].isInt()) {
        int value = root[key].asInt();
        return value;
    } else {
        std::cout << "parse Json int WRONG!! " << key << std::endl;
        return 0;
    }
}

/**
 * @brief 把json文件中的key,转换成bool数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return bool数据
*/
inline bool parseJsonBool(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 int 数据
    if (root.isMember(key) && root[key].isBool()) {
        bool value = root[key].asBool();
        return value;
    } else {
        std::cout << "parse Json int WRONG!! " << key << std::endl;
        return 0;
    }
}

/**
 * @brief 把json文件中的key,转换成string数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return double数据
*/
inline std::string parseJsonString(const std::string& file_path, const std::string& key){
        
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 double 数据
    if (root.isMember(key) && root[key].isString()) {
        std::string value = root[key].asString();
        return value;
    } else {
        std::cout << "parse Json String WRONG!! " << key << std::endl;
        return "";
    }
}