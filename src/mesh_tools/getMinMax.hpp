/*
*   Greedy Search
*   by R. Falque
*   29/11/2018
*/

#ifndef GETMINMAX_HPP
#define GETMINMAX_HPP

#include <Eigen/Core>
#include <limits> 

inline void getMinMax(Eigen::MatrixXd in_cloud, Eigen::Vector3d & min_point, Eigen::Vector3d & max_point){
    min_point << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();
    max_point << -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity();

    for(int i = 0; i < in_cloud.rows(); i++){
        min_point(0) = std::min(min_point(0), in_cloud.row(i)(0));
        min_point(1) = std::min(min_point(1), in_cloud.row(i)(1));
        min_point(2) = std::min(min_point(2), in_cloud.row(i)(2));
	
        max_point(0) = std::max(max_point(0), in_cloud.row(i)(0));
        max_point(1) = std::max(max_point(1), in_cloud.row(i)(1));
        max_point(2) = std::max(max_point(2), in_cloud.row(i)(2));
    }
};

inline void getScale(Eigen::MatrixXd in_cloud, double & scale){
    Eigen::Vector3d min_point;
    Eigen::Vector3d max_point;

    getMinMax(in_cloud, min_point, max_point);

    scale = (max_point - min_point).norm();
};

#endif
