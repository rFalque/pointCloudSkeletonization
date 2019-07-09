/*
 * wrapper for delaunator: https://github.com/delfrrr/delaunator-cpp
 * by R. Falque
 * 27/06/2019
 */

#ifndef DELAUNAY_TRIANGULATION_HPP
#define DELAUNAY_TRIANGULATION_HPP

#include "delaunator.hpp"
#include <Eigen/Dense>
#include <vector>

inline Eigen::MatrixXi delaunay_triangulation_2d(Eigen::MatrixXd & points) {

std::cout << "tests 2 \n";
    if (points.cols() != 2) {
        std::cout << "the dimension of the input are incorrect" << std::endl;
    }

std::cout << "tests 3 \n";
    // std formating
    std::vector< double > std_points;
    for (int i=0; i<points.rows(); i++) {
        std_points.push_back(points(i, 0));
        std_points.push_back(points(i, 1));
    }

std::cout << "tests 4 \n";
    delaunator::Delaunator d(std_points);

std::cout << "tests 5 \n";
    // eigen formating
    std::cout << "d.triangles.size() " << d.triangles.size() << "\n";


    Eigen::MatrixXi triangles(int(d.triangles.size()/3), 3);
    std::cout << "triangles size : " << triangles.rows() << ", " << triangles.cols() << "\n";

    for(int i=0; i < d.triangles.size(); i+=3) {
        std::cout << "i = " << i << "\n";
        triangles(i/3, 0) = int(d.triangles[i]);
        triangles(i/3, 1) = int(d.triangles[i+1]);
        triangles(i/3, 2) = int(d.triangles[i+2]);
    }

std::cout << "tests 6 \n";
    return triangles;
};

#endif
