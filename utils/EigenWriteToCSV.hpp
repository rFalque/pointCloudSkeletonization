/*
*   back up Eigen vector of matrix to CSV file
*   by R. Falque
*   30/07/2019
*/

#ifndef EIGEN_WRITE_TO_CSV_HPP
#define EIGEN_WRITE_TO_CSV_HPP

#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <string>

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

template <typename T>
inline bool EigenWriteToCSVfile(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix, std::string name)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
    file.close();

    return true;
};


inline bool EigenWriteToCSVfile(Eigen::VectorXi matrix, std::string name)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
    file.close();

    return true;
};

#endif
