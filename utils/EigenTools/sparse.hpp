/*
 * from https://github.com/libigl/libigl/blob/master/include/igl/list_to_matrix.cpp
 * by R. Falque
 * 03/07/2019
 */

#ifndef EIGEN_SPARSE_HPP
#define EIGEN_SPARSE_HPP

#include <iostream>
#include <Eigen/Core>
#include <vector>

namespace SparseDiagonalMatrix {
    inline Eigen::SparseMatrix<double> Constant(int size, double value) {
      Eigen::SparseMatrix<double> sparse_matrix(size, size);
      sparse_matrix.reserve(size);
      for (int i=0; i<size; i++)
        sparse_matrix.insert(i,i) = value;
      return sparse_matrix;
    };
}

#endif
