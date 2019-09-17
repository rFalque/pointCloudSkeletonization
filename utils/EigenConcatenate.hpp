/*
 * concatenate Eigen matrices
 * by R. Falque
 * 03/07/2019
 */

#ifndef EIGEN_CONCATENATE_HPP
#define EIGEN_CONCATENATE_HPP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>




// https://stackoverflow.com/a/21496281/2562693
template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> concatenate(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> in_1, 
                                                                    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> in_2, 
                                                                    int direction)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> out;
    int cols, rows;
    if (in_1.rows() == 0 & in_1.cols() ==0) {
        rows = in_2.rows();
        cols = in_2.cols();
    } else if (in_2.rows() == 0 & in_2.cols() ==0) {
        rows = in_1.rows();
        cols = in_1.cols();
    } else if (direction == 1 & in_1.cols() == in_2.cols()) {
        rows = in_1.rows()+in_2.rows();
        cols = in_1.cols();
    } else if (direction == 2 & in_1.rows() == in_2.rows()) {
        rows = in_1.rows();
        cols = in_1.cols()+in_2.cols();
    }

    out.resize(rows, cols);
    out << in_1, in_2;
    
    return out;
};


// https://stackoverflow.com/a/50353398/2562693
template <typename T>
inline Eigen::SparseMatrix< T > concatenate(Eigen::SparseMatrix<T>in_1, Eigen::SparseMatrix<T>in_2, int direction)
{
    typedef typename Eigen::SparseMatrix<T>::InnerIterator SparseIterator;
    
    // first test if the input size are correct with respect to the direction:
    if (direction == 1)
        if (in_1.cols() != in_2.cols())
        {
            std::cout << "Error: wrong input size, the cols size do not match.\n";
            std::exit(0);
        }
    else if (direction == 2)
        if (in_1.rows() != in_2.rows())
        {
            std::cout << "Error: wrong input size, the rows size do not match.\n";
            std::exit(0);
        }
    else {
        std::cout << "Error: wrong direction (direction should be 1 or 2).\n";
        std::exit(0);
    }

    // start the concatenation process:
    Eigen::SparseMatrix<T> out;

    if (direction == 1) // vertical
        out.resize(in_1.rows() + in_2.rows(), in_1.cols());
    else // horizontal
        out.resize(in_1.rows(), in_1.cols() + in_2.cols());
    
    out.reserve(in_1.nonZeros() + in_2.nonZeros());

    std::vector< Eigen::Triplet<T, size_t> > triplets;
    triplets.reserve(in_1.nonZeros() + in_2.nonZeros());

    for (int k = 0; k < in_1.outerSize(); ++k) {
        for (SparseIterator it(in_1, k); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }

    for (int k = 0; k < in_2.outerSize(); ++k) {
        for (SparseIterator it(in_2, k); it; ++it) {
            if (direction == 1)
                triplets.emplace_back(in_1.rows() + it.row(), it.col(), it.value());
            else
                triplets.emplace_back(it.row(), in_1.cols() + it.col(), it.value());
        }
    }

    out.setFromTriplets(triplets.begin(), triplets.end());

    return out;
};

#endif
