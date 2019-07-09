/*
 * concatenate Eigen matrices
 * by R. Falque
 * 03/07/2019
 */

#ifndef EIGEN_FIND_HPP
#define EIGEN_FIND_HPP

#include <iostream>
#include <Eigen/Core>



// https://stackoverflow.com/a/21496281/2562693
//template <typename T>
//inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> concatenate(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> in_1, 
//                                                                    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> in_2, 
//                                                                    int direction);

template <typename T>
inline Eigen::VectorXi find(Eigen::Matrix<T, Eigen::Dynamic, 1> X, T x){
    Eigen::VectorXi elements( (X.array() == x).count() );

    int counter =0;
    for (int i=0; i<X.rows(); i++)
        if (X(i) == x) {
            elements(counter) = i;
            counter ++;
        }
    return elements;
};

template <typename T>
inline Eigen::VectorXi find(Eigen::Matrix<T, 1, Eigen::Dynamic> X, T x){
    Eigen::VectorXi elements( (X.array() == x).count() );

    int counter =0;
    for (int i=0; i<X.rows(); i++)
        if (X(i) == x) {
            elements(counter) = i;
            counter ++;
        }
    return elements;
};


template <typename T>
inline Eigen::Matrix<T, 1, Eigen::Dynamic> boolean_selection(Eigen::Matrix<T, 1, Eigen::Dynamic> X, Eigen::VectorXi selection){
    
    std::vector <T> vector_handle;

    int counter =0;
    for (int i=0; i<X.rows(); i++)
        if (selection(i) == 1) 
            vector_handle.push_back(X(i));

    Eigen::Matrix<T, 1, Eigen::Dynamic> elements = Eigen::Map<Eigen::Matrix<T, 1, Eigen::Dynamic>, Eigen::Unaligned>(vector_handle.data(), vector_handle.size());

    return elements;
};

inline Eigen::VectorXd index_slice(Eigen::VectorXd X, Eigen::VectorXi selection){
    Eigen::VectorXd out;
    out.resize(selection.cols());

    for (int i=0; i<selection.cols(); i++)
        out(i) = X(selection(i));

    return out;
};

//https://stackoverflow.com/a/21068014/2562693
template <typename T>
inline void removeRow(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

template <typename T>
inline void removeCol(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

#endif
