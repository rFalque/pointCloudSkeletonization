/*
*   nanoflann wrapper
*   by R. Falque
*   21/11/2018
*/

#ifndef NANOFLANN_WRAPPER
#define NANOFLANN_WRAPPER

#include <iostream>
#include <memory>

#include "nanoflann.hpp"

class nanoflann_wrapper
{
public:
	nanoflann_wrapper(Eigen::MatrixXd& target)
	{
		// set up kdtree
		int leaf_size=10;
		int dimensionality=3;

		this->kd_tree_index = std::make_shared< nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd> >(dimensionality, target, leaf_size);
		this->kd_tree_index->index->buildIndex();

	}

	~nanoflann_wrapper(){
	}

	std::vector< int > return_k_closest_points(Eigen::Vector3d query_point, int k)
	{
		// Query point:
		std::vector<double> query_pt;
		for (int d=0; d<3; d++)
			query_pt.push_back( query_point(d) );

		// set wtf vectors
		std::vector<size_t> ret_indexes(k);
		std::vector<double> out_dists_sqr(k);
		nanoflann::KNNResultSet<double> resultSet(k);
		resultSet.init( &ret_indexes.at(0), &out_dists_sqr.at(0) );

		// knn search
		this->kd_tree_index->index->findNeighbors(resultSet, &query_pt.at(0), nanoflann::SearchParams(k));

		// pack result into std::vector<int>
		std::vector< int > indexes;
		for (int i = 0; i < k; i++)
			indexes.push_back( ret_indexes.at(i) );

		return indexes;
	}

private:
	std::shared_ptr < nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd > > kd_tree_index;


};

#endif
