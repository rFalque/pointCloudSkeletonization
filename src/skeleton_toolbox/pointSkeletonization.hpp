#ifndef POINT_SKELETONIZATION_HPP
#define POINT_SKELETONIZATION_HPP

#include <algorithm>           // std::all_of
#include <Eigen/Dense>
#include <limits>
#include "../graph_lib/graphStructure.hpp"


#include "normalization.hpp"
#include "pointSkeletonizationOptions.hpp"

#include "point_ring.hpp"

#include "../mesh_tools/nanoflannWrapper.hpp"


class PointSkeletonization
{
private:
    point_skeletonization_options opts_;                                // Store all visualization infos

    Eigen::MatrixXd pointcloud_;                                        // pointcloud
	Eigen::MatrixXd contracted_pointcloud_;                             // contracted pointcloud
	
    Eigen::MatrixXi F_;                                                 // F: faces of the surface (for plots)

	
	std::vector< OneRing > one_ring_list_;


	Graph skeleton;

	inline bool normalization();
	inline Eigen::VectorXd one_ring_size(std::string distance_type);

public:

	// overloaded creations of the class (set verbose and load the data)
	PointSkeletonization(Eigen::MatrixXd V, Eigen::MatrixXi F)
	{
		pointcloud_ = V;
		F_ = F;
	}


	// destructor
	~PointSkeletonization()
	{
	}

	// initialisation of the private variables
	bool init()
	{
		normalization();

		// get the nearest neighbours
		nanoflann_wrapper knn_search(V);

		// get the one ring
		for (int i=0; i<V.rows(); i++) {
			// (1) search for k closest points and (2) remove the point itself
			std::vector < int > neighbours_id;
			neighbours_id = knn_search.return_k_closest_points(V.row(i), k_for_knn+1);
			neighbours_id.erase(neighbours_id.begin());

			// look for the one_ring neighbors
			OneRing one_ring(i, neighbours_id, V);
			one_ring_list_.push_back(one_ring);
		}

		return true;
	}

	bool laplacian_contraction() {
	    // initialization of: WH, WL, and L
		Eigen::VectorXd ms;
    	ms = one_ring_size("mean");
    	double initWL = 1.0/(5.0*ms.mean());

		Eigen::SparseMatrix<double> WL(V.rows(), V.rows());
		for (int i=0; i<V.rows(); i++)
			WL.insert(i,i) = initWL;

		Eigen::SparseMatrix<double> WH(V.rows(), V.rows());
		for (int i=0; i<V.rows(); i++)
			WH.insert(i,i) = 1;

		// get laplacian
	    Eigen::SparseMatrix<double> L = laplacian(pointcloud_, one_ring_list_);

		// first contraction

	    WhP = WH*V;
		WlL = WL*L;
		zeros = Eigen::MatrixXd::Zero(WhP.rows(), 3);

		A = concatenate(WlL, WH, 1);
		b = concatenate(zeros, WhP, 1);
		A2 = A.transpose() * A;
		b2 = A.transpose() * b;

		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
		solver.compute(A2);
		if(solver.info()!=Eigen::Success) {
			// decomposition failed
			std::cout << "Error: solver failed\n";
		}
		contracted_pointcloud_ = solver.solve(b2);

	}

	bool set_default_options()
	{
		opts_.verbose = false;
		opts_.nodes_ratio = 50.0;
		opts_.edges_ratio = 200.0;
		opts_.graph_res = 30.0;
		opts_.nodes_color = {1.0, 0.1, 0.1};
		opts_.edges_color = {0.1, 0.1, 0.1};
	}

};


inline Eigen::MatrixXd contracted_points solve_contraction(Eigen::SparseMatrix<double> WH, 
                                                           Eigen::SparseMatrix<double> WL, Eigen::SparseMatrix<double> L, Eigen::MatrixXd P)
{
	// contraction time :)
	Eigen::SparseMatrix<double> A, A2, WlL;
	Eigen::MatrixXd b, b2, WhP, zeros;

	WhP = WH*P;
	WlL = WL*L;
	zeros = Eigen::MatrixXd::Zero(WhP.rows(), 3);

	A = concatenate(WlL, WH, 1);
	b = concatenate(zeros, WhP, 1);
	A2 = A.transpose() * A;
	b2 = A.transpose() * b;
}

inline bool PointSkeletonization::normalization()
{
    // centre in zero
    pointcloud_.rowwise() -= (pointcloud_.colwise().minCoeff() + pointcloud_.colwise().maxCoeff()) / 2;

    // make the diagonal 1.6
    pointcloud_ *= 1.6/(pointcloud_.colwise().maxCoeff() - pointcloud_.colwise().minCoeff()).maxCoeff();

    return true;
};


// return for each point the mean distance to its neighbours
inline Eigen::VectorXd PointSkeletonization::one_ring_size(std::string distance_type)
{


    // vector to return
    Eigen::VectorXd out(pointcloud_.rows());

    for (int i=0; i<pointcloud_.rows(); i++) {
        // get the point:
        Eigen::Vector3d vertex;
        vertex = pointcloud_.row(i);

        // get the corresponding one ring structure
        std::vector<int> one_ring = one_ring_list_.at(i).get_one_ring();

        // store all the distances between the vertex and its neighbours
        Eigen::VectorXd temp(one_ring.size());
        for (int neighbour_it=0; neighbour_it<one_ring.size(); neighbour_it++) {
            Eigen::Vector3d neighbour;
            neighbour = cloud.row(one_ring.at(neighbour_it));
            temp(neighbour_it) = ( vertex - neighbour ).norm();
        }

        // get the mean of all the distance to the point
        if (distance_type=="min")
            out(i) = temp.minCoeff();
        else if (distance_type=="mean")
            out(i) = temp.mean();
        else if (distance_type=="max")
            out(i) = temp.maxCoeff();
    }

    return out;
};

/*
 * Definition of cotangent with respect to the dot and cross product 
 * see "The Geometry of the Dot and Cross Products" from Tevian Dray & Corinne A. Manogue
 * (there might be a better citation)
 * 
 * with v and w two vector and theta in between
 * |v x w| = |v|.|w|.sin(theta)
 *  v . w  = |v|.|w|.cos(theta)
 * cot(theta) = cos(theta) / sin(theta)
 *            = (v . w) / |v x w|
 */
inline double PointSkeletonization::cotan(Eigen::Vector3d v, Eigen::Vector3d w)
{ 
    return( ( v.dot(w) ) / ( (v.cross(w)).norm() ) ); 
};

inline Eigen::SparseMatrix<double> PointSkeletonization::laplacian(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list) {
    //Eigen::MatrixXd L = Eigen::MatrixXd::Zero(cloud.rows(), cloud.rows());

    Eigen::SparseMatrix<double> L_2(cloud.rows(), cloud.rows());

    for (int i=0; i<cloud.rows(); i++) {
        /* The laplacian operator is defined with the cotangent weights for each edge.
         * E.g., with the two triangles (abd) and (acd), for the edge (ad) we sum cotan(b) and cotan(c).
         * 
         *    a---b
         *    | \ |
         *    c---d
         * 
         */

        Eigen::Vector3d a = cloud.row(i);

        std::vector<int> one_ring = one_ring_list[i].get_one_ring();
        for (int j=0; j<one_ring.size(); j++) {

            double cot_theta = 0;
            
            Eigen::Vector3d d = cloud.row(one_ring[j]);

            std::vector<int> connected_element = one_ring_list[i].get_connected_components(j);
            
            for (int connected_element_it=0; connected_element_it<connected_element.size(); connected_element_it++) {
                Eigen::Vector3d b = cloud.row(connected_element[connected_element_it]);

                Eigen::Vector3d ba = b-a;
                Eigen::Vector3d bd = b-d;

                cot_theta += cotan(ba, bd);//( ba.dot(bd) ) / ( (ba.cross(bd)).norm() )
            }

            //cot_theta /= connected_element.size();
            //if (cot_theta>10000)
            //    cot_theta = 10000;
            
            L_2.coeffRef(i, one_ring[j]) += cot_theta;
            L_2.coeffRef(i, i) -= cot_theta;
        }
    }
    
    return L_2;
};


#endif
