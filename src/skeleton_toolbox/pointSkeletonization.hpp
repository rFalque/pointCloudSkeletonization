#ifndef POINT_SKELETONIZATION_HPP
#define POINT_SKELETONIZATION_HPP

#include <algorithm>           // std::all_of
#include <Eigen/Core>
#include <limits>

#include "point_ring.hpp"
//#include "pointSkeletonizationOptions.hpp"

#include "../graph_lib/graphStructure.hpp"
#include "../mesh_tools/nanoflannWrapper.hpp"
#include "../mesh_tools/plotMesh.hpp"

#include "farthest_sampling_by_sphere.hpp"
#include "connect_by_inherit_neigh.hpp"

#include "../utils/options.hpp"
#include "../utils/EigenConcatenate.hpp"
#include "../utils/EigenTools.hpp"
#include "../utils/EigenMinMax.hpp"
#include "../utils/matplotlibcpp.h"

class PointSkeletonization
{
private:
    options opts_;                                                      // Store all visualization infos

    Eigen::MatrixXd pointcloud_;                                        // pointcloud
	Eigen::MatrixXd contracted_pointcloud_;                             // contracted pointcloud
	Eigen::VectorXi correspondences_;                                    // association between each point from pointcloud_ and the nodes of the skeleton
	
    Eigen::MatrixXi F_;                                                 // F: faces of the surface (for plots)
	
	std::vector< OneRing > one_ring_list_;

	Graph* skeleton_;

    bool initialized_ = false;

    // normalization factor
    Eigen::Vector3d nomalization_deplacement_;
    double normalization_scaling_;

    // internal functions
	inline bool normalization();
	inline Eigen::VectorXd one_ring_size(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list, std::string distance_type);
    inline double cotan(Eigen::Vector3d v, Eigen::Vector3d w);
    inline Eigen::SparseMatrix<double> laplacian(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list);
    inline Eigen::MatrixXd solve_contraction(Eigen::SparseMatrix<double> WH, 
                                             Eigen::SparseMatrix<double> WL, 
                                             Eigen::SparseMatrix<double> L, 
                                             Eigen::MatrixXd points);
public:

	// overloaded creations of the class (set verbose and load the data)
	PointSkeletonization(Eigen::MatrixXd V, Eigen::MatrixXi F, options opts)
	{
        opts_ = opts;
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
		nanoflann_wrapper knn_search(pointcloud_);
        int k_for_knn = 30;

		// get the one ring
		for (int i=0; i<pointcloud_.rows(); i++) {
			// (1) search for k closest points and (2) remove the point itself
			std::vector < int > neighbours_id;
			neighbours_id = knn_search.return_k_closest_points(pointcloud_.row(i), k_for_knn+1);
			neighbours_id.erase(neighbours_id.begin());

			// look for the one_ring neighbors
			OneRing one_ring(i, neighbours_id, pointcloud_);
			one_ring_list_.push_back(one_ring);
		}

        initialized_ = true;
		return true;
	}

	bool laplacian_contraction() 
    {
        if ( not initialized_ )
            init();

        // store the contraction for each step
        std::vector<double> contraction_history;

	    // initialization of: WH, WL, and L
    	Eigen::VectorXd ms = one_ring_size(pointcloud_, one_ring_list_, "mean");
    	double initWL = 1.0/(5.0*ms.mean());

        Eigen::SparseMatrix<double> WL = SparseDiagonalMatrix::Constant(pointcloud_.rows(), initWL);
        Eigen::SparseMatrix<double> WH = SparseDiagonalMatrix::Constant(pointcloud_.rows(), 1);

		// get laplacian
	    Eigen::SparseMatrix<double> L = laplacian(pointcloud_, one_ring_list_);

		contracted_pointcloud_ = solve_contraction(WH, WL, L, pointcloud_);

        // further contraction steps
        int iteration_time = opts_.iteration_time;
        double termination_criteria = opts_.termination_criteria;
        double sl = opts_.sl;
        double WC = opts_.WC;

        if (opts_.visualization)
            plot_mesh_and_cloud (contracted_pointcloud_, F_, pointcloud_);
        Eigen::VectorXd size = one_ring_size(pointcloud_, one_ring_list_, "min");

        for (int i=0; i< iteration_time; i++) {

            Eigen::VectorXd new_size = one_ring_size(contracted_pointcloud_, one_ring_list_, "min");
            contraction_history.push_back(new_size.mean() / size.mean() );
            if (contraction_history.size()>2)
                if (contraction_history.rbegin()[1] - contraction_history.rbegin()[0] < opts_.termination_criteria)
                    break;

            std::cout<<"previous contraction ratio : "<< contraction_history.back() <<"\n";
            std::cout<<"contraction step : "<< i+1 <<"\n";

            // update WL
            WL *= sl;

            // update WH
            for (int i=0; i<pointcloud_.rows(); i++)
                if (WC*size(i)/new_size(i) < opts_.MAX_POSITION_CONSTRAINT_WEIGHT)
                    WH.coeffRef(i,i) = WC*size(i)/new_size(i);
                else
                {
                    std::cout << "Warning: reached upper limit for WH: " << WC*size(i)/new_size(i) << "\n";
                    WH.coeffRef(i,i) = opts_.MAX_POSITION_CONSTRAINT_WEIGHT;
                }

            // set up upper limit
            if (WL.coeffRef(0,0) * sl > opts_.MAX_LAPLACIAN_CONSTRAINT_WEIGHT)
            {
                std::cout << "Warning: reached upper limit for the WL: " << WL.coeffRef(0,0) * sl << "\n";
                for (int i=0; i<pointcloud_.rows(); i++)
                    WL.coeffRef(i,i) = opts_.MAX_LAPLACIAN_CONSTRAINT_WEIGHT;
            }

            // recompute L
            L = laplacian(contracted_pointcloud_, one_ring_list_);

            // i^th +1 contraction
            contracted_pointcloud_ = solve_contraction(WH, WL, L, contracted_pointcloud_);
            
            if (opts_.visualization)
                plot_mesh_and_cloud (contracted_pointcloud_, F_, pointcloud_);
        }

        if (opts_.visualization)
        {
            std::vector <int> x(contraction_history.size());
            std::iota(x.begin(), x.end(), 0);
            
            matplotlibcpp::plot(x, contraction_history);
            matplotlibcpp::title("Contraction history:");
            matplotlibcpp::show();
        }
	}

    bool skeletonization() 
    {
        // turn into a Skeletonization part
        double sample_radius;
        getScale(pointcloud_, sample_radius);
        sample_radius *= opts_.sample_radius;

        Eigen::MatrixXd nodes;
        Eigen::VectorXi correspondences;
        farthest_sampling_by_sphere(contracted_pointcloud_, sample_radius, nodes, correspondences);





        Eigen::MatrixXi adjacency_matrix;
        connect_by_inherit_neigh(pointcloud_, nodes, correspondences, one_ring_list_, adjacency_matrix);


        // Graph skeleton(nodes, adjacency_matrix);
        std::vector<std::vector <int> > merged_nodes;
        skeleton_ = new Graph(nodes, adjacency_matrix);
        skeleton_->init();
        skeleton_->plot();
        merged_nodes = skeleton_->make_1D_curve();
        skeleton_->plot();

        if (opts_.verbose) {
            std::cout << "Nodes merged together:\n";
            for (int i=0; i<merged_nodes.size(); i++) {
                std::cout << "The " << i << "-th point now contains the points: ";
                for (int j=0; j<merged_nodes[i].size(); j++) {
                    std::cout << merged_nodes[i][j] << " ";
                }
                std::cout << "\n";
            }
        }

        // update the point correspondence
        Eigen::VectorXi updated_correspondences = Eigen::VectorXi::Constant(correspondences.rows(), -1);

        for (int k=0; k<correspondences.rows(); k++) 
            for (int i=0; i<merged_nodes.size(); i++)
                for (int j=0; j<merged_nodes[i].size(); j++)
                        if (correspondences(k) == merged_nodes[i][j])
                            updated_correspondences(k) = i;
        
        std::cout << "correspondences.max" << correspondences.maxCoeff() << "\n";

        if ( (updated_correspondences.array() == -1).any() )
            std::cout << "Oh noooooo\n'";
        
        correspondences_ = updated_correspondences;

        return true;
    }

    Eigen::VectorXi get_correspondences() {
        return correspondences_;
    }    

};

inline bool PointSkeletonization::normalization()
{
    // centre in zero
    nomalization_deplacement_ = (pointcloud_.colwise().minCoeff() + pointcloud_.colwise().maxCoeff()) / 2;
    pointcloud_.rowwise() -= nomalization_deplacement_.transpose();

    // make the diagonal 1.6
    normalization_scaling_ = 1.6/(pointcloud_.colwise().maxCoeff() - pointcloud_.colwise().minCoeff()).maxCoeff();
    pointcloud_ *= normalization_scaling_;

    return true;
};

// return for each point the mean distance to its neighbours
inline Eigen::VectorXd PointSkeletonization::one_ring_size(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list, std::string distance_type)
{
    // sum all triangle (abc) 
    Eigen::VectorXd out = Eigen::VectorXd::Zero(cloud.rows());
    if (distance_type=="area")
    {


        for (int i=0; i<cloud.rows(); i++) {

            Eigen::Vector3d a = cloud.row(i);

            std::vector<int> one_ring = one_ring_list[i].get_one_ring();
            for (int j=0; j<one_ring.size(); j++) {
                
                Eigen::Vector3d d = cloud.row(one_ring[j]);

                std::vector<int> connected_element = one_ring_list[i].get_connected_components(j);
                
                for (int connected_element_it=0; connected_element_it<connected_element.size(); connected_element_it++) {
                    Eigen::Vector3d b = cloud.row(connected_element[connected_element_it]);

                    Eigen::Vector3d ba = b-a;
                    Eigen::Vector3d bd = b-d;


                    out(i) += ((ba).cross(bd)).norm()*1/4;
                }
            }
        }


    }
    else
    {
        for (int i=0; i<cloud.rows(); i++) {
            // get the point:
            Eigen::Vector3d vertex;
            vertex = cloud.row(i);

            // get the corresponding one ring structure
            std::vector<int> one_ring = one_ring_list.at(i).get_one_ring();

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

inline Eigen::SparseMatrix<double> PointSkeletonization::laplacian(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list) 
{
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
        bool require_normalization = false;
        double max_cot_value = 0;

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

            if (cot_theta>opts_.laplacian_threshold) {
                require_normalization = true;
                max_cot_value = std::max(max_cot_value, cot_theta);
            }

            L_2.coeffRef(i, one_ring[j]) += cot_theta;
            L_2.coeffRef(i, i) -= cot_theta;
        }

        if (require_normalization)
        {
            //std::cout << "Warning: very high laplacian value reached: cot_theta=" << max_cot_value << std::endl;
            for (int j=0; j<one_ring.size(); j++) {
                L_2.coeffRef(i, one_ring[j]) *= opts_.laplacian_threshold / max_cot_value;
            }
            L_2.coeffRef(i, i) *= opts_.laplacian_threshold / max_cot_value;
        }

    }
    
    return L_2;
};

inline Eigen::MatrixXd PointSkeletonization::solve_contraction(Eigen::SparseMatrix<double> WH, 
                                                               Eigen::SparseMatrix<double> WL, 
                                                               Eigen::SparseMatrix<double> L, 
                                                               Eigen::MatrixXd points)
{
	Eigen::SparseMatrix<double> A, A2, WlL;
	Eigen::MatrixXd b, b2, WhP, zeros;

	WhP = WH*points;
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
    return solver.solve(b2);
};




#endif
