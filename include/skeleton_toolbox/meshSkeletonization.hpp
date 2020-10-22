#ifndef MESH_SKELETONIZATION_HPP
#define MESH_SKELETONIZATION_HPP

#include <algorithm>           // std::all_of
#include <Eigen/Core>
#include <limits>
#include <math.h>

#include "libGraphCpp/include/libGraphCpp/graph.hpp"
#include "libGraphCpp/include/libGraphCpp/plotGraph.hpp"

#include "farthest_sampling.hpp"

#include "EigenTools/concatenate.hpp"
#include "EigenTools/sparse.hpp"
#include "EigenTools/minMax.hpp"
#include "EigenTools/nanoflannWrapper.hpp"

#include "options.hpp"
#include "matplotlibcpp.h"
#include "plotMesh.hpp"
#include "plotCloud.hpp"
#include "plotHybrid.hpp"

class MeshSkeletonization
{
private:
    options opts_;                                    // Store all visualization infos

    Eigen::MatrixXd pointcloud_;                      // pointcloud
	Eigen::MatrixXd contracted_pointcloud_;           // contracted pointcloud
    Eigen::MatrixXi faces_;
	Eigen::VectorXi correspondences_;                 // association between each point from pointcloud_ and the nodes of the skeleton
	
	libgraphcpp::Graph* skeleton_;

    bool initialized_ = false;

    // normalization factor
    Eigen::Vector3d nomalization_deplacement_;
    double normalization_scaling_;

    // internal functions
	inline Eigen::VectorXd one_ring_size(const Eigen::MatrixXd & vertices, const Eigen::MatrixXi & faces, std::string distance_type);
    inline double cotan(Eigen::Vector3d v, Eigen::Vector3d w);
    inline Eigen::SparseMatrix<double> laplacian(Eigen::MatrixXd & vertices, Eigen::MatrixXi & faces);
    inline Eigen::MatrixXd solve_contraction(Eigen::SparseMatrix<double> WH, 
                                             Eigen::SparseMatrix<double> WL, 
                                             Eigen::SparseMatrix<double> L, 
                                             Eigen::MatrixXd points);
    inline bool connect_by_faces(const Eigen::MatrixXd & cloud, 
                                 const Eigen::MatrixXd & nodes, 
                                 const Eigen::VectorXi & correspondences, 
                                 const Eigen::MatrixXi & faces, 
                                 Eigen::MatrixXi & A);
public:

	// overloaded creations of the class (set verbose and load the data)
	MeshSkeletonization(Eigen::MatrixXd& V, Eigen::MatrixXi& F, options opts)
	{
        opts_ = opts;
		pointcloud_ = V;
		faces_ = F;
	}


	// destructor
	~MeshSkeletonization()
	{
	}

    void skeletonize()
    {
        init();
        normalization();
        laplacian_contraction();
        contracted_cloud_to_skeleton();
        un_normalization();
    }

	// initialisation of the private variables
	bool init()
	{
        initialized_ = true;
		return true;
	}

	void laplacian_contraction() 
    {
        if ( not initialized_ )
            init();

        // store the contraction for each step
        std::vector<double> contraction_history;

	    // initialization of: WH, WL, and L
    	Eigen::VectorXd ms = one_ring_size(pointcloud_, faces_, "mean");
    	double initWL = 1.0/(5.0*ms.mean());

        Eigen::SparseMatrix<double> WL = SparseDiagonalMatrix::Constant(pointcloud_.rows(), initWL);
        Eigen::SparseMatrix<double> WH = SparseDiagonalMatrix::Constant(pointcloud_.rows(), 1);

		// get laplacian
	    Eigen::SparseMatrix<double> L = laplacian(pointcloud_, faces_);

        std::cout<<"contraction step : 0\n";
		contracted_pointcloud_ = solve_contraction(WH, WL, L, pointcloud_);

        // further contraction steps
        int iteration_time = opts_.iteration_time;
        double termination_criteria = opts_.termination_criteria;
        double sl = opts_.sl;
        double WC = opts_.WC;

        Eigen::VectorXd size = one_ring_size(pointcloud_, faces_, "min");
        Eigen::VectorXd new_size = one_ring_size(contracted_pointcloud_, faces_, "min");
        double contraction_rate = new_size.mean() / size.mean();
        contraction_history.push_back( contraction_rate );
        std::cout<<"contraction ratio : "<< contraction_history.back() <<"\n";

        if (opts_.visualization)
                plot_mesh_and_cloud (contracted_pointcloud_, faces_, pointcloud_);

        for (int i=0; i< iteration_time; i++) {
            std::cout<<"contraction step : "<< i+1 <<"\n";

            // update WL
            WL *= sl;

            // update WH
            for (int i=0; i<pointcloud_.rows(); i++) {
                if (WC*size(i)/new_size(i) < opts_.MAX_POSITION_CONSTRAINT_WEIGHT) {
                    WH.coeffRef(i,i) = WC*size(i)/new_size(i);
                } else {
                    std::cout << "Warning: reached upper limit for WH: " << WC*size(i)/new_size(i) << "\n";
                    WH.coeffRef(i,i) = opts_.MAX_POSITION_CONSTRAINT_WEIGHT;
                }
            }

            // set up upper limit
            if (WL.coeffRef(0,0) * sl > opts_.MAX_LAPLACIAN_CONSTRAINT_WEIGHT) {
                std::cout << "Warning: reached upper limit for the WL: " << WL.coeffRef(0,0) * sl << "\n";
                for (int i=0; i<pointcloud_.rows(); i++)
                    WL.coeffRef(i,i) = opts_.MAX_LAPLACIAN_CONSTRAINT_WEIGHT;
            }

            // recompute L
            L = laplacian(contracted_pointcloud_, faces_);

            // i^th +1 contraction
            Eigen::MatrixXd contracted_pointcloud_temp = solve_contraction(WH, WL, L, contracted_pointcloud_);
            
            // store contraction history and update the contracted_pointcloud_ if the contraction_rate is good enough
            new_size = one_ring_size(contracted_pointcloud_temp, faces_, "min");
            contraction_rate = new_size.mean() / size.mean();
            if (contraction_history.rbegin()[0] - contraction_rate < opts_.termination_criteria)
                break;

            contraction_history.push_back( contraction_rate );
            std::cout<<"contraction ratio : "<< contraction_history.back() <<"\n";
            contracted_pointcloud_ = contracted_pointcloud_temp;
            
            if (opts_.visualization) {
                    plot_mesh_and_cloud (contracted_pointcloud_, faces_, pointcloud_);
            }
        }

        if (opts_.visualization_with_matplotlibcpp) {
            std::vector <int> x(contraction_history.size());
            std::iota(x.begin(), x.end(), 0);
            
            matplotlibcpp::plot(x, contraction_history);
            matplotlibcpp::xlabel("contraction iteration");
            matplotlibcpp::ylabel("contraction ratio");
            matplotlibcpp::title("Contraction history:");
            matplotlibcpp::show();
        }

	}

    bool contracted_cloud_to_skeleton() 
    {
        // turn into a Skeletonization part
        double scale;
        getScale(pointcloud_, scale);

        Eigen::MatrixXd nodes;
        Eigen::VectorXi correspondences;

        if (opts_.use_radius) {
            double sample_radius = scale * opts_.sample_radius;
            farthest_sampling_by_sphere(contracted_pointcloud_, sample_radius, nodes, correspondences);
        } else {
            int k = round(pointcloud_.rows() / opts_.sample_ratio);
            farthest_sampling_by_knn(contracted_pointcloud_, k, nodes, correspondences);
        }

        Eigen::MatrixXi adjacency_matrix;
        connect_by_faces(pointcloud_, nodes, correspondences, faces_, adjacency_matrix);

        skeleton_ = new libgraphcpp::Graph(nodes, adjacency_matrix);
        skeleton_->init();
        
        Eigen::VectorXi updated_correspondences = Eigen::VectorXi::Constant(correspondences.rows(), -1);
        std::vector<std::vector <int> > merged_nodes;
        if (opts_.skeleton_editing != 0 ) {
            merged_nodes = skeleton_->make_tree();

            // update the point correspondence
            for (int k=0; k<correspondences.rows(); k++) 
                for (int i=0; i<merged_nodes.size(); i++)
                    for (int j=0; j<merged_nodes[i].size(); j++)
                        if (correspondences(k) == merged_nodes[i][j])
                            updated_correspondences(k) = i;
        } else {
            updated_correspondences = correspondences;
        }
        
        if (opts_.visualization)
            visualization::plot(*skeleton_, "skeletonization");

        if ( (updated_correspondences.array() == -1).any() )
            std::cout << "Warning: bad point correspondence\n'";
        
        correspondences_ = updated_correspondences;

        return true;
    }

    inline bool normalization()
    {
        // centre in zero
        nomalization_deplacement_ = (pointcloud_.colwise().minCoeff() + pointcloud_.colwise().maxCoeff()) / 2;
        pointcloud_.rowwise() -= nomalization_deplacement_.transpose();

        // make the diagonal 1.6
        normalization_scaling_ = 1.6/(pointcloud_.colwise().maxCoeff() - pointcloud_.colwise().minCoeff()).maxCoeff();
        pointcloud_ *= normalization_scaling_;

        return true;
    }

    inline bool un_normalization()
    {
        pointcloud_ /= normalization_scaling_;
        pointcloud_.rowwise() += nomalization_deplacement_.transpose();

        contracted_pointcloud_ /= normalization_scaling_;
        contracted_pointcloud_.rowwise() += nomalization_deplacement_.transpose();

        skeleton_->transform(normalization_scaling_, nomalization_deplacement_);

        return true;
    }

    // set up assessors
    Eigen::VectorXi get_correspondences() 
    {
        return correspondences_;
    }

    libgraphcpp::Graph get_skeleton() 
    {
        return * skeleton_;
    }

};



// return for each point the mean distance to its neighbours
inline Eigen::VectorXd MeshSkeletonization::one_ring_size(const Eigen::MatrixXd & vertices, const Eigen::MatrixXi & faces, std::string distance_type)
{
    // initialization
    Eigen::VectorXd out = Eigen::VectorXd::Zero(vertices.rows());

    // sum all triangle (abc) 
    if (distance_type=="area")
    {
        // for each triangles, for each point, add the area of the triangle
        for (int i=0; i<faces.rows(); i++) {
            Eigen::Vector3d v_0 = vertices.row(faces(i,0));
            Eigen::Vector3d v_1 = vertices.row(faces(i,1));
            Eigen::Vector3d v_2 = vertices.row(faces(i,2));

            double triangle_area = ((v_0-v_1).cross(v_0-v_2)).norm()*1/4;

            out( faces(i,0) ) += triangle_area;
            out( faces(i,1) ) += triangle_area;
            out( faces(i,2) ) += triangle_area;
        }

    }
    else // min, mean or max distance from a point to its neighbours
    {

        // build a vector of vector:
        std::vector <std::vector<double>> distance_to_neighbours(vertices.rows());

        // for each triangles, for each point, add the length of the opposite edge
        for (int i=0; i<faces.rows(); i++) {
            // first vertex
            distance_to_neighbours[ faces(i,0) ].push_back( (vertices.row(faces(i,0)) - vertices.row(faces(i,1))).norm() );
            distance_to_neighbours[ faces(i,0) ].push_back( (vertices.row(faces(i,0)) - vertices.row(faces(i,2))).norm() );

            // second vertex
            distance_to_neighbours[ faces(i,1) ].push_back( (vertices.row(faces(i,1)) - vertices.row(faces(i,0))).norm() );
            distance_to_neighbours[ faces(i,1) ].push_back( (vertices.row(faces(i,1)) - vertices.row(faces(i,2))).norm() );

            // third vertex
            distance_to_neighbours[ faces(i,2) ].push_back( (vertices.row(faces(i,2)) - vertices.row(faces(i,0))).norm() );
            distance_to_neighbours[ faces(i,2) ].push_back( (vertices.row(faces(i,2)) - vertices.row(faces(i,1))).norm() );
        }

        // get the mean of all the distance to the point
        for (int i=0; i<vertices.rows(); i++) {
            std::vector<double> temp = distance_to_neighbours[i];

            if (distance_type=="min")
                out(i) = temp[std::distance(temp.begin(), std::min_element(temp.begin(), temp.end()))];
            else if (distance_type=="mean")
                out(i) = std::accumulate(temp.begin(), temp.end(), 0.0)/temp.size();
            else if (distance_type=="max")
                out(i) = temp[std::distance(temp.begin(), std::max_element(temp.begin(), temp.end()))];
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
inline double MeshSkeletonization::cotan(Eigen::Vector3d v, Eigen::Vector3d w)
{ 
    return( ( v.dot(w) ) / ( (v.cross(w)).norm() ) ); 
};


inline Eigen::SparseMatrix<double> MeshSkeletonization::laplacian(Eigen::MatrixXd & vertices, Eigen::MatrixXi & faces)
{
    Eigen::SparseMatrix<double> L(vertices.rows(), vertices.rows());

    for (int i=0; i<faces.rows(); ++i) {
        /* The laplacian operator is defined with the cotangent weights for each edge.
         * E.g., with the two triangles (abd) and (acd), for the edge (ad) we sum cotan(b) and cotan(c).
         * 
         *    a---b
         *    | \ |
         *    c---d
         * 
         */

        Eigen::Vector3d v_0 = vertices.row(faces(i,0));
        Eigen::Vector3d v_1 = vertices.row(faces(i,1));
        Eigen::Vector3d v_2 = vertices.row(faces(i,2));

        L.coeffRef(faces(i,0), faces(i,1)) +=  cotan(v_2-v_0, v_2-v_1);
        L.coeffRef(faces(i,1), faces(i,0)) +=  cotan(v_2-v_0, v_2-v_1);
        L.coeffRef(faces(i,0), faces(i,0)) -=  cotan(v_2-v_0, v_2-v_1);
        L.coeffRef(faces(i,1), faces(i,1)) -=  cotan(v_2-v_0, v_2-v_1);

        L.coeffRef(faces(i,0), faces(i,2)) +=  cotan(v_1-v_0, v_1-v_2);
        L.coeffRef(faces(i,2), faces(i,0)) +=  cotan(v_1-v_0, v_1-v_2);
        L.coeffRef(faces(i,0), faces(i,0)) -=  cotan(v_1-v_0, v_1-v_2);
        L.coeffRef(faces(i,2), faces(i,2)) -=  cotan(v_1-v_0, v_1-v_2);

        L.coeffRef(faces(i,1), faces(i,2)) +=  cotan(v_0-v_1, v_0-v_2);
        L.coeffRef(faces(i,2), faces(i,1)) +=  cotan(v_0-v_1, v_0-v_2);
        L.coeffRef(faces(i,1), faces(i,1)) -=  cotan(v_0-v_1, v_0-v_2);
        L.coeffRef(faces(i,2), faces(i,2)) -=  cotan(v_0-v_1, v_0-v_2);

    }

    return L;
};

inline Eigen::MatrixXd MeshSkeletonization::solve_contraction(Eigen::SparseMatrix<double> WH, 
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

inline bool MeshSkeletonization::connect_by_faces(const Eigen::MatrixXd & cloud, 
                                                  const Eigen::MatrixXd & nodes, 
                                                  const Eigen::VectorXi & correspondences, 
                                                  const Eigen::MatrixXi & faces, 
                                                  Eigen::MatrixXi & A) // A for adjacency matrix
{
    A=Eigen::MatrixXi::Zero( nodes.rows(), nodes.rows() );
    for (int i=0; i<faces.rows(); ++i) {
        if (correspondences(faces(i, 0)) != correspondences(faces(i, 1))) {
            A(correspondences(faces(i, 0)), correspondences(faces(i, 1))) = 1;
            A(correspondences(faces(i, 1)), correspondences(faces(i, 0))) = 1;
        }
        
        if (correspondences(faces(i, 0)) != correspondences(faces(i, 2))) {
            A(correspondences(faces(i, 0)), correspondences(faces(i, 2))) = 1;
            A(correspondences(faces(i, 2)), correspondences(faces(i, 0))) = 1;
        }
        
        if (correspondences(faces(i, 1)) != correspondences(faces(i, 2))) {
            A(correspondences(faces(i, 1)), correspondences(faces(i, 2))) = 1;
            A(correspondences(faces(i, 2)), correspondences(faces(i, 1))) = 1;
        }
    }

    return true;
};

#endif
