/**
 * Author: R. Falque
 * 
 * main for testing the skeleton extraction
 * by R. Falque
 * 27/06/2019
 **/

// dependencies

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <igl/opengl/glfw/Viewer.h>

#include "mesh_tools/plotMesh.hpp"
#include "mesh_tools/nanoflannWrapper.hpp"

#include "utils/loadCSV.hpp"
#include "utils/options.hpp"
#include "utils/EigenSparseConcatenate.hpp"

#include <Eigen/Dense>

#include "skeleton_toolbox/point_ring.hpp"
#include "skeleton_toolbox/compute_initial_laplacian_constraint_weight.hpp"
#include "skeleton_toolbox/compute_point_laplacian.hpp"
#include "skeleton_toolbox/normalization.hpp"

#include <ctime>

#include <yaml-cpp/yaml.h>
#include <vector>

#include<Eigen/SparseCholesky>



/*
 * List of things that could go wrong:
 *  - in the laplacian computation, line (63), ba instead of ab?
 *  - in the laplacian computation, line (66), division by number of neighbours needed?
 *  - in the point_ring, with SVD decomposition, is V the proper matrix for the 2D projection
 *  - in the point_ring, what happen if the one_ring is not connected?
 *  - does the laplacian need to be symmetric (see compute_point_laplacian.m line 31)
 *  - does the laplacian need to be normalized
 */


/* TODO:
 *  - add upper bound to avoid division by 0
 *  - add stopping criteria
 */

int main(int argc, char* argv[])
{
    options opts;
    opts.loadYAML("../config.yaml");

    Eigen::MatrixXd V; // V: vertex of the surface
    Eigen::MatrixXi F; // F: faces of the surface

    igl::readPLY(opts.path_input_file,V, F);
    //igl::readOFF("../data/simplejoint_v4770.off", V, F);
	plot_mesh(V,F);
    normalization(V);

    int k_for_knn = 30;

    // get the nearest neighbours
    nanoflann_wrapper knn_search(V);

    // get the one ring
    std::vector< OneRing > one_ring_list;
    for (int i=0; i<V.rows(); i++) {
        // (1) search for k closest points and (2) remove the point itself
        std::vector < int > neighbours_id;
        neighbours_id = knn_search.return_k_closest_points(V.row(i), k_for_knn+1);
        neighbours_id.erase(neighbours_id.begin());

        // look for the one_ring neighbors
        OneRing one_ring(i, neighbours_id, V);
        one_ring_list.push_back(one_ring);
    }

    int iteration_time = 3;
    double termination_criteria = 0.01;

    double sl = 3;



    // initialization of: WH, WL, and L
    Eigen::SparseMatrix<double> WH(V.rows(), V.rows());
    for (int i=0; i<V.rows(); i++)
        WH.insert(i,i) = 1;
    
    double initWL = compute_initial_laplacian_weight(V, one_ring_list);

    Eigen::SparseMatrix<double> WL(V.rows(), V.rows());
    for (int i=0; i<V.rows(); i++)
        WL.insert(i,i) = initWL;

    Eigen::SparseMatrix<double> L = compute_laplacian_weight(V, one_ring_list);
    Eigen::MatrixXd P, P2;

    // contraction time :)
    Eigen::SparseMatrix<double> A, A2, WlL;
    Eigen::MatrixXd b, b2, WhP, zeros;

    WhP = WH*V;
    WlL = WL*L;
    zeros = Eigen::MatrixXd::Zero(WhP.rows(), 3);

    A = concatenate(WlL, WH, 1);
    b = concatenate(zeros, WhP, 1);
    A2 = A.transpose() * A;
    b2 = A.transpose() * b;


    std::clock_t start;
    double duration;
    start = std::clock();

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(A2);
    if(solver.info()!=Eigen::Success) {
        // decomposition failed
        std::cout << "Error: solver failed\n";
    }
    P = solver.solve(b2);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"SimplicialLDLT runned in : "<< duration <<" s\n";

    plot_mesh(P,F);

    // next step:
    for (int i=0; i< iteration_time; i++) {

        L = compute_laplacian_weight(P, one_ring_list);
        WL *= 3;
        Eigen::VectorXd size, new_size;
        size = one_ring_size(V, one_ring_list);
        new_size = one_ring_size(P, one_ring_list);

        for (int i=0; i<V.rows(); i++)
            WH.coeffRef(i,i) *= size(i)/new_size(i);


        WhP = WH*P;
        WlL = WL*L;

        A = concatenate(WlL, WH, 1);
        b = concatenate(zeros, WhP, 1);
        A2 = A.transpose() * A;
        b2 = A.transpose() * b;



        solver.compute(A2);
        if(solver.info()!=Eigen::Success) {
            // decomposition failed
            std::cout << "Error: solver failed\n";
        }
        P = solver.solve(b2);

        plot_mesh(P,F);

    }

    // 
    double sample_radius = 0.0357;
    


    return 0;
}

