/* Author: R. Falque
 * 
 * main for testing the skeleton extraction
 * by R. Falque
 * 27/06/2019
 */

#include "IO/readOBJ.h"
#include "IO/readOFF.h"

#include <Eigen/Dense>
#include <vector>

#include "polyscope/polyscope.h"

#include "plotMesh.hpp"
#include "plotCloud.hpp"
#include "plotHybrid.hpp"

int main(int argc, char* argv[])
{
    polyscope::init();

    Eigen::MatrixXd V; // V: vertex of the surface
    Eigen::MatrixXd C; // C: color of the surface
    Eigen::MatrixXd N; // N: normal of the surface
    Eigen::MatrixXi F; // F: faces of the surface (used for plots)

    igl::readOFF("../data/crab_full.off", V, F, N, C);
    
    plot_mesh(V, F, C, "../data/output/crab_full.png");

    plot_cloud(V, C);

    screenshot_mesh(V, F, C, "../data/output/crab_full.png");

    return 0;
}

