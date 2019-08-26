/**
 * Author: R. Falque
 * 
 * plot graph in the libigl viewer
 * by R. Falque
 * 14/02/2019
 **/

#ifndef PLOT_MESH_H
#define PLOT_MESH_H

#include <Eigen/Core>


#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

inline bool init_polyscope() {
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 1024;
}

inline bool add_mesh (const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& V_color) {
    polyscope::registerSurfaceMesh("input mesh", V, F);
    polyscope::getSurfaceMesh("input mesh")->addVertexColorQuantity("fColor", V_color);
    return 0;
};

inline bool add_mesh (const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    polyscope::registerSurfaceMesh("input mesh", V, F);
    return 0;
};

inline bool add_cloud (const Eigen::MatrixXd& V, const Eigen::MatrixXd& V_color) {
    polyscope::registerPointCloud("point cloud", V);
    polyscope::getPointCloud("point cloud")->addColorQuantity("fColor", V_color);
    return 0;
};

inline bool add_cloud (const Eigen::MatrixXd& V) {
    polyscope::registerPointCloud("point cloud", V);
    return 0;
};




inline bool plot_mesh (const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& V_color)
{
    init_polyscope();
    add_mesh (V, F, V_color);
    polyscope::show();
    return 0;
};

inline bool plot_mesh (const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    init_polyscope();
    add_mesh (V, F);
    polyscope::show();
    return 0;
};

inline bool plot_mesh_and_cloud (const Eigen::MatrixXd& mesh_V, const Eigen::MatrixXi& mesh_F, const Eigen::MatrixXd& cloud_V) {   
    init_polyscope();
    add_mesh (mesh_V, mesh_F);
    add_cloud (cloud_V);
    polyscope::show();
    return 0;
};

inline bool plot_cloud (const Eigen::MatrixXd& V) {
    init_polyscope();
    add_cloud (V);
    polyscope::show();
    return 0;
};


inline bool plot_cloud_with_color (const Eigen::MatrixXd& V, const Eigen::MatrixXd& color) {
    init_polyscope();
    add_cloud (V, color);
    polyscope::show();
    return 0;
};


#endif
