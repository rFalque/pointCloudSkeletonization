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
#include <igl/opengl/glfw/Viewer.h>

#include "../skeleton_toolbox/point_ring.hpp"


std::vector< OneRing > one_ring_list_copy;
int vertex_iterator = 0;


bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    std::cout<<"Key: "<<key<<" "<<(unsigned int)key<<std::endl;
    if (key == '1')
    {
        if (vertex_iterator < one_ring_list_copy.size())
            vertex_iterator++;
        else
            vertex_iterator =0;
        
        Eigen::MatrixXd vertices_color = Eigen::MatrixXd::Constant(one_ring_list_copy.size(), 3, 0.9);
        std::vector< int > one_ring = one_ring_list_copy[vertex_iterator].get_one_ring();

        for (int i = 0; i<one_ring.size(); i++) {
            vertices_color.row(one_ring.at(i)) << 1, 0.1, 0.1;
        }

        viewer.data().set_colors(vertices_color);
    }

  return false;
}


inline bool plot_one_ring (const Eigen::MatrixXd& V,
                       const Eigen::MatrixXi& F,
                       std::vector< OneRing > one_ring_list)
{
    one_ring_list_copy = one_ring_list;
    // visualization
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.callback_key_down = &key_down;
    viewer.launch();

    return 0;
};



inline bool add_mesh (igl::opengl::glfw::Viewer & viewer,
                      const Eigen::MatrixXd& V,
                      const Eigen::MatrixXi& F,
                      const Eigen::MatrixXd& V_color)
{
    // visualization
    viewer.append_mesh();
    viewer.data().set_mesh(V, F);
    viewer.core().background_color << 1, 1, 1, 1;
    viewer.data().set_colors(V_color);

    return 0;
};

inline bool plot_mesh (const Eigen::MatrixXd& V,
                       const Eigen::MatrixXi& F,
                       const Eigen::MatrixXd& V_color)
{
    // visualization
    igl::opengl::glfw::Viewer viewer;
    add_mesh (viewer, V, F, V_color);
    viewer.launch();

    return 0;
};

inline bool plot_mesh (const Eigen::MatrixXd& V,
                       const Eigen::MatrixXi& F)
{
    Eigen::MatrixXd vertices_color;
    vertices_color = Eigen::MatrixXd::Constant(V.rows(), 3, 0.9);

    return plot_mesh (V, F, vertices_color);
};

#endif
