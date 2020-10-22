/**
 * Author: R. Falque
 * 
 * plot mesh in polyscope
 * by R. Falque
 * 26/09/2019
 **/

#ifndef PLOT_MESH_HPP
#define PLOT_MESH_HPP

#include <Eigen/Core>

#include <string>

#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"

// polyscope wrapper
class MeshVisualization
{
    private:
        std::string mesh_object_name_ = "Mesh";

    public:

        MeshVisualization()
        {
            init();
        }

        // destructor
        ~MeshVisualization()
        {
            polyscope::removeAllStructures();
        }

        void init() {
            // Options
            polyscope::options::autocenterStructures = true;
            polyscope::view::windowWidth = 1024;
            polyscope::view::windowHeight = 1024;
        }

        void add_mesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces) {
            polyscope::registerSurfaceMesh(mesh_object_name_, vertices, faces);
            polyscope::getSurfaceMesh(mesh_object_name_)->setSurfaceColor(glm::vec3{0.1, 0.1, 1});
            polyscope::view::resetCameraToHomeView();
        }

        void add_color(const Eigen::MatrixXd & color, std::string color_name) {
            if (color.rows() != 0){
                polyscope::getSurfaceMesh(mesh_object_name_)->addVertexColorQuantity(color_name, color);
                polyscope::getSurfaceMesh(mesh_object_name_)->getQuantity(color_name)->setEnabled(true);
            }
        }

        void screenshot(std::string screenshot_path) {
            polyscope::screenshot(screenshot_path, false);
        }

        void show() {
            polyscope::show();
            //polyscope::removeAllStructures(); // move to destructor?
        }
};


inline bool plot_mesh (const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces) {
    MeshVisualization viz;
    viz.add_mesh(vertices, faces);
    viz.show();
    return true;
};

inline bool plot_mesh (const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces, const Eigen::MatrixXd& color) {
    MeshVisualization viz;
    viz.add_mesh(vertices, faces);
    viz.add_color(color, "highlight");
    viz.show();
    return true;
};

inline bool plot_mesh (const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces, const Eigen::MatrixXd& color, std::string screenshot_path) {
    MeshVisualization viz;
    viz.add_mesh(vertices, faces);
    viz.add_color(color, "highlight");
    viz.screenshot(screenshot_path);
    viz.show();
    return true;
};

inline bool screenshot_mesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces, const Eigen::MatrixXd& color, std::string screenshot_path) {
    MeshVisualization viz;
    viz.add_mesh(vertices, faces);
    viz.add_color(color, "highlight");
    viz.screenshot(screenshot_path);
    return true;
};


#endif
