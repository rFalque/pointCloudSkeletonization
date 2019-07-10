#ifndef LIB_GRAPH_OPTIONS_HPP
#define LIB_GRAPH_OPTIONS_HPP

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct graph_options
{
    // path to the graph to load
    std::string path_graph_obj;

    // general options
    bool visualization;
    bool verbose;

    // visualization parameters
    double nodes_ratio;
    double edges_ratio;
    double graph_res;
    std::vector<double> nodes_color;
    std::vector<double> edges_color;

    void print(){
        std::cout << "list of the parameters:" << std::endl;
        std::cout << std::endl;
        std::cout << "*** IO files: ***" << std::endl;
        std::cout <<  "path_graph_obj: " << path_graph_obj << std::endl;
        std::cout << std::endl;
        std::cout << "*** General parameters ***" << std::endl;
        std::cout << "visualization: " << visualization << std::endl;
        std::cout << "verbose: " << verbose << std::endl;
        std::cout << std::endl;
        std::cout << "*** Embedded deformation parameters ***" << std::endl;
        std::cout << "nodes_ratio: " << nodes_ratio << std::endl;
        std::cout << "edges_ratio: " << edges_ratio << std::endl;
        std::cout << "graph_res:   " << graph_res << std::endl;
        std::cout << "nodes_color: [" << nodes_color[0] << "," << nodes_color[1] << "," << nodes_color[2] << "]" << std::endl;
        std::cout << "edges_color: [" << edges_color[0] << "," << edges_color[1] << "," << edges_color[2] << "]" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
    }

    bool loadYAML(std::string config_file){
        YAML::Node config = YAML::LoadFile(config_file);
        
        // IO
        path_graph_obj     = config["io_files"]["graph_obj"].as<std::string>();

        // general options
        visualization      = config["general_params"]["visualization"].as<bool>();
        verbose            = config["general_params"]["verbose"].as<bool>();

        // visualization parameters
        nodes_ratio        = config["visualization_params"]["nodes_ratio"].as<double>();
        edges_ratio        = config["visualization_params"]["edges_ratio"].as<double>();
        graph_res          = config["visualization_params"]["graph_res"].as<double>();
        nodes_color        = config["visualization_params"]["nodes_color"].as<std::vector<double>>();
        edges_color        = config["visualization_params"]["edges_color"].as<std::vector<double>>();

        if (verbose)
            print();
    }
};

#endif
