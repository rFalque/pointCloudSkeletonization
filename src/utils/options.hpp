#pragma once

#include<string>
#include<iostream>
#include <yaml-cpp/yaml.h>

struct options
{
    /* data */
    std::string path_input_obj;

    bool   visualization;
    bool   verbose;

    double laplacian_threshold;
    int iteration_time = 5;
    double termination_criteria = 0.01;
    double sl = 3;
    double WC = 1;
    double sample_radius = 0.002;
    double MAX_POSITION_CONSTRAINT_WEIGHT;
    double MAX_LAPLACIAN_CONSTRAINT_WEIGHT;
    int skeleton_editing = 0;

    double nodes_ratio;
    double edges_ratio;
    double graph_res;
    std::vector<double> nodes_color;
    std::vector<double> edges_color;

    void print(){
        std::cout << "list of the parameters:" << std::endl;
        std::cout << std::endl;
        std::cout << "*** IO files: ***" << std::endl;
        std::cout <<  "path_input_obj: " << path_input_obj << std::endl;
        std::cout << std::endl;
        std::cout << "*** General parameters ***" << std::endl;
        std::cout << "visualization: " << visualization << std::endl;
        std::cout << "verbose: " << verbose << std::endl;
        std::cout << std::endl;
        std::cout << "*** Skeletonization parameters ***" << std::endl;
        std::cout << "laplacian_threshold: " << laplacian_threshold << std::endl;
        std::cout << "iteration_time: " << iteration_time << std::endl;
        std::cout << "termination_criteria: " << termination_criteria << std::endl;
        std::cout << "sl: " << sl << std::endl;
        std::cout << "WC: " << WC << std::endl;
        std::cout << "sample_radius: " << sample_radius << std::endl;
        std::cout << "MAX_POSITION_CONSTRAINT_WEIGHT: " << MAX_POSITION_CONSTRAINT_WEIGHT << std::endl;
        std::cout << "MAX_LAPLACIAN_CONSTRAINT_WEIGHT: " << MAX_LAPLACIAN_CONSTRAINT_WEIGHT << std::endl;
        std::cout << "skeleton_editing: " << skeleton_editing << std::endl;
        std::cout << std::endl;
        std::cout << "*** Visualization parameters ***" << std::endl;
        std::cout << "nodes_ratio: " << nodes_ratio << std::endl; 
        std::cout << "edges_ratio: " << edges_ratio << std::endl; 
        std::cout << "graph_res: " << graph_res << std::endl; 
        std::cout << "nodes_color: [" << nodes_color[0] << ", " << nodes_color[1] << ", " << nodes_color[2] << "]" << std::endl; 
        std::cout << "edges_color: [" << edges_color[0] << ", " << edges_color[1] << ", " << edges_color[2] << "]" << std::endl; 
        std::cout << std::endl;
        std::cout << std::endl;
    }

    bool loadYAML(std::string config_file){
        YAML::Node config = YAML::LoadFile(config_file);
        
        // IO
        path_input_obj                  = config["io_files"]["input_obj"].as<std::string>();

        // general options
        visualization                   = config["general_params"]["visualization"].as<bool>();
        verbose                         = config["general_params"]["verbose"].as<bool>();

        // skeletonization
        laplacian_threshold             = config["skeletonization"]["laplacian_threshold"].as<double>();
        iteration_time                  = config["skeletonization"]["iteration_time"].as<int>();
        termination_criteria            = config["skeletonization"]["termination_criteria"].as<double>();
        sl                              = config["skeletonization"]["sl"].as<double>();
        WC                              = config["skeletonization"]["WC"].as<double>();
        sample_radius                   = config["skeletonization"]["sample_radius"].as<double>();
        MAX_POSITION_CONSTRAINT_WEIGHT  = config["skeletonization"]["MAX_POSITION_CONSTRAINT_WEIGHT"].as<double>();
        MAX_LAPLACIAN_CONSTRAINT_WEIGHT = config["skeletonization"]["MAX_LAPLACIAN_CONSTRAINT_WEIGHT"].as<double>();
        skeleton_editing                = config["skeletonization"]["skeleton_editing"].as<int>();

        // visualization parameters
        nodes_ratio                     = config["visualization_params"]["nodes_ratio"].as<double>();
        edges_ratio                     = config["visualization_params"]["edges_ratio"].as<double>();
        graph_res                       = config["visualization_params"]["graph_res"].as<double>();
        nodes_color                     = config["visualization_params"]["nodes_color"].as<std::vector<double>>();
        edges_color                     = config["visualization_params"]["edges_color"].as<std::vector<double>>();

        if (verbose)
            print();
    }
};
