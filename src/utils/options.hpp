#pragma once

#include<string>
#include<iostream>
#include <yaml-cpp/yaml.h>

struct options
{
    /* data */
    std::string path_pairwise_correspondence;
    std::string path_input_file;
    std::string path_output_file;
    std::string path_input_obj;
    std::string path_graph_obj;

    bool   visualization;
    bool   verbose;
    bool   graph_provided;

    double nodes_ratio;
    double edges_ratio;
    double graph_res;
    std::vector<double> nodes_color;
    std::vector<double> edges_color;

    void print(){
        std::cout << "list of the parameters:" << std::endl;
        std::cout << std::endl;
        std::cout << "*** IO files: ***" << std::endl;
        if (graph_provided)
        {
            std::cout <<  "path_input_obj: " << path_input_obj << std::endl;
            std::cout <<  "path_graph_obj: " << path_graph_obj << std::endl;
        }
        else
        {
            std::cout <<  "path_input_file: " << path_input_file << std::endl;
        }
        std::cout << "path_output_file: " << path_output_file << std::endl;
        std::cout << "path_pairwise_correspondence: " << path_pairwise_correspondence << std::endl;
        std::cout << std::endl;
        std::cout << "*** General parameters ***" << std::endl;
        std::cout << "visualization: " << visualization << std::endl;
        std::cout << "verbose: " << verbose << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
    }

    bool loadYAML(std::string config_file){
        YAML::Node config = YAML::LoadFile(config_file);
        
        // IO
        path_input_file                 = config["io_files"]["input_ply"].as<std::string>();
        path_output_file                = config["io_files"]["output_ply"].as<std::string>();
        path_input_obj                  = config["io_files"]["input_obj"].as<std::string>();
        path_graph_obj                  = config["io_files"]["graph_obj"].as<std::string>();
        path_pairwise_correspondence    = config["io_files"]["pointwise_correspondence"].as<std::string>();

        // general options
        visualization                   = config["general_params"]["visualization"].as<bool>();
        verbose                         = config["general_params"]["verbose"].as<bool>();
        graph_provided                  = config["general_params"]["graph_provided"].as<bool>();

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
