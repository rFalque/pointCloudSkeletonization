# C++ implementation of "Point Cloud Skeletons via Laplacian-Based Contraction"

## What is in this repository?
This repository contains the C++ implementation of the point cloud skeletonization  based on the Laplacian-based contraction [[1]](#link-to-the-original-papers). In short, a point cloud is used as an input, a Laplacian operator is then built based on the concept of a one-ring neighbourhood. This operator is then used for contracting the point cloud. A skeleton is then built using farthest distance sampling with either a k-NN or a radius search.

Additionally, mesh skeletonization using the same method with the Laplacian defined on the mesh is also provided.

## Dependencies

### Standard dev tools
On Ubuntu, you need to install the following packages:
```bash
sudo apt-get update
sudo apt-get install git build-essential cmake libx11-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxi-dev libxmu-dev libblas-dev libxinerama-dev libxcursor-dev libeigen3-dev libyaml-cpp-dev python-matplotlib python-numpy python2.7-dev
```

### Specific dependencies
The dependencies are specified for information only, however they should all be installed automatically.
1. [Eigen](https://eigen.tuxfamily.org/) (downloaded through sudo apt-get install)
2. [yaml-cpp](https://github.com/jbeder/yaml-cpp) (downloaded through sudo apt-get install)
3. [libGraphCpp](https://github.com/rFalque/libGraphCpp) (automatically download from the cmake file)
4. [polyscope](http://polyscope.run/) (downloaded as part of libGraphCpp)

Optional dependencies:

5. [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

## Installation instruction
To build, type into the console:
```bash
git clone https://github.com/rFalque/pointCloudSkeletonization.git
cd pointCloudSkeletonization
mkdir build
cd build
cmake ..
make -j3
```

## Run the code

> :information_source: **Info**:  The input files and the skeleton trimming method can be changed through the config.yaml file.

To run the sample, then just type:
```bash
./pointcloud_skeleton_extraction
```

![skeletonization](https://github.com/rFalque/pointCloudSkeletonization/raw/master/images/skeletonization.png "example of point cloud skeletonization through Laplacian contraction")

## todo
* add parallelization
* stop the contraction once the shrinking reach a certain stage, otherwise the skeleton diverges at some point

## Differences with the original paper [[1]](#link-to-the-original-papers)
* Different implementation of the laplacian
* The nodes are generated using the farthest sampling algorithm (using either a sphere or k-NN to find neighbours). The k-NN sampling allows to have nodes generated with respect to the points density
* Several skeleton trimming is available depending of the required skeleton type

## Link to the original papers
1. [Point Cloud Skeletons via Laplacian-Based Contraction](https://gfx.uvic.ca/pubs/2010/cao_smi10/paper.pdf)
2. [Skeleton Extraction by Mesh Contraction](http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf)

## Other implementations
* Matlab implementation: [https://github.com/ataiya/cloudcontr](https://github.com/ataiya/cloudcontr)
* C++ implementation for triangular meshes: [CGAL](https://doc.cgal.org/latest/Surface_mesh_skeletonization/index.html#Chapter_3D_Surface_mesh_skeletonization)

## Used in paper
```
@ARTICLE{8968326,
  author={Wu, Lan and Falque, Raphael and Perez-Puchalt, Victor and Liu, Liyang and Pietroni, Nico and Vidal-Calleja, Teresa},
  journal={IEEE Robotics and Automation Letters}, 
  title={Skeleton-Based Conditionally Independent Gaussian Process Implicit Surfaces for Fusion in Sparse to Dense 3D Reconstruction}, 
  year={2020},
  volume={5},
  number={2},
  pages={1532-1539},
  doi={10.1109/LRA.2020.2969175}}
```
