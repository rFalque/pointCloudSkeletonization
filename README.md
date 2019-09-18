# C++ implementation of "Point Cloud Skeletons via Laplacian-Based Contraction"

## todo:
* inspect the cases where the laplacian contraction produces a segmentation fault

## Differences with the original paper
* Differente implementation of the laplacian
* Allows to produce a set of correspondence with similar number of points associated to it
* Simpler skeleton trimming (which works only for shape of [genus](https://en.wikipedia.org/wiki/Genus_(mathematics)) 0)

## Dependencies

### Standard dev tools
On Ubuntu, you need to install the following packages:
```bash
sudo apt-get update
sudo apt-get install git build-essential cmake libx11-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxi-dev libxmu-dev libblas-dev libxinerama-dev libxcursor-dev libeigen3-dev libyaml-cpp-dev
```

### Specific dependencies:
1. [Eigen](https://eigen.tuxfamily.org/) (for visualization and files loading)
2. [libGraphCpp](https://github.com/rFalque/libGraphCpp) (automatically download from the cmake file)
3. [polyscope](http://polyscope.run/) (downloaded as part of libGraphCpp)

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

## example

> :information_source: **Info**:  The input files and the skeleton trimming method can be changed through the config.yaml file.

To run the sample, then just type:
```bash
./skeleton_extraction_bin
```

![skeletonization](https://github.com/rFalque/pointCloudSkeletonization/raw/master/images/skeletonization.png "example of point cloud skeletonization through Laplacian contraction")

## Link to the original papers:
1. [Point Cloud Skeletons via Laplacian-Based Contraction](https://gfx.uvic.ca/pubs/2010/cao_smi10/paper.pdf)
2. [Skeleton Extraction by Mesh Contraction](http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf)

## Other implementations:
* Matlab implementation: [https://github.com/ataiya/cloudcontr](https://github.com/ataiya/cloudcontr)
* C++ implementation for triangular meshes: [CGAL](https://doc.cgal.org/latest/Surface_mesh_skeletonization/index.html#Chapter_3D_Surface_mesh_skeletonization)
