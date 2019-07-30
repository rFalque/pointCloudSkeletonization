# C++ implementation of "Point Cloud Skeletons via Laplacian-Based Contraction"

## todo:
* remove libigl dependency (and replace it with [polyscope](https://github.com/nmwsharp/polyscope)) or move it into the cmake file

## Differences with the original paper
* Differente implementation of the laplacian
* Simpler skeleton trimming (which works only for shape of [genus](https://en.wikipedia.org/wiki/Genus_(mathematics)) 0)

## Dependencies

### Standard dev tools
On Ubuntu, you need to install the following packages:
```bash
sudo apt-get install git
sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install libx11-dev
sudo apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev
sudo apt-get install libxrandr-dev
sudo apt-get install libxi-dev
sudo apt-get install libxmu-dev
sudo apt-get install libblas-dev
sudo apt-get install libxinerama-dev
sudo apt-get install libxcursor-dev
```

### Specific dependencies:
1. [LIBIGL](https://github.com/libigl/libigl/) (for visualization and files loading)
2. [libGraphCpp](https://github.com/rFalque/libGraphCpp) (automatically download from the cmake file)

to build libigl:
```bash
git clone https://github.com/libigl/libigl.git
cd libigl
mkdir build
cd build
cmake ..
make -j3
```

## Installation instruction

> ⚠️ **Warning**: You might have to update the path of libigl in the ./src/cmake/FindLIBIGL.cmake file

type into the console:
```bash
mkdir build
cd build
cmake ../src
make -j3
```

## example

![skeletonization](https://github.com/rFalque/pointCloudSkeletonization/raw/master/images/skeletonization.png "example of point cloud skeletonization through Laplacian contraction")

## Link to the original papers:
1. [Point Cloud Skeletons via Laplacian-Based Contraction](https://gfx.uvic.ca/pubs/2010/cao_smi10/paper.pdf)
2. [Skeleton Extraction by Mesh Contraction](http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf)

## Other implementations:
* Matlab implementation: [https://github.com/ataiya/cloudcontr](https://github.com/ataiya/cloudcontr)
* C++ implementation for triangular meshes: [CGAL](https://doc.cgal.org/latest/Surface_mesh_skeletonization/index.html#Chapter_3D_Surface_mesh_skeletonization)
