# C++ implementation of "Point Cloud Skeletons via Laplacian-Based Contraction"

## Differences with the original paper
* Differente implementation of the laplacian
* Simpler skeleton trimming (which works only for shape of [genus](https://en.wikipedia.org/wiki/Genus_(mathematics)) 0)

## Dependencies
1. LIBIGL (for visualization and files loading)

## Installation instruction
first you need to edit the file located in ./src/cmake/FindLIBIGL.cmake to edit the location of libigl

type into the console:

* mkdir build
* cd build
* cmake ../src
* make

## example

![skeletonization](https://github.com/rFalque/pointCloudSkeletonization/raw/master/images/skeletonization.png "example of point cloud skeletonization through Laplacian contraction")

## Link to the original papers:
1. [Point Cloud Skeletons via Laplacian-Based Contraction](https://gfx.uvic.ca/pubs/2010/cao_smi10/paper.pdf)
2. [Skeleton Extraction by Mesh Contraction](http://visgraph.cse.ust.hk/projects/skeleton/skeleton_sig08.pdf)

## Other implementations:
* Matlab implementation: [https://github.com/ataiya/cloudcontr](https://github.com/ataiya/cloudcontr)
* C++ implementation for triangular meshes: [CGAL](https://doc.cgal.org/latest/Surface_mesh_skeletonization/index.html#Chapter_3D_Surface_mesh_skeletonization)
