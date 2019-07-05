# C++ implementation of "Point Cloud Skeletons via Laplacian-Based Contraction"

## Differences with the original paper
TBD.

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

![example](https://github.com/rFalque/pointCloudSkeletonization/raw/master/images/laplacian_contraction.png "example of Laplacian contraction")

## Link to the original paper:
[1] [Point Cloud Skeletons via Laplacian-Based Contraction](https://gfx.uvic.ca/pubs/2010/cao_smi10/paper.pdf)
