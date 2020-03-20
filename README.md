# Automatic 2D Floorplan CAD Generation from 3D Point Clouds
Created by Uuganbayar Gankhuyag and Ji-Hyeong Han from Seoul National University of Science and Technology.

## Introduction

This repository is code release for our paper (here).

In the Architecture, Engineering, and Construction (AEC) industry, creating an indoor model of existing
buildings has been a challenging task since the building information modeling (BIM) was introduced.
Since the process of BIM is largely manual and implies the great possibility of error, the
automated creation of indoor models is a still ongoing research. In this paper, we propose fully automated
method to generate 2D floorplan CAD from 3D point clouds. The proposed method consists of
two main parts. The first one is detecting planes in the buildings, such as walls, floors, and ceilings,
from unstructured 3D point clouds and classifying them based on the assumption of Manhattan-World
(MW). The other one is generating 3D BIM in Industry Foundation Classes (IFC) format and 2D floorplan
CAD using the proposed line detection algorithm. We experiment the proposed method on the 3D
point clouds data from university building and evaluate the geometry quality of wall reconstruction.

## Requirements

PCL 1.2, OpenCV 4, CMake, last version of ifcopenshell and Python 3

## Run demo

### Detect Wall

```
mkdir build
cd build
make 

./floorplan
```

### Create IFC file

```
python create_ifc.py
```

### Create 2D floorplan CAD

copy `bin/IfcConvert` to `build/out/bim`

```
./IfcConvert out.ifc out.svg --include --entities IfcWall --bounds 512x512
```
