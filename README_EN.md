# Point Cloud to CAD 3D Model System

English | [简体中文](README.md)

## Project Overview

This is a point cloud to CAD 3D model conversion system based on C++ and Qt, supporting interactive processing, visualization-based reconstruction, and CAD model export.

## System Features

### Core Features
- **Data Import/Export**: Support for multiple point cloud formats (.ply, .obj, .xyz, .pcd, .las) and CAD model formats (.stl, .obj, .step, .iges)
- **Batch Processing**: Support for batch import of multiple point cloud files with batch processing and export capabilities
- **Point Cloud Preprocessing**: Statistical filtering, radius filtering, voxel grid downsampling, uniform sampling, random sampling, and more
- **Point Cloud Segmentation**: Plane segmentation, Euclidean clustering, region growing, RGB region growing, and more
- **Point Cloud Registration**: ICP, GICP, SAC-IA, NDT registration algorithms
- **Feature Extraction**: FPFH, SHOT feature extraction algorithms
- **Surface Reconstruction**: Poisson reconstruction, greedy triangulation algorithms
- **Model Optimization**: Mesh simplification, smoothing algorithms
- **Measurement Tools**: Distance, angle, area, and volume measurement capabilities
- **Project Management**: Create, open, and save project files
- **Logging**: Operation log window for real-time recording of system operations and error messages
- **Interactive Visualization**: Real-time visualization of point cloud data and 3D models with view control, zoom, and rotation

### User Interface
- **Menu Bar**: File, Edit, View, Tools, Help menus with project management, batch processing, and measurement tool submenus
- **Toolbar**: Quick access buttons for common operations
- **3D View**: Display point cloud data and 3D models with multiple rendering modes and multi-view layout support
- **Sidebar**: Data management, parameter settings, and tool options panels
- **Status Bar**: System status, processing progress, coordinate information, and zoom level
- **Log Window**: Real-time display of operation records and error messages (toggleable via View menu)

## Project Structure

```
pointcloud2cad/
├── src/                      # Source code directory
│   ├── algorithm/            # Algorithm module
│   │   ├── PointCloudProcessor.h
│   │   └── PointCloudProcessor.cpp
│   ├── data/                 # Data management module
│   │   ├── DataManager.h
│   │   ├── DataManager.cpp
│   │   ├── IOManager.h
│   │   └── IOManager.cpp
│   ├── visualization/        # Visualization module
│   │   ├── VisualizationWidget.h
│   │   └── VisualizationWidget.cpp
│   ├── ui/                   # User interface module
│   │   ├── MainWindow.h
│   │   └── MainWindow.cpp
│   ├── core/                 # Core module
│   ├── utils/                # Utility module
│   └── main.cpp              # Main function
├── CMakeLists.txt            # CMake configuration file
├── LICENSE                   # License file
└── README.md                 # Project documentation
```

## Tech Stack

- **C++17**: Core programming language
- **Qt 5.15+**: Graphical user interface framework
- **PCL 1.12+**: Point Cloud Library, including features, segmentation, and registration modules
- **CGAL 5.4+**: Computational Geometry Algorithms Library
- **OpenGL**: Graphics rendering API
- **Assimp**: 3D model import/export library
- **Eigen3**: Linear algebra library
- **Boost**: General-purpose utility library
- **CMake**: Build system

## Project Architecture

This project adopts a layered architecture design:

### 1. Data Layer
- **DataManager**: Manages point cloud data and 3D model data, providing add, delete, and query operations
- **IOManager**: Handles file import/export with support for multiple file formats and batch processing

### 2. Core Processing Layer
- **PointCloudProcessor**: Implements core point cloud processing algorithms, including:
  - Preprocessing: Statistical filtering, radius filtering, voxel grid downsampling, uniform sampling, random sampling
  - Feature extraction: FPFH, SHOT
  - Segmentation: Plane segmentation, Euclidean clustering, region growing, RGB region growing
  - Registration: ICP, GICP, SAC-IA, NDT
  - Reconstruction: Poisson reconstruction, greedy triangulation
  - Optimization: Moving least squares smoothing
  - Analysis: Centroid calculation, bounding box calculation

### 3. Visualization Layer
- **VisualizationWidget**: OpenGL rendering of point cloud data and 3D models with interactive operations and multi-view layout support

### 4. User Interface Layer
- **MainWindow**: Main window class containing menu bar, toolbar, 3D view, and sidebar components
- **Measurement Tools**: Distance, angle, area, and volume measurement functionality
- **Project Management**: Create, open, and save project files
- **Logging System**: Record and display operation logs

### 5. Utility Layer
- **Various Utility Classes**: Provide auxiliary functions such as parameter settings and progress display

## Build Environment

### Hardware Requirements
- CPU: At least 4 cores, 8+ cores recommended
- RAM: At least 8GB, 16GB+ recommended
- GPU: OpenGL 4.5+ support, dedicated GPU recommended

### Software Requirements
- Operating System: Windows 10/11 64-bit
- Compiler: Visual Studio 2019+ or MinGW 8.0+
- CMake: 3.16+
- Qt: 5.15+ (with Widgets, Core, Gui modules)
- PCL: 1.12+ (with common, io, filters, segmentation, registration, surface, features modules)
- CGAL: 5.4+
- Assimp: 5.1+
- Eigen3: 3.3+
- Boost: 1.70+

## Build Instructions

### Building with Visual Studio
1. Install all dependencies
2. Clone the project repository
3. Create a build directory and navigate into it
4. Run CMake to generate Visual Studio project:
   ```bash
   cmake .. -G "Visual Studio 17 2022" -A x64
   ```
5. Open the generated .sln file and build the project
6. Run the generated executable

### Building with MinGW
1. Install all dependencies
2. Clone the project repository
3. Create a build directory and navigate into it
4. Run CMake to generate MinGW Makefiles:
   ```bash
   cmake .. -G "MinGW Makefiles"
   ```
5. Run mingw32-make to build the project:
   ```bash
   mingw32-make
   ```
6. Run the generated executable

## Usage Instructions

### Basic Operations
1. **Import Point Cloud**: Click "File" -> "Open", select point cloud file (.ply, .obj, .xyz, .pcd, .las)
2. **Batch Import**: Click "Tools" -> "Batch Processing" -> "Batch Import", select multiple point cloud files
3. **Point Cloud Filtering**: Click the "Filter" button in the left tool panel to apply statistical filtering
4. **Point Cloud Segmentation**: Click the "Segment" button to perform Euclidean clustering segmentation
5. **Point Cloud Registration**: Click the "Register" button to align multiple point clouds
6. **Point Cloud Reconstruction**: Click the "Reconstruct" button to perform Poisson surface reconstruction
7. **Model Optimization**: Click the "Optimize" button to optimize the current model
8. **Export CAD Model**: Click "File" -> "Export", select CAD model format (.stl, .obj, .step, .iges)
9. **Batch Export**: Click "Tools" -> "Batch Processing" -> "Batch Export", select export directory

### Project Management
1. **New Project**: Click "File" -> "Project" -> "New Project"
2. **Open Project**: Click "File" -> "Project" -> "Open Project", select project file (.pc2cad)
3. **Save Project**: Click "File" -> "Project" -> "Save Project", select save path

### Measurement Tools
1. **Distance Measurement**: Click "Tools" -> "Measurement Tools" -> "Distance", select two points in the 3D view
2. **Angle Measurement**: Click "Tools" -> "Measurement Tools" -> "Angle", select three points in the 3D view
3. **Area Measurement**: Click "Tools" -> "Measurement Tools" -> "Area", select multiple points in the 3D view
4. **Volume Measurement**: Click "Tools" -> "Measurement Tools" -> "Volume", select a closed mesh model

### Visualization Controls
- **Rotate View**: Hold left mouse button and drag
- **Zoom View**: Use mouse scroll wheel
- **Pan View**: Hold middle mouse button and drag
- **Preset Views**: Click "View" -> "Preset Views", select preset view (top, bottom, front, back, left, right, isometric)
- **Multi-view Layout**: Click "View" -> "Multi-view Layout", select layout mode (single, dual, quad view)
- **View Controls**: Click "View" to use reset view, zoom in, zoom out, and other functions

### Logging
- **Show Operation Log**: Click "View" -> "Show Operation Log" to open the log window
- **View Operation Records**: Check system operation records and error messages in the log window

### Parameter Settings
- **Filter Parameters**: Adjust statistical filter neighbor count and standard deviation threshold in the right parameter panel
- **Reconstruction Parameters**: Adjust Poisson reconstruction depth, point weight, and scale in the right parameter panel
- **Rendering Parameters**: Adjust point size, line width, and rendering mode in the right parameter panel

## Sample Data

### Point Cloud Samples
- **bunny.ply**: Bunny model point cloud data
- **dragon.ply**: Dragon model point cloud data
- **teapot.ply**: Teapot model point cloud data

### CAD Model Samples
- **bunny.stl**: Bunny model in STL format
- **dragon.obj**: Dragon model in OBJ format
- **teapot.step**: Teapot model in STEP format

## FAQ

### Problem: Cannot load point cloud file
**Solutions**:
1. Check if the file format is supported
2. Check if the file path is correct
3. Check if the file is corrupted

### Problem: Poor point cloud filtering results
**Solutions**:
1. Adjust filter parameters, increase neighbor count or decrease standard deviation threshold
2. Try other filtering algorithms such as radius filtering

### Problem: Poor reconstructed model quality
**Solutions**:
1. Adjust reconstruction parameters, increase reconstruction tree depth
2. Apply better preprocessing to original point cloud, such as smoothing and denoising
3. Try other reconstruction algorithms such as greedy triangulation

### Problem: Visualization window stuttering
**Solutions**:
1. Downsample the point cloud data to reduce point count
2. Reduce point size or line width
3. Disable unnecessary rendering effects

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

- **Email**: 980139383@qq.com
- **GitHub**: https://github.com/EricXiaoHuan/pointclound2cad

## Acknowledgments

This project uses the following open-source libraries:
- [PCL (Point Cloud Library)](https://pointclouds.org/)
- [CGAL (Computational Geometry Algorithms Library)](https://www.cgal.org/)
- [Qt](https://www.qt.io/)
- [Assimp](https://www.assimp.org/)
- [Eigen](https://eigen.tuxfamily.org/)
- [Boost](https://www.boost.org/)

Thanks to all developers and contributors of these open-source libraries!
