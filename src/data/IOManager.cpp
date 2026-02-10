#include "IOManager.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <iostream>
#include <fstream>

/**
 * @brief 导入导出管理器构造函数
 */
IOManager::IOManager()
{
    // 初始化支持的文件格式
    pointCloudFormats = {"ply", "obj", "pcd", "xyz", "las"};
    meshFormats = {"stl", "obj", "ply", "vtk"};
    cadFormats = {"step", "iges", "stl", "obj"};
}

/**
 * @brief 导入导出管理器析构函数
 */
IOManager::~IOManager()
{
}

/**
 * @brief 获取导入导出管理器实例
 * @return 导入导出管理器实例
 */
IOManager& IOManager::getInstance()
{
    static IOManager instance;
    return instance;
}

/**
 * @brief 获取文件扩展名
 * @param filePath 文件路径
 * @return 文件扩展名
 */
std::string IOManager::getFileExtension(const std::string& filePath)
{
    size_t dotPos = filePath.find_last_of('.');
    if (dotPos != std::string::npos) {
        return filePath.substr(dotPos + 1);
    }
    return "";
}

/**
 * @brief 导入点云数据
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导入成功
 */
bool IOManager::importPointCloud(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    std::string extension = getFileExtension(filePath);
    
    if (extension == "ply") {
        return importPLY(filePath, cloud);
    } else if (extension == "obj") {
        return importOBJ(filePath, cloud);
    } else if (extension == "pcd") {
        return importPCD(filePath, cloud);
    } else if (extension == "xyz") {
        return importXYZ(filePath, cloud);
    } else {
        std::cerr << "不支持的点云文件格式: " << extension << std::endl;
        return false;
    }
}

/**
 * @brief 导出点云数据
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导出成功
 */
bool IOManager::exportPointCloud(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    std::string extension = getFileExtension(filePath);
    
    if (extension == "ply") {
        return exportPLY(filePath, cloud);
    } else if (extension == "obj") {
        return exportOBJ(filePath, cloud);
    } else if (extension == "pcd") {
        return exportPCD(filePath, cloud);
    } else {
        std::cerr << "不支持的点云文件格式: " << extension << std::endl;
        return false;
    }
}

/**
 * @brief 导入网格模型
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导入成功
 */
bool IOManager::importMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    std::string extension = getFileExtension(filePath);
    
    if (extension == "stl") {
        return importSTL(filePath, mesh);
    } else if (extension == "obj") {
        return importOBJMesh(filePath, mesh);
    } else {
        std::cerr << "不支持的网格模型文件格式: " << extension << std::endl;
        return false;
    }
}

/**
 * @brief 导出网格模型
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导出成功
 */
bool IOManager::exportMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    std::string extension = getFileExtension(filePath);
    
    if (extension == "stl") {
        return exportSTL(filePath, mesh);
    } else if (extension == "obj") {
        return exportOBJMesh(filePath, mesh);
    } else if (extension == "step") {
        return exportSTEP(filePath, mesh);
    } else if (extension == "iges") {
        return exportIGES(filePath, mesh);
    } else {
        std::cerr << "不支持的网格模型文件格式: " << extension << std::endl;
        return false;
    }
}

/**
 * @brief 获取支持的点云文件格式
 * @return 支持的点云文件格式列表
 */
std::vector<std::string> IOManager::getSupportedPointCloudFormats()
{
    return pointCloudFormats;
}

/**
 * @brief 获取支持的网格模型文件格式
 * @return 支持的网格模型文件格式列表
 */
std::vector<std::string> IOManager::getSupportedMeshFormats()
{
    return meshFormats;
}

/**
 * @brief 获取支持的CAD模型文件格式
 * @return 支持的CAD模型文件格式列表
 */
std::vector<std::string> IOManager::getSupportedCADFormats()
{
    return cadFormats;
}

/**
 * @brief 导入PLY格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导入成功
 */
bool IOManager::importPLY(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filePath, *rgbCloud) == -1) {
        std::cerr << "无法加载PLY文件: " << filePath << std::endl;
        return false;
    }
    
    // 转换为PointXYZRGBNormal格式
    pcl::copyPointCloud(*rgbCloud, *cloud);
    return true;
}

/**
 * @brief 导入OBJ格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导入成功
 */
bool IOManager::importOBJ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadOBJFile(filePath, *xyzCloud) == -1) {
        std::cerr << "无法加载OBJ文件: " << filePath << std::endl;
        return false;
    }
    
    // 转换为PointXYZRGBNormal格式
    pcl::copyPointCloud(*xyzCloud, *cloud);
    return true;
}

/**
 * @brief 导入PCD格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导入成功
 */
bool IOManager::importPCD(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filePath, *cloud) == -1) {
        std::cerr << "无法加载PCD文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导入XYZ格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导入成功
 */
bool IOManager::importXYZ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "无法打开XYZ文件: " << filePath << std::endl;
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z) {
            pcl::PointXYZRGBNormal point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->push_back(point);
        }
    }
    
    file.close();
    return true;
}

/**
 * @brief 导出PLY格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导出成功
 */
bool IOManager::exportPLY(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    if (pcl::io::savePLYFileBinary<pcl::PointXYZRGBNormal>(filePath, *cloud) == -1) {
        std::cerr << "无法保存PLY文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导出OBJ格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导出成功
 */
bool IOManager::exportOBJ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *xyzCloud);
    
    if (pcl::io::saveOBJFile(filePath, *xyzCloud) == -1) {
        std::cerr << "无法保存OBJ文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导出PCD格式点云
 * @param filePath 文件路径
 * @param cloud 点云数据
 * @return 是否导出成功
 */
bool IOManager::exportPCD(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    if (pcl::io::savePCDFileBinary<pcl::PointXYZRGBNormal>(filePath, *cloud) == -1) {
        std::cerr << "无法保存PCD文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导入STL格式网格
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导入成功
 */
bool IOManager::importSTL(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filePath.c_str());
    reader->Update();
    
    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
    pcl::io::vtkPolyDataToPolygonMesh(*polydata, *mesh);
    
    return true;
}

/**
 * @brief 导入OBJ格式网格
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导入成功
 */
bool IOManager::importOBJMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    if (pcl::io::loadOBJFile(filePath, *mesh) == -1) {
        std::cerr << "无法加载OBJ网格文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导出STL格式网格
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导出成功
 */
bool IOManager::exportSTL(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::polygonMeshToVTKPolyData(*mesh, *polydata);
    
    vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
    writer->SetFileName(filePath.c_str());
    writer->SetInputData(polydata);
    writer->Write();
    
    return true;
}

/**
 * @brief 导出OBJ格式网格
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导出成功
 */
bool IOManager::exportOBJMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    if (pcl::io::saveOBJFile(filePath, *mesh) == -1) {
        std::cerr << "无法保存OBJ网格文件: " << filePath << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 导出STEP格式CAD模型
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导出成功
 */
bool IOManager::exportSTEP(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    // TODO: 实现STEP格式导出
    std::cerr << "STEP格式导出功能尚未实现" << std::endl;
    return false;
}

/**
 * @brief 导出IGES格式CAD模型
 * @param filePath 文件路径
 * @param mesh 网格模型
 * @return 是否导出成功
 */
bool IOManager::exportIGES(const std::string& filePath, pcl::PolygonMesh::Ptr mesh)
{
    // TODO: 实现IGES格式导出
    std::cerr << "IGES格式导出功能尚未实现" << std::endl;
    return false;
}