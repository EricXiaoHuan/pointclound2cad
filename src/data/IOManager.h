#ifndef IOMANAGER_H
#define IOMANAGER_H

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/polygonmesh.h>

/**
 * @brief 导入导出管理器
 * 负责处理不同格式的点云数据和CAD模型的读写操作
 */
class IOManager {
public:
    /**
     * @brief 获取导入导出管理器实例
     * @return 导入导出管理器实例
     */
    static IOManager& getInstance();
    
    /**
     * @brief 导入点云数据
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导入成功
     */
    bool importPointCloud(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导出点云数据
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导出成功
     */
    bool exportPointCloud(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导入网格模型
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导入成功
     */
    bool importMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导出网格模型
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导出成功
     */
    bool exportMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 获取支持的点云文件格式
     * @return 支持的点云文件格式列表
     */
    std::vector<std::string> getSupportedPointCloudFormats();
    
    /**
     * @brief 获取支持的网格模型文件格式
     * @return 支持的网格模型文件格式列表
     */
    std::vector<std::string> getSupportedMeshFormats();
    
    /**
     * @brief 获取支持的CAD模型文件格式
     * @return 支持的CAD模型文件格式列表
     */
    std::vector<std::string> getSupportedCADFormats();

private:
    /**
     * @brief 构造函数
     */
    IOManager();
    
    /**
     * @brief 析构函数
     */
    ~IOManager();
    
    /**
     * @brief 获取文件扩展名
     * @param filePath 文件路径
     * @return 文件扩展名
     */
    std::string getFileExtension(const std::string& filePath);
    
    /**
     * @brief 导入PLY格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导入成功
     */
    bool importPLY(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导入OBJ格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导入成功
     */
    bool importOBJ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导入PCD格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导入成功
     */
    bool importPCD(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导入XYZ格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导入成功
     */
    bool importXYZ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导出PLY格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导出成功
     */
    bool exportPLY(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导出OBJ格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导出成功
     */
    bool exportOBJ(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导出PCD格式点云
     * @param filePath 文件路径
     * @param cloud 点云数据
     * @return 是否导出成功
     */
    bool exportPCD(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 导入STL格式网格
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导入成功
     */
    bool importSTL(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导入OBJ格式网格
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导入成功
     */
    bool importOBJMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导出STL格式网格
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导出成功
     */
    bool exportSTL(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导出OBJ格式网格
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导出成功
     */
    bool exportOBJMesh(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导出STEP格式CAD模型
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导出成功
     */
    bool exportSTEP(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 导出IGES格式CAD模型
     * @param filePath 文件路径
     * @param mesh 网格模型
     * @return 是否导出成功
     */
    bool exportIGES(const std::string& filePath, pcl::PolygonMesh::Ptr mesh);

private:
    std::vector<std::string> pointCloudFormats;  // 支持的点云文件格式
    std::vector<std::string> meshFormats;  // 支持的网格模型文件格式
    std::vector<std::string> cadFormats;  // 支持的CAD模型文件格式
};

#endif // IOMANAGER_H