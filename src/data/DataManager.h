#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <vector>
#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/polygonmesh.h>

/**
 * @brief 数据类型枚举
 */
enum class DataType {
    POINT_CLOUD,  // 点云数据
    MESH_MODEL,   // 网格模型
    CAD_MODEL     // CAD模型
};

/**
 * @brief 数据项基类
 */
class DataItem {
public:
    /**
     * @brief 构造函数
     * @param name 数据名称
     * @param type 数据类型
     */
    DataItem(const std::string& name, DataType type);
    
    /**
     * @brief 析构函数
     */
    virtual ~DataItem();
    
    /**
     * @brief 获取数据名称
     * @return 数据名称
     */
    std::string getName() const;
    
    /**
     * @brief 设置数据名称
     * @param name 数据名称
     */
    void setName(const std::string& name);
    
    /**
     * @brief 获取数据类型
     * @return 数据类型
     */
    DataType getType() const;
    
    /**
     * @brief 获取数据路径
     * @return 数据路径
     */
    std::string getPath() const;
    
    /**
     * @brief 设置数据路径
     * @param path 数据路径
     */
    void setPath(const std::string& path);
    
    /**
     * @brief 克隆数据项
     * @return 克隆的数据项
     */
    virtual std::shared_ptr<DataItem> clone() const = 0;

protected:
    std::string name;   // 数据名称
    DataType type;      // 数据类型
    std::string path;   // 数据路径
};

/**
 * @brief 点云数据项
 */
class PointCloudItem : public DataItem {
public:
    /**
     * @brief 构造函数
     * @param name 数据名称
     */
    PointCloudItem(const std::string& name);
    
    /**
     * @brief 析构函数
     */
    ~PointCloudItem() override;
    
    /**
     * @brief 获取点云数据
     * @return 点云数据
     */
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getPointCloud();
    
    /**
     * @brief 设置点云数据
     * @param cloud 点云数据
     */
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 克隆数据项
     * @return 克隆的数据项
     */
    std::shared_ptr<DataItem> clone() const override;

private:
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud;  // 点云数据
};

/**
 * @brief 网格模型数据项
 */
class MeshModelItem : public DataItem {
public:
    /**
     * @brief 构造函数
     * @param name 数据名称
     */
    MeshModelItem(const std::string& name);
    
    /**
     * @brief 析构函数
     */
    ~MeshModelItem() override;
    
    /**
     * @brief 获取网格模型
     * @return 网格模型
     */
    pcl::PolygonMesh::Ptr getMesh();
    
    /**
     * @brief 设置网格模型
     * @param mesh 网格模型
     */
    void setMesh(pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 克隆数据项
     * @return 克隆的数据项
     */
    std::shared_ptr<DataItem> clone() const override;

private:
    pcl::PolygonMesh::Ptr mesh;  // 网格模型
};

/**
 * @brief 数据管理器
 * 负责管理所有数据项，包括点云数据、网格模型等
 */
class DataManager {
public:
    /**
     * @brief 获取数据管理器实例
     * @return 数据管理器实例
     */
    static DataManager& getInstance();
    
    /**
     * @brief 添加数据项
     * @param item 数据项
     * @return 数据项ID
     */
    size_t addItem(std::shared_ptr<DataItem> item);
    
    /**
     * @brief 获取数据项
     * @param index 数据项索引
     * @return 数据项
     */
    std::shared_ptr<DataItem> getItem(size_t index);
    
    /**
     * @brief 获取点云数据项
     * @param index 数据项索引
     * @return 点云数据项
     */
    std::shared_ptr<PointCloudItem> getPointCloudItem(size_t index);
    
    /**
     * @brief 获取网格模型数据项
     * @param index 数据项索引
     * @return 网格模型数据项
     */
    std::shared_ptr<MeshModelItem> getMeshModelItem(size_t index);
    
    /**
     * @brief 移除数据项
     * @param index 数据项索引
     * @return 是否移除成功
     */
    bool removeItem(size_t index);
    
    /**
     * @brief 清空所有数据项
     */
    void clearItems();
    
    /**
     * @brief 获取数据项数量
     * @return 数据项数量
     */
    size_t getItemCount() const;
    
    /**
     * @brief 获取当前选中的数据项索引
     * @return 当前选中的数据项索引
     */
    size_t getCurrentItemIndex() const;
    
    /**
     * @brief 设置当前选中的数据项索引
     * @param index 数据项索引
     */
    void setCurrentItemIndex(size_t index);
    
    /**
     * @brief 获取当前选中的数据项
     * @return 当前选中的数据项
     */
    std::shared_ptr<DataItem> getCurrentItem();

private:
    /**
     * @brief 构造函数
     */
    DataManager();
    
    /**
     * @brief 析构函数
     */
    ~DataManager();

private:
    std::vector<std::shared_ptr<DataItem>> items;  // 数据项列表
    size_t currentIndex;  // 当前选中的数据项索引
};

#endif // DATAMANAGER_H