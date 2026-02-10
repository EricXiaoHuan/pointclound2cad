#include "DataManager.h"

/**
 * @brief 数据项基类构造函数
 * @param name 数据名称
 * @param type 数据类型
 */
DataItem::DataItem(const std::string& name, DataType type)
    : name(name), type(type), path("")
{
}

/**
 * @brief 数据项基类析构函数
 */
DataItem::~DataItem()
{
}

/**
 * @brief 获取数据名称
 * @return 数据名称
 */
std::string DataItem::getName() const
{
    return name;
}

/**
 * @brief 设置数据名称
 * @param name 数据名称
 */
void DataItem::setName(const std::string& name)
{
    this->name = name;
}

/**
 * @brief 获取数据类型
 * @return 数据类型
 */
DataType DataItem::getType() const
{
    return type;
}

/**
 * @brief 获取数据路径
 * @return 数据路径
 */
std::string DataItem::getPath() const
{
    return path;
}

/**
 * @brief 设置数据路径
 * @param path 数据路径
 */
void DataItem::setPath(const std::string& path)
{
    this->path = path;
}

/**
 * @brief 点云数据项构造函数
 * @param name 数据名称
 */
PointCloudItem::PointCloudItem(const std::string& name)
    : DataItem(name, DataType::POINT_CLOUD)
{
    pointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
}

/**
 * @brief 点云数据项析构函数
 */
PointCloudItem::~PointCloudItem()
{
}

/**
 * @brief 获取点云数据
 * @return 点云数据
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudItem::getPointCloud()
{
    return pointCloud;
}

/**
 * @brief 设置点云数据
 * @param cloud 点云数据
 */
void PointCloudItem::setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pointCloud = cloud;
}

/**
 * @brief 克隆数据项
 * @return 克隆的数据项
 */
std::shared_ptr<DataItem> PointCloudItem::clone() const
{
    std::shared_ptr<PointCloudItem> clonedItem = std::make_shared<PointCloudItem>(name);
    clonedItem->setPath(path);
    
    // 克隆点云数据
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clonedCloud = 
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    *clonedCloud = *pointCloud;
    clonedItem->setPointCloud(clonedCloud);
    
    return clonedItem;
}

/**
 * @brief 网格模型数据项构造函数
 * @param name 数据名称
 */
MeshModelItem::MeshModelItem(const std::string& name)
    : DataItem(name, DataType::MESH_MODEL)
{
    mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
}

/**
 * @brief 网格模型数据项析构函数
 */
MeshModelItem::~MeshModelItem()
{
}

/**
 * @brief 获取网格模型
 * @return 网格模型
 */
pcl::PolygonMesh::Ptr MeshModelItem::getMesh()
{
    return mesh;
}

/**
 * @brief 设置网格模型
 * @param mesh 网格模型
 */
void MeshModelItem::setMesh(pcl::PolygonMesh::Ptr mesh)
{
    this->mesh = mesh;
}

/**
 * @brief 克隆数据项
 * @return 克隆的数据项
 */
std::shared_ptr<DataItem> MeshModelItem::clone() const
{
    std::shared_ptr<MeshModelItem> clonedItem = std::make_shared<MeshModelItem>(name);
    clonedItem->setPath(path);
    
    // 克隆网格模型
    pcl::PolygonMesh::Ptr clonedMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
    *clonedMesh = *mesh;
    clonedItem->setMesh(clonedMesh);
    
    return clonedItem;
}

/**
 * @brief 数据管理器构造函数
 */
DataManager::DataManager()
    : currentIndex(0)
{
}

/**
 * @brief 数据管理器析构函数
 */
DataManager::~DataManager()
{
    clearItems();
}

/**
 * @brief 获取数据管理器实例
 * @return 数据管理器实例
 */
DataManager& DataManager::getInstance()
{
    static DataManager instance;
    return instance;
}

/**
 * @brief 添加数据项
 * @param item 数据项
 * @return 数据项ID
 */
size_t DataManager::addItem(std::shared_ptr<DataItem> item)
{
    items.push_back(item);
    currentIndex = items.size() - 1;
    return currentIndex;
}

/**
 * @brief 获取数据项
 * @param index 数据项索引
 * @return 数据项
 */
std::shared_ptr<DataItem> DataManager::getItem(size_t index)
{
    if (index < items.size()) {
        return items[index];
    }
    return nullptr;
}

/**
 * @brief 获取点云数据项
 * @param index 数据项索引
 * @return 点云数据项
 */
std::shared_ptr<PointCloudItem> DataManager::getPointCloudItem(size_t index)
{
    if (index < items.size() && items[index]->getType() == DataType::POINT_CLOUD) {
        return std::dynamic_pointer_cast<PointCloudItem>(items[index]);
    }
    return nullptr;
}

/**
 * @brief 获取网格模型数据项
 * @param index 数据项索引
 * @return 网格模型数据项
 */
std::shared_ptr<MeshModelItem> DataManager::getMeshModelItem(size_t index)
{
    if (index < items.size() && items[index]->getType() == DataType::MESH_MODEL) {
        return std::dynamic_pointer_cast<MeshModelItem>(items[index]);
    }
    return nullptr;
}

/**
 * @brief 移除数据项
 * @param index 数据项索引
 * @return 是否移除成功
 */
bool DataManager::removeItem(size_t index)
{
    if (index < items.size()) {
        items.erase(items.begin() + index);
        if (currentIndex >= items.size() && !items.empty()) {
            currentIndex = items.size() - 1;
        } else if (items.empty()) {
            currentIndex = 0;
        }
        return true;
    }
    return false;
}

/**
 * @brief 清空所有数据项
 */
void DataManager::clearItems()
{
    items.clear();
    currentIndex = 0;
}

/**
 * @brief 获取数据项数量
 * @return 数据项数量
 */
size_t DataManager::getItemCount() const
{
    return items.size();
}

/**
 * @brief 获取当前选中的数据项索引
 * @return 当前选中的数据项索引
 */
size_t DataManager::getCurrentItemIndex() const
{
    return currentIndex;
}

/**
 * @brief 设置当前选中的数据项索引
 * @param index 数据项索引
 */
void DataManager::setCurrentItemIndex(size_t index)
{
    if (index < items.size()) {
        currentIndex = index;
    }
}

/**
 * @brief 获取当前选中的数据项
 * @return 当前选中的数据项
 */
std::shared_ptr<DataItem> DataManager::getCurrentItem()
{
    if (!items.empty() && currentIndex < items.size()) {
        return items[currentIndex];
    }
    return nullptr;
}