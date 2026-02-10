#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/3dsc.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sac_ia.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>

/**
 * @brief 点云处理器类
 * 实现点云数据的预处理、分割、配准、重建等功能
 */
class PointCloudProcessor {
public:
    /**
     * @brief 获取点云处理器实例
     * @return 点云处理器实例
     */
    static PointCloudProcessor& getInstance();
    
    /**
     * @brief 统计滤波
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param meanK 近邻点数量
     * @param stdDevMulThresh 标准差阈值
     */
    void statisticalFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                          int meanK = 50,
                          double stdDevMulThresh = 1.0);
    
    /**
     * @brief 半径滤波
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param radiusSearch 搜索半径
     * @param minPts 最小邻点数量
     */
    void radiusFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                     double radiusSearch = 0.01,
                     int minPts = 10);
    
    /**
     * @brief 体素网格下采样
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param leafSize 体素大小
     */
    void voxelGridDownsample(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                            double leafSize = 0.01);
    
    /**
     * @brief 直通滤波
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param fieldName 字段名称
     * @param minValue 最小值
     * @param maxValue 最大值
     */
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                          const std::string& fieldName = "z",
                          double minValue = 0.0,
                          double maxValue = 1.0);
    
    /**
     * @brief 裁剪框滤波
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param minPoint 最小点
     * @param maxPoint 最大点
     */
    void cropBoxFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                      const Eigen::Vector4f& minPoint,
                      const Eigen::Vector4f& maxPoint);
    
    /**
     * @brief 计算法线
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云（包含法线）
     * @param searchRadius 搜索半径
     */
    void computeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                       double searchRadius = 0.03);
    
    /**
     * @brief 移动最小二乘平滑
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param searchRadius 搜索半径
     * @param polynomialOrder 多项式阶数
     */
    void movingLeastSquares(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                           double searchRadius = 0.03,
                           int polynomialOrder = 2);
    
    /**
     * @brief 平面分割
     * @param inputCloud 输入点云
     * @param inliers 内点索引
     * @param coefficients 平面系数
     * @param distanceThreshold 距离阈值
     * @param maxIterations 最大迭代次数
     */
    void planeSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                          pcl::PointIndices::Ptr inliers,
                          pcl::ModelCoefficients::Ptr coefficients,
                          double distanceThreshold = 0.01,
                          int maxIterations = 1000);
    
    /**
     * @brief 欧氏聚类分割
     * @param inputCloud 输入点云
     * @param clusters 聚类结果
     * @param tolerance 聚类距离阈值
     * @param minClusterSize 最小聚类大小
     * @param maxClusterSize 最大聚类大小
     */
    void euclideanClustering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                            std::vector<pcl::PointIndices>& clusters,
                            double tolerance = 0.02,
                            int minClusterSize = 100,
                            int maxClusterSize = 25000);
    
    /**
     * @brief SAC-IA配准
     * @param sourceCloud 源点云
     * @param targetCloud 目标点云
     * @param outputCloud 配准后的点云
     * @param transformation 变换矩阵
     * @param minSampleDistance 最小采样距离
     * @param maxCorrespondenceDistance 最大对应点距离
     */
    void sacIARegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                          Eigen::Matrix4f& transformation,
                          double minSampleDistance = 0.05,
                          double maxCorrespondenceDistance = 0.1);
    
    /**
     * @brief ICP配准
     * @param sourceCloud 源点云
     * @param targetCloud 目标点云
     * @param outputCloud 配准后的点云
     * @param transformation 变换矩阵
     * @param maxIterations 最大迭代次数
     * @param transformationEpsilon 变换收敛阈值
     * @param euclideanFitnessEpsilon 欧氏距离收敛阈值
     */
    void icpRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                        Eigen::Matrix4f& transformation,
                        int maxIterations = 100,
                        double transformationEpsilon = 1e-8,
                        double euclideanFitnessEpsilon = 1e-6);
    
    /**
     * @brief GICP配准
     * @param sourceCloud 源点云
     * @param targetCloud 目标点云
     * @param outputCloud 配准后的点云
     * @param transformation 变换矩阵
     * @param maxIterations 最大迭代次数
     * @param transformationEpsilon 变换收敛阈值
     * @param euclideanFitnessEpsilon 欧氏距离收敛阈值
     */
    void gicpRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                         Eigen::Matrix4f& transformation,
                         int maxIterations = 100,
                         double transformationEpsilon = 1e-8,
                         double euclideanFitnessEpsilon = 1e-6);
    
    /**
     * @brief 泊松重建
     * @param inputCloud 输入点云
     * @param outputMesh 输出网格模型
     * @param depth 重建树深度
     * @param pointWeight 点权重
     * @param scale 点云尺度
     */
    void poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                              pcl::PolygonMesh::Ptr outputMesh,
                              int depth = 8,
                              int pointWeight = 4,
                              float scale = 1.1f);
    
    /**
     * @brief 贪婪三角化重建
     * @param inputCloud 输入点云
     * @param outputMesh 输出网格模型
     * @param searchRadius 搜索半径
     * @param mu 距离乘数
     * @param maximumNearestNeighbors 最大邻点数量
     * @param maximumSurfaceAngle 最大表面角度
     */
    void greedyTriangulation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                            pcl::PolygonMesh::Ptr outputMesh,
                            double searchRadius = 0.025,
                            double mu = 2.5,
                            int maximumNearestNeighbors = 100,
                            double maximumSurfaceAngle = M_PI / 4);
    
    /**
     * @brief 计算FPFH特征
     * @param inputCloud 输入点云
     * @param outputFeatures 输出特征
     * @param searchRadius 搜索半径
     */
    void computeFPFH(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
                    double searchRadius = 0.05);
    
    /**
     * @brief 计算SHOT特征
     * @param inputCloud 输入点云
     * @param outputFeatures 输出特征
     * @param searchRadius 搜索半径
     */
    void computeSHOT(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                    pcl::PointCloud<pcl::SHOT352>::Ptr outputFeatures,
                    double searchRadius = 0.05);
    
    /**
     * @brief 区域生长分割
     * @param inputCloud 输入点云
     * @param clusters 聚类结果
     * @param searchRadius 搜索半径
     * @param minClusterSize 最小聚类大小
     * @param maxClusterSize 最大聚类大小
     * @param smoothnessThreshold 平滑度阈值
     * @param curvatureThreshold 曲率阈值
     */
    void regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                  std::vector<pcl::PointIndices>& clusters,
                                  double searchRadius = 0.03,
                                  int minClusterSize = 50,
                                  int maxClusterSize = 10000,
                                  double smoothnessThreshold = 30.0,
                                  double curvatureThreshold = 1.0);
    
    /**
     * @brief RGB区域生长分割
     * @param inputCloud 输入点云
     * @param clusters 聚类结果
     * @param searchRadius 搜索半径
     * @param minClusterSize 最小聚类大小
     * @param maxClusterSize 最大聚类大小
     * @param colorThreshold 颜色阈值
     * @param distanceThreshold 距离阈值
     */
    void regionGrowingRGBSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                     std::vector<pcl::PointIndices>& clusters,
                                     double searchRadius = 0.03,
                                     int minClusterSize = 50,
                                     int maxClusterSize = 10000,
                                     int colorThreshold = 30,
                                     double distanceThreshold = 0.03);
    
    /**
     * @brief NDT配准
     * @param sourceCloud 源点云
     * @param targetCloud 目标点云
     * @param outputCloud 配准后的点云
     * @param transformation 变换矩阵
     * @param resolution 体素分辨率
     * @param stepSize 步长
     * @param outlierRatio 异常值比例
     */
    void ndtRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                        Eigen::Matrix4f& transformation,
                        double resolution = 1.0,
                        double stepSize = 0.1,
                        double outlierRatio = 0.5);
    
    /**
     * @brief 均匀采样
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param radius 采样半径
     */
    void uniformSampling(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                        double radius = 0.01);
    
    /**
     * @brief 随机采样
     * @param inputCloud 输入点云
     * @param outputCloud 输出点云
     * @param sampleSize 采样点数量
     */
    void randomSampling(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                       int sampleSize = 10000);
    
    /**
     * @brief 计算点云质心
     * @param inputCloud 输入点云
     * @return 质心坐标
     */
    Eigen::Vector4f computeCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud);
    
    /**
     * @brief 计算点云边界框
     * @param inputCloud 输入点云
     * @param minPoint 最小点
     * @param maxPoint 最大点
     */
    void computeBoundingBox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                          Eigen::Vector4f& minPoint,
                          Eigen::Vector4f& maxPoint);

private:
    /**
     * @brief 构造函数
     */
    PointCloudProcessor();
    
    /**
     * @brief 析构函数
     */
    ~PointCloudProcessor();
};

#endif // POINTCLOUDPROCESSOR_H