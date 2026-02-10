#include "PointCloudProcessor.h"

/**
 * @brief 构造函数
 */
PointCloudProcessor::PointCloudProcessor()
{
}

/**
 * @brief 析构函数
 */
PointCloudProcessor::~PointCloudProcessor()
{
}

/**
 * @brief 获取点云处理器实例
 * @return 点云处理器实例
 */
PointCloudProcessor& PointCloudProcessor::getInstance()
{
    static PointCloudProcessor instance;
    return instance;
}

/**
 * @brief 统计滤波
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param meanK 近邻点数量
 * @param stdDevMulThresh 标准差阈值
 */
void PointCloudProcessor::statisticalFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                          int meanK,
                                          double stdDevMulThresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud(inputCloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdDevMulThresh);
    sor.filter(*outputCloud);
}

/**
 * @brief 半径滤波
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param radiusSearch 搜索半径
 * @param minPts 最小邻点数量
 */
void PointCloudProcessor::radiusFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                     double radiusSearch,
                                     int minPts)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> ror;
    ror.setInputCloud(inputCloud);
    ror.setRadiusSearch(radiusSearch);
    ror.setMinNeighborsInRadius(minPts);
    ror.filter(*outputCloud);
}

/**
 * @brief 体素网格下采样
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param leafSize 体素大小
 */
void PointCloudProcessor::voxelGridDownsample(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                            double leafSize)
{
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
    vg.setInputCloud(inputCloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*outputCloud);
}

/**
 * @brief 直通滤波
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param fieldName 字段名称
 * @param minValue 最小值
 * @param maxValue 最大值
 */
void PointCloudProcessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                          const std::string& fieldName,
                                          double minValue,
                                          double maxValue)
{
    pcl::PassThrough<pcl::PointXYZRGBNormal> pt;
    pt.setInputCloud(inputCloud);
    pt.setFilterFieldName(fieldName);
    pt.setFilterLimits(minValue, maxValue);
    pt.filter(*outputCloud);
}

/**
 * @brief 裁剪框滤波
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param minPoint 最小点
 * @param maxPoint 最大点
 */
void PointCloudProcessor::cropBoxFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                      const Eigen::Vector4f& minPoint,
                                      const Eigen::Vector4f& maxPoint)
{
    pcl::CropBox<pcl::PointXYZRGBNormal> cb;
    cb.setInputCloud(inputCloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*outputCloud);
}

/**
 * @brief 计算法线
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云（包含法线）
 * @param searchRadius 搜索半径
 */
void PointCloudProcessor::computeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                       double searchRadius)
{
    pcl::copyPointCloud(*inputCloud, *outputCloud);
    
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud(outputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(searchRadius);
    ne.compute(*outputCloud);
}

/**
 * @brief 移动最小二乘平滑
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param searchRadius 搜索半径
 * @param polynomialOrder 多项式阶数
 */
void PointCloudProcessor::movingLeastSquares(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                           double searchRadius,
                                           int polynomialOrder)
{
    pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
    mls.setInputCloud(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(searchRadius);
    mls.setPolynomialOrder(polynomialOrder);
    mls.setComputeNormals(true);
    
    mls.process(*outputCloud);
}

/**
 * @brief 平面分割
 * @param inputCloud 输入点云
 * @param inliers 内点索引
 * @param coefficients 平面系数
 * @param distanceThreshold 距离阈值
 * @param maxIterations 最大迭代次数
 */
void PointCloudProcessor::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                          pcl::PointIndices::Ptr inliers,
                                          pcl::ModelCoefficients::Ptr coefficients,
                                          double distanceThreshold,
                                          int maxIterations)
{
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);
}

/**
 * @brief 欧氏聚类分割
 * @param inputCloud 输入点云
 * @param clusters 聚类结果
 * @param tolerance 聚类距离阈值
 * @param minClusterSize 最小聚类大小
 * @param maxClusterSize 最大聚类大小
 */
void PointCloudProcessor::euclideanClustering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                            std::vector<pcl::PointIndices>& clusters,
                                            double tolerance,
                                            int minClusterSize,
                                            int maxClusterSize)
{
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(inputCloud);
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(inputCloud);
    ec.extract(clusters);
}

/**
 * @brief SAC-IA配准
 * @param sourceCloud 源点云
 * @param targetCloud 目标点云
 * @param outputCloud 配准后的点云
 * @param transformation 变换矩阵
 * @param minSampleDistance 最小采样距离
 * @param maxCorrespondenceDistance 最大对应点距离
 */
void PointCloudProcessor::sacIARegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                          Eigen::Matrix4f& transformation,
                                          double minSampleDistance,
                                          double maxCorrespondenceDistance)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> sac_ia;
    
    // 计算源点云和目标点云的FPFH特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    
    pcl::FPFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.05);
    
    fpfh.setInputCloud(sourceCloud);
    fpfh.setInputNormals(sourceCloud);
    fpfh.compute(*source_features);
    
    fpfh.setInputCloud(targetCloud);
    fpfh.setInputNormals(targetCloud);
    fpfh.compute(*target_features);
    
    // 设置SAC-IA参数
    sac_ia.setInputSource(sourceCloud);
    sac_ia.setInputTarget(targetCloud);
    sac_ia.setSourceFeatures(source_features);
    sac_ia.setTargetFeatures(target_features);
    sac_ia.setMinSampleDistance(minSampleDistance);
    sac_ia.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    sac_ia.setNumberOfSamples(20);
    sac_ia.setCorrespondenceRandomness(5);
    
    // 执行配准
    sac_ia.align(*outputCloud);
    transformation = sac_ia.getFinalTransformation();
}

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
void PointCloudProcessor::icpRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                        Eigen::Matrix4f& transformation,
                                        int maxIterations,
                                        double transformationEpsilon,
                                        double euclideanFitnessEpsilon)
{
    pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    icp.setInputSource(sourceCloud);
    icp.setInputTarget(targetCloud);
    icp.setMaxIterations(maxIterations);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
    icp.align(*outputCloud);
    transformation = icp.getFinalTransformation();
}

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
void PointCloudProcessor::gicpRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                         Eigen::Matrix4f& transformation,
                                         int maxIterations,
                                         double transformationEpsilon,
                                         double euclideanFitnessEpsilon)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> gicp;
    gicp.setInputSource(sourceCloud);
    gicp.setInputTarget(targetCloud);
    gicp.setMaxIterations(maxIterations);
    gicp.setTransformationEpsilon(transformationEpsilon);
    gicp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
    gicp.align(*outputCloud);
    transformation = gicp.getFinalTransformation();
}

/**
 * @brief 泊松重建
 * @param inputCloud 输入点云
 * @param outputMesh 输出网格模型
 * @param depth 重建树深度
 * @param pointWeight 点权重
 * @param scale 点云尺度
 */
void PointCloudProcessor::poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                              pcl::PolygonMesh::Ptr outputMesh,
                                              int depth,
                                              int pointWeight,
                                              float scale)
{
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setInputCloud(inputCloud);
    poisson.setDepth(depth);
    poisson.setPointWeight(pointWeight);
    poisson.setScale(scale);
    poisson.setSamplesPerNode(3.0f);
    poisson.reconstruct(*outputMesh);
}

/**
 * @brief 贪婪三角化重建
 * @param inputCloud 输入点云
 * @param outputMesh 输出网格模型
 * @param searchRadius 搜索半径
 * @param mu 距离乘数
 * @param maximumNearestNeighbors 最大邻点数量
 * @param maximumSurfaceAngle 最大表面角度
 */
void PointCloudProcessor::greedyTriangulation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                            pcl::PolygonMesh::Ptr outputMesh,
                                            double searchRadius,
                                            double mu,
                                            int maximumNearestNeighbors,
                                            double maximumSurfaceAngle)
{
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    gp3.setInputCloud(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    gp3.setSearchMethod(tree);
    gp3.setSearchRadius(searchRadius);
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(maximumNearestNeighbors);
    gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);
    
    gp3.reconstruct(*outputMesh);
}

/**
 * @brief 计算FPFH特征
 * @param inputCloud 输入点云
 * @param outputFeatures 输出特征
 * @param searchRadius 搜索半径
 */
void PointCloudProcessor::computeFPFH(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures,
                                    double searchRadius)
{
    pcl::FPFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(inputCloud);
    fpfh.setInputNormals(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(searchRadius);
    
    fpfh.compute(*outputFeatures);
}

/**
 * @brief 计算SHOT特征
 * @param inputCloud 输入点云
 * @param outputFeatures 输出特征
 * @param searchRadius 搜索半径
 */
void PointCloudProcessor::computeSHOT(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                    pcl::PointCloud<pcl::SHOT352>::Ptr outputFeatures,
                                    double searchRadius)
{
    pcl::SHOTEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> shot;
    shot.setInputCloud(inputCloud);
    shot.setInputNormals(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    shot.setSearchMethod(tree);
    shot.setRadiusSearch(searchRadius);
    
    shot.compute(*outputFeatures);
}

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
void PointCloudProcessor::regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                                  std::vector<pcl::PointIndices>& clusters,
                                                  double searchRadius,
                                                  int minClusterSize,
                                                  int maxClusterSize,
                                                  double smoothnessThreshold,
                                                  double curvatureThreshold)
{
    pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> reg;
    reg.setInputCloud(inputCloud);
    reg.setInputNormals(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    reg.setSearchMethod(tree);
    reg.setSearchRadius(searchRadius);
    reg.setMinClusterSize(minClusterSize);
    reg.setMaxClusterSize(maxClusterSize);
    reg.setSmoothnessThreshold(smoothnessThreshold / 180.0 * M_PI); // 转换为弧度
    reg.setCurvatureThreshold(curvatureThreshold);
    
    reg.extract(clusters);
}

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
void PointCloudProcessor::regionGrowingRGBSegmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                                     std::vector<pcl::PointIndices>& clusters,
                                                     double searchRadius,
                                                     int minClusterSize,
                                                     int maxClusterSize,
                                                     int colorThreshold,
                                                     double distanceThreshold)
{
    pcl::RegionGrowingRGB<pcl::PointXYZRGBNormal> reg;
    reg.setInputCloud(inputCloud);
    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    reg.setSearchMethod(tree);
    reg.setSearchRadius(searchRadius);
    reg.setMinClusterSize(minClusterSize);
    reg.setMaxClusterSize(maxClusterSize);
    reg.setDistanceThreshold(distanceThreshold);
    reg.setPointColorThreshold(colorThreshold);
    reg.setRegionColorThreshold(colorThreshold);
    
    reg.extract(clusters);
}

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
void PointCloudProcessor::ndtRegistration(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                        Eigen::Matrix4f& transformation,
                                        double resolution,
                                        double stepSize,
                                        double outlierRatio)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ndt;
    ndt.setInputSource(sourceCloud);
    ndt.setInputTarget(targetCloud);
    
    ndt.setResolution(resolution);
    ndt.setStepSize(stepSize);
    ndt.setOutlierRatio(outlierRatio);
    ndt.setMaximumIterations(35);
    
    // 执行配准
    ndt.align(*outputCloud);
    transformation = ndt.getFinalTransformation();
}

/**
 * @brief 均匀采样
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param radius 采样半径
 */
void PointCloudProcessor::uniformSampling(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                        double radius)
{
    pcl::UniformSampling<pcl::PointXYZRGBNormal> us;
    us.setInputCloud(inputCloud);
    us.setRadiusSearch(radius);
    us.filter(*outputCloud);
}

/**
 * @brief 随机采样
 * @param inputCloud 输入点云
 * @param outputCloud 输出点云
 * @param sampleSize 采样点数量
 */
void PointCloudProcessor::randomSampling(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                                       int sampleSize)
{
    pcl::RandomSample<pcl::PointXYZRGBNormal> rs;
    rs.setInputCloud(inputCloud);
    rs.setSample(sampleSize);
    rs.setSeed(42); // 设置随机种子，保证结果可重现
    rs.filter(*outputCloud);
}

/**
 * @brief 计算点云质心
 * @param inputCloud 输入点云
 * @return 质心坐标
 */
Eigen::Vector4f PointCloudProcessor::computeCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*inputCloud, centroid);
    return centroid;
}

/**
 * @brief 计算点云边界框
 * @param inputCloud 输入点云
 * @param minPoint 最小点
 * @param maxPoint 最大点
 */
void PointCloudProcessor::computeBoundingBox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                                          Eigen::Vector4f& minPoint,
                                          Eigen::Vector4f& maxPoint)
{
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);
}