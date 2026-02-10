#ifndef VISUALIZATIONWIDGET_H
#define VISUALIZATIONWIDGET_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QQuaternion>
#include <QMouseEvent>
#include <QWheelEvent>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/polygonmesh.h>

/**
 * @brief 可视化模式枚举
 */
enum class VisualizationMode {
    POINT_CLOUD,  // 点云模式
    MESH_MODEL,   // 网格模型模式
    BOTH          // 同时显示
};

/**
 * @brief 渲染模式枚举
 */
enum class RenderMode {
    POINTS,       // 点模式
    WIREFRAME,    // 线框模式
    SOLID         // 实体模式
};

/**
 * @brief 可视化窗口类
 * 实现点云数据和3D模型的OpenGL渲染
 */
class VisualizationWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口
     */
    VisualizationWidget(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~VisualizationWidget();
    
    /**
     * @brief 设置点云数据
     * @param cloud 点云数据
     */
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    
    /**
     * @brief 设置网格模型
     * @param mesh 网格模型
     */
    void setMeshModel(pcl::PolygonMesh::Ptr mesh);
    
    /**
     * @brief 设置可视化模式
     * @param mode 可视化模式
     */
    void setVisualizationMode(VisualizationMode mode);
    
    /**
     * @brief 设置渲染模式
     * @param mode 渲染模式
     */
    void setRenderMode(RenderMode mode);
    
    /**
     * @brief 设置点大小
     * @param size 点大小
     */
    void setPointSize(float size);
    
    /**
     * @brief 设置线宽
     * @param width 线宽
     */
    void setLineWidth(float width);
    
    /**
     * @brief 重置视角
     */
    void resetView();
    
    /**
     * @brief 放大视图
     */
    void zoomIn();
    
    /**
     * @brief 缩小视图
     */
    void zoomOut();
    
    /**
     * @brief 俯视图
     */
    void topView();
    
    /**
     * @brief 仰视图
     */
    void bottomView();
    
    /**
     * @brief 前视图
     */
    void frontView();
    
    /**
     * @brief 后视图
     */
    void backView();
    
    /**
     * @brief 左视图
     */
    void leftView();
    
    /**
     * @brief 右视图
     */
    void rightView();
    
    /**
     * @brief 等轴测视图
     */
    void isometricView();

protected:
    /**
     * @brief 初始化OpenGL
     */
    void initializeGL() override;
    
    /**
     * @brief 调整窗口大小
     * @param width 宽度
     * @param height 高度
     */
    void resizeGL(int width, int height) override;
    
    /**
     * @brief 绘制场景
     */
    void paintGL() override;
    
    /**
     * @brief 鼠标按下事件
     * @param event 鼠标事件
     */
    void mousePressEvent(QMouseEvent *event) override;
    
    /**
     * @brief 鼠标移动事件
     * @param event 鼠标事件
     */
    void mouseMoveEvent(QMouseEvent *event) override;
    
    /**
     * @brief 鼠标释放事件
     * @param event 鼠标事件
     */
    void mouseReleaseEvent(QMouseEvent *event) override;
    
    /**
     * @brief 鼠标滚轮事件
     * @param event 鼠标事件
     */
    void wheelEvent(QWheelEvent *event) override;

private:
    /**
     * @brief 初始化着色器
     */
    void initShaders();
    
    /**
     * @brief 初始化缓冲区
     */
    void initBuffers();
    
    /**
     * @brief 更新点云缓冲区
     */
    void updatePointCloudBuffer();
    
    /**
     * @brief 更新网格缓冲区
     */
    void updateMeshBuffer();
    
    /**
     * @brief 渲染点云
     */
    void renderPointCloud();
    
    /**
     * @brief 渲染网格模型
     */
    void renderMeshModel();
    
    /**
     * @brief 渲染坐标系
     */
    void renderCoordinateSystem();
    
    /**
     * @brief 计算模型视图投影矩阵
     */
    void computeMatrices();

private:
    // 着色器程序
    QOpenGLShaderProgram *pointShaderProgram;
    QOpenGLShaderProgram *meshShaderProgram;
    QOpenGLShaderProgram *coordShaderProgram;
    
    // 顶点数组对象
    QOpenGLVertexArrayObject pointVAO;
    QOpenGLVertexArrayObject meshVAO;
    QOpenGLVertexArrayObject coordVAO;
    
    // 缓冲区
    QOpenGLBuffer pointVBO;
    QOpenGLBuffer meshVBO;
    QOpenGLBuffer meshEBO;
    QOpenGLBuffer coordVBO;
    
    // 矩阵
    QMatrix4x4 modelMatrix;
    QMatrix4x4 viewMatrix;
    QMatrix4x4 projectionMatrix;
    QMatrix4x4 mvpMatrix;
    
    // 相机参数
    QVector3D cameraPosition;
    QVector3D cameraTarget;
    QVector3D cameraUp;
    float cameraDistance;
    
    // 鼠标交互
    bool mousePressed;
    QPoint lastMousePos;
    QQuaternion rotation;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud;
    bool pointCloudUpdated;
    
    // 网格模型
    pcl::PolygonMesh::Ptr meshModel;
    bool meshUpdated;
    
    // 渲染参数
    VisualizationMode visualizationMode;
    RenderMode renderMode;
    float pointSize;
    float lineWidth;
    
    // 视图参数
    float fieldOfView;
    float aspectRatio;
    float nearPlane;
    float farPlane;
};

#endif // VISUALIZATIONWIDGET_H