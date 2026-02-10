#include "VisualizationWidget.h"
#include <iostream>

/**
 * @brief 构造函数
 * @param parent 父窗口
 */
VisualizationWidget::VisualizationWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      mousePressed(false),
      pointCloudUpdated(false),
      meshUpdated(false),
      visualizationMode(VisualizationMode::POINT_CLOUD),
      renderMode(RenderMode::POINTS),
      pointSize(2.0f),
      lineWidth(1.0f),
      fieldOfView(45.0f),
      nearPlane(0.1f),
      farPlane(1000.0f)
{
    // 初始化相机参数
    cameraPosition = QVector3D(0.0f, 0.0f, 2.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    cameraDistance = 2.0f;
    
    // 初始化旋转
    rotation = QQuaternion();
    
    // 初始化点云数据
    pointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    // 初始化网格模型
    meshModel = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
}

/**
 * @brief 析构函数
 */
VisualizationWidget::~VisualizationWidget()
{
    // 清理资源
    makeCurrent();
    pointVBO.destroy();
    meshVBO.destroy();
    meshEBO.destroy();
    coordVBO.destroy();
    pointVAO.destroy();
    meshVAO.destroy();
    coordVAO.destroy();
    delete pointShaderProgram;
    delete meshShaderProgram;
    delete coordShaderProgram;
    doneCurrent();
}

/**
 * @brief 初始化OpenGL
 */
void VisualizationWidget::initializeGL()
{
    // 初始化OpenGL函数
    initializeOpenGLFunctions();
    
    // 设置背景色
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    
    // 初始化着色器
    initShaders();
    
    // 初始化缓冲区
    initBuffers();
}

/**
 * @brief 调整窗口大小
 * @param width 宽度
 * @param height 高度
 */
void VisualizationWidget::resizeGL(int width, int height)
{
    // 设置视口
    glViewport(0, 0, width, height);
    
    // 更新宽高比
    aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    
    // 重新计算投影矩阵
    computeMatrices();
}

/**
 * @brief 绘制场景
 */
void VisualizationWidget::paintGL()
{
    // 清除颜色和深度缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 更新缓冲区
    if (pointCloudUpdated) {
        updatePointCloudBuffer();
        pointCloudUpdated = false;
    }
    
    if (meshUpdated) {
        updateMeshBuffer();
        meshUpdated = false;
    }
    
    // 计算矩阵
    computeMatrices();
    
    // 渲染坐标系
    renderCoordinateSystem();
    
    // 渲染点云或网格
    if (visualizationMode == VisualizationMode::POINT_CLOUD || visualizationMode == VisualizationMode::BOTH) {
        renderPointCloud();
    }
    
    if (visualizationMode == VisualizationMode::MESH_MODEL || visualizationMode == VisualizationMode::BOTH) {
        renderMeshModel();
    }
}

/**
 * @brief 鼠标按下事件
 * @param event 鼠标事件
 */
void VisualizationWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        mousePressed = true;
        lastMousePos = event->pos();
    }
}

/**
 * @brief 鼠标移动事件
 * @param event 鼠标事件
 */
void VisualizationWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (mousePressed) {
        QPoint delta = event->pos() - lastMousePos;
        
        // 计算旋转角度
        float angleX = delta.y() * 0.5f;
        float angleY = delta.x() * 0.5f;
        
        // 创建旋转四元数
        QQuaternion rotationX = QQuaternion::fromAxisAndAngle(1.0f, 0.0f, 0.0f, angleX);
        QQuaternion rotationY = QQuaternion::fromAxisAndAngle(0.0f, 1.0f, 0.0f, angleY);
        
        // 应用旋转
        rotation = rotationY * rotationX * rotation;
        
        // 更新相机位置
        QVector3D direction = QVector3D(0.0f, 0.0f, -cameraDistance);
        direction = rotation.rotatedVector(direction);
        cameraPosition = cameraTarget + direction;
        
        lastMousePos = event->pos();
        update();
    }
}

/**
 * @brief 鼠标释放事件
 * @param event 鼠标事件
 */
void VisualizationWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        mousePressed = false;
    }
}

/**
 * @brief 鼠标滚轮事件
 * @param event 鼠标事件
 */
void VisualizationWidget::wheelEvent(QWheelEvent *event)
{
    // 计算缩放因子
    float scaleFactor = event->angleDelta().y() > 0 ? 0.9f : 1.1f;
    cameraDistance *= scaleFactor;
    
    // 限制相机距离
    cameraDistance = qMax(0.1f, qMin(cameraDistance, 100.0f));
    
    // 更新相机位置
    QVector3D direction = QVector3D(0.0f, 0.0f, -cameraDistance);
    direction = rotation.rotatedVector(direction);
    cameraPosition = cameraTarget + direction;
    
    update();
}

/**
 * @brief 初始化着色器
 */
void VisualizationWidget::initShaders()
{
    // 点云着色器
    pointShaderProgram = new QOpenGLShaderProgram;
    pointShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, ""
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aColor;\n"
        "uniform mat4 MVP;\n"
        "out vec3 ourColor;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = MVP * vec4(aPos, 1.0);\n"
        "    ourColor = aColor;\n"
        "}\n"
    );
    pointShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, ""
        "#version 330 core\n"
        "in vec3 ourColor;\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "    FragColor = vec4(ourColor, 1.0);\n"
        "}\n"
    );
    pointShaderProgram->link();
    
    // 网格着色器
    meshShaderProgram = new QOpenGLShaderProgram;
    meshShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, ""
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aNormal;\n"
        "uniform mat4 MVP;\n"
        "uniform mat4 model;\n"
        "out vec3 Normal;\n"
        "out vec3 FragPos;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = MVP * vec4(aPos, 1.0);\n"
        "    FragPos = vec3(model * vec4(aPos, 1.0));\n"
        "    Normal = mat3(transpose(inverse(model))) * aNormal;\n"
        "}\n"
    );
    meshShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, ""
        "#version 330 core\n"
        "in vec3 Normal;\n"
        "in vec3 FragPos;\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "    vec3 lightPos = vec3(1.0, 1.0, 1.0);\n"
        "    vec3 lightColor = vec3(1.0, 1.0, 1.0);\n"
        "    vec3 objectColor = vec3(0.5, 0.5, 0.5);\n"
        "    \n"
        "    // 环境光\n"
        "    float ambientStrength = 0.1;\n"
        "    vec3 ambient = ambientStrength * lightColor;\n"
        "    \n"
        "    // 漫反射\n"
        "    vec3 norm = normalize(Normal);\n"
        "    vec3 lightDir = normalize(lightPos - FragPos);\n"
        "    float diff = max(dot(norm, lightDir), 0.0);\n"
        "    vec3 diffuse = diff * lightColor;\n"
        "    \n"
        "    // 镜面反射\n"
        "    float specularStrength = 0.5;\n"
        "    vec3 viewDir = normalize(vec3(0.0) - FragPos);\n"
        "    vec3 reflectDir = reflect(-lightDir, norm);\n"
        "    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);\n"
        "    vec3 specular = specularStrength * spec * lightColor;\n"
        "    \n"
        "    vec3 result = (ambient + diffuse + specular) * objectColor;\n"
        "    FragColor = vec4(result, 1.0);\n"
        "}\n"
    );
    meshShaderProgram->link();
    
    // 坐标系着色器
    coordShaderProgram = new QOpenGLShaderProgram;
    coordShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, ""
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aColor;\n"
        "uniform mat4 MVP;\n"
        "out vec3 ourColor;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = MVP * vec4(aPos, 1.0);\n"
        "    ourColor = aColor;\n"
        "}\n"
    );
    coordShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, ""
        "#version 330 core\n"
        "in vec3 ourColor;\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "    FragColor = vec4(ourColor, 1.0);\n"
        "}\n"
    );
    coordShaderProgram->link();
}

/**
 * @brief 初始化缓冲区
 */
void VisualizationWidget::initBuffers()
{
    // 初始化点云缓冲区
    pointVAO.create();
    pointVBO.create();
    
    // 初始化网格缓冲区
    meshVAO.create();
    meshVBO.create();
    meshEBO.create();
    
    // 初始化坐标系缓冲区
    coordVAO.create();
    coordVBO.create();
    
    // 坐标系数据
    float coordData[] = {
        // 位置           颜色
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X轴起点（红色）
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X轴终点
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y轴起点（绿色）
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y轴终点
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // Z轴起点（蓝色）
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f   // Z轴终点
    };
    
    coordVAO.bind();
    coordVBO.bind();
    coordVBO.allocate(coordData, sizeof(coordData));
    
    coordShaderProgram->bind();
    int posLocation = coordShaderProgram->attributeLocation("aPos");
    int colorLocation = coordShaderProgram->attributeLocation("aColor");
    
    coordVBO.setAttributeBuffer(posLocation, GL_FLOAT, 0, 3, 6 * sizeof(float));
    coordVBO.setAttributeBuffer(colorLocation, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    
    coordShaderProgram->enableAttributeArray(posLocation);
    coordShaderProgram->enableAttributeArray(colorLocation);
    
    coordVAO.release();
    coordVBO.release();
}

/**
 * @brief 更新点云缓冲区
 */
void VisualizationWidget::updatePointCloudBuffer()
{
    if (pointCloud->empty()) {
        return;
    }
    
    // 准备点云数据
    std::vector<float> pointData;
    for (const auto& point : pointCloud->points) {
        // 位置
        pointData.push_back(point.x);
        pointData.push_back(point.y);
        pointData.push_back(point.z);
        // 颜色
        pointData.push_back(static_cast<float>(point.r) / 255.0f);
        pointData.push_back(static_cast<float>(point.g) / 255.0f);
        pointData.push_back(static_cast<float>(point.b) / 255.0f);
    }
    
    pointVAO.bind();
    pointVBO.bind();
    pointVBO.allocate(pointData.data(), pointData.size() * sizeof(float));
    
    pointShaderProgram->bind();
    int posLocation = pointShaderProgram->attributeLocation("aPos");
    int colorLocation = pointShaderProgram->attributeLocation("aColor");
    
    pointVBO.setAttributeBuffer(posLocation, GL_FLOAT, 0, 3, 6 * sizeof(float));
    pointVBO.setAttributeBuffer(colorLocation, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    
    pointShaderProgram->enableAttributeArray(posLocation);
    pointShaderProgram->enableAttributeArray(colorLocation);
    
    pointVAO.release();
    pointVBO.release();
}

/**
 * @brief 更新网格缓冲区
 */
void VisualizationWidget::updateMeshBuffer()
{
    if (meshModel->polygons.empty()) {
        return;
    }
    
    // 从网格模型中提取顶点和索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(meshModel->cloud, *cloud);
    
    // 准备顶点数据
    std::vector<float> vertexData;
    for (const auto& point : cloud->points) {
        vertexData.push_back(point.x);
        vertexData.push_back(point.y);
        vertexData.push_back(point.z);
        // 法线（暂时使用默认值）
        vertexData.push_back(0.0f);
        vertexData.push_back(0.0f);
        vertexData.push_back(1.0f);
    }
    
    // 准备索引数据
    std::vector<unsigned int> indexData;
    for (const auto& polygon : meshModel->polygons) {
        for (const auto& index : polygon.vertices) {
            indexData.push_back(index);
        }
    }
    
    meshVAO.bind();
    meshVBO.bind();
    meshVBO.allocate(vertexData.data(), vertexData.size() * sizeof(float));
    
    meshEBO.bind();
    meshEBO.allocate(indexData.data(), indexData.size() * sizeof(unsigned int));
    
    meshShaderProgram->bind();
    int posLocation = meshShaderProgram->attributeLocation("aPos");
    int normalLocation = meshShaderProgram->attributeLocation("aNormal");
    
    meshVBO.setAttributeBuffer(posLocation, GL_FLOAT, 0, 3, 6 * sizeof(float));
    meshVBO.setAttributeBuffer(normalLocation, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    
    meshShaderProgram->enableAttributeArray(posLocation);
    meshShaderProgram->enableAttributeArray(normalLocation);
    
    meshVAO.release();
    meshVBO.release();
    meshEBO.release();
}

/**
 * @brief 渲染点云
 */
void VisualizationWidget::renderPointCloud()
{
    if (pointCloud->empty()) {
        return;
    }
    
    // 设置点大小
    glPointSize(pointSize);
    
    // 绑定着色器程序和VAO
    pointShaderProgram->bind();
    pointVAO.bind();
    
    // 设置 uniforms
    pointShaderProgram->setUniformValue("MVP", mvpMatrix);
    
    // 渲染点云
    glDrawArrays(GL_POINTS, 0, pointCloud->size());
    
    // 释放资源
    pointVAO.release();
    pointShaderProgram->release();
}

/**
 * @brief 渲染网格模型
 */
void VisualizationWidget::renderMeshModel()
{
    if (meshModel->polygons.empty()) {
        return;
    }
    
    // 设置线宽
    glLineWidth(lineWidth);
    
    // 绑定着色器程序和VAO
    meshShaderProgram->bind();
    meshVAO.bind();
    
    // 设置 uniforms
    meshShaderProgram->setUniformValue("MVP", mvpMatrix);
    meshShaderProgram->setUniformValue("model", modelMatrix);
    
    // 根据渲染模式绘制
    if (renderMode == RenderMode::WIREFRAME) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    // 计算三角形数量
    int triangleCount = 0;
    for (const auto& polygon : meshModel->polygons) {
        triangleCount += polygon.vertices.size() - 2;
    }
    
    // 渲染网格
    glDrawElements(GL_TRIANGLES, triangleCount * 3, GL_UNSIGNED_INT, 0);
    
    // 恢复默认多边形模式
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    // 释放资源
    meshVAO.release();
    meshShaderProgram->release();
}

/**
 * @brief 渲染坐标系
 */
void VisualizationWidget::renderCoordinateSystem()
{
    // 设置线宽
    glLineWidth(2.0f);
    
    // 绑定着色器程序和VAO
    coordShaderProgram->bind();
    coordVAO.bind();
    
    // 设置 uniforms
    coordShaderProgram->setUniformValue("MVP", mvpMatrix);
    
    // 渲染坐标系
    glDrawArrays(GL_LINES, 0, 6);
    
    // 释放资源
    coordVAO.release();
    coordShaderProgram->release();
}

/**
 * @brief 计算模型视图投影矩阵
 */
void VisualizationWidget::computeMatrices()
{
    // 模型矩阵
    modelMatrix.setToIdentity();
    
    // 视图矩阵
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(cameraPosition, cameraTarget, cameraUp);
    
    // 投影矩阵
    projectionMatrix.setToIdentity();
    projectionMatrix.perspective(fieldOfView, aspectRatio, nearPlane, farPlane);
    
    // MVP矩阵
    mvpMatrix = projectionMatrix * viewMatrix * modelMatrix;
}

/**
 * @brief 设置点云数据
 * @param cloud 点云数据
 */
void VisualizationWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pointCloud = cloud;
    pointCloudUpdated = true;
    update();
}

/**
 * @brief 设置网格模型
 * @param mesh 网格模型
 */
void VisualizationWidget::setMeshModel(pcl::PolygonMesh::Ptr mesh)
{
    meshModel = mesh;
    meshUpdated = true;
    update();
}

/**
 * @brief 设置可视化模式
 * @param mode 可视化模式
 */
void VisualizationWidget::setVisualizationMode(VisualizationMode mode)
{
    visualizationMode = mode;
    update();
}

/**
 * @brief 设置渲染模式
 * @param mode 渲染模式
 */
void VisualizationWidget::setRenderMode(RenderMode mode)
{
    renderMode = mode;
    update();
}

/**
 * @brief 设置点大小
 * @param size 点大小
 */
void VisualizationWidget::setPointSize(float size)
{
    pointSize = size;
    update();
}

/**
 * @brief 设置线宽
 * @param width 线宽
 */
void VisualizationWidget::setLineWidth(float width)
{
    lineWidth = width;
    update();
}

/**
 * @brief 重置视角
 */
void VisualizationWidget::resetView()
{
    // 重置相机参数
    cameraPosition = QVector3D(0.0f, 0.0f, 2.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    cameraDistance = 2.0f;
    
    // 重置旋转
    rotation = QQuaternion();
    
    update();
}

/**
 * @brief 放大视图
 */
void VisualizationWidget::zoomIn()
{
    cameraDistance *= 0.9f;
    cameraDistance = qMax(0.1f, cameraDistance);
    
    // 更新相机位置
    QVector3D direction = QVector3D(0.0f, 0.0f, -cameraDistance);
    direction = rotation.rotatedVector(direction);
    cameraPosition = cameraTarget + direction;
    
    update();
}

/**
 * @brief 缩小视图
 */
void VisualizationWidget::zoomOut()
{
    cameraDistance *= 1.1f;
    cameraDistance = qMin(cameraDistance, 100.0f);
    
    // 更新相机位置
    QVector3D direction = QVector3D(0.0f, 0.0f, -cameraDistance);
    direction = rotation.rotatedVector(direction);
    cameraPosition = cameraTarget + direction;
    
    update();
}

/**
 * @brief 俯视图
 */
void VisualizationWidget::topView()
{
    cameraPosition = QVector3D(0.0f, 2.0f, 0.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 0.0f, -1.0f);
    update();
}

/**
 * @brief 仰视图
 */
void VisualizationWidget::bottomView()
{
    cameraPosition = QVector3D(0.0f, -2.0f, 0.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 0.0f, 1.0f);
    update();
}

/**
 * @brief 前视图
 */
void VisualizationWidget::frontView()
{
    cameraPosition = QVector3D(0.0f, 0.0f, 2.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    update();
}

/**
 * @brief 后视图
 */
void VisualizationWidget::backView()
{
    cameraPosition = QVector3D(0.0f, 0.0f, -2.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    update();
}

/**
 * @brief 左视图
 */
void VisualizationWidget::leftView()
{
    cameraPosition = QVector3D(-2.0f, 0.0f, 0.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    update();
}

/**
 * @brief 右视图
 */
void VisualizationWidget::rightView()
{
    cameraPosition = QVector3D(2.0f, 0.0f, 0.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    update();
}

/**
 * @brief 等轴测视图
 */
void VisualizationWidget::isometricView()
{
    cameraPosition = QVector3D(1.0f, 1.0f, 1.0f);
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    update();
}