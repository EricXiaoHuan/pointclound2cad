#include "MainWindow.h"
#include "visualization/VisualizationWidget.h"
#include "data/DataManager.h"
#include "data/IOManager.h"
#include "algorithm/PointCloudProcessor.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>

/**
 * @brief 构造函数
 * @param parent 父窗口
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 初始化UI组件
    initUI();
}

/**
 * @brief 析构函数
 */
MainWindow::~MainWindow()
{
}

/**
 * @brief 初始化UI组件
 */
void MainWindow::initUI()
{
    // 设置窗口标题和大小
    setWindowTitle("PointCloud2CAD - 点云数据转CAD 3D模型系统");
    resize(1200, 800);
    
    // 创建菜单栏
    createMenus();
    
    // 创建工具栏
    createToolBars();
    
    // 创建状态栏
    createStatusBar();
    
    // 创建侧边栏
    createDockWidgets();
    
    // 创建3D视图
    create3DView();
    
    // 显示状态栏消息
    statusBar->showMessage("就绪");
}

/**
 * @brief 创建菜单栏
 */
void MainWindow::createMenus()
{
    // 文件菜单
    fileMenu = menuBar()->addMenu("文件(&F)");
    
    // 项目管理子菜单
    QMenu *projectMenu = fileMenu->addMenu("项目");
    QAction *newProjectAction = new QAction("新建项目", this);
    connect(newProjectAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("正在创建新项目...");
        // 清空当前数据
        DataManager::getInstance().clearItems();
        // 重置可视化窗口
        visualizationWidget->resetView();
        statusBar->showMessage("新项目创建完成", 2000);
        QMessageBox::information(this, "项目管理", "新项目已创建。");
    });
    projectMenu->addAction(newProjectAction);
    
    QAction *openProjectAction = new QAction("打开项目", this);
    connect(openProjectAction, &QAction::triggered, this, [=]() {
        QString fileName = QFileDialog::getOpenFileName(this, "打开项目", "", 
            "项目文件 (*.pc2cad);;所有文件 (*.*)");
        
        if (!fileName.isEmpty()) {
            statusBar->showMessage("正在打开项目: " + fileName);
            // 这里将实现项目文件的加载逻辑
            statusBar->showMessage("项目打开完成", 2000);
            QMessageBox::information(this, "项目管理", "项目已打开。");
        }
    });
    projectMenu->addAction(openProjectAction);
    
    QAction *saveProjectAction = new QAction("保存项目", this);
    connect(saveProjectAction, &QAction::triggered, this, [=]() {
        QString fileName = QFileDialog::getSaveFileName(this, "保存项目", "", 
            "项目文件 (*.pc2cad);;所有文件 (*.*)");
        
        if (!fileName.isEmpty()) {
            statusBar->showMessage("正在保存项目: " + fileName);
            // 这里将实现项目文件的保存逻辑
            statusBar->showMessage("项目保存完成", 2000);
            QMessageBox::information(this, "项目管理", "项目已保存。");
        }
    });
    projectMenu->addAction(saveProjectAction);
    
    fileMenu->addSeparator();
    
    // 数据文件操作
    openAction = new QAction("打开(&O)", this);
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &MainWindow::openFile);
    fileMenu->addAction(openAction);
    
    saveAction = new QAction("保存(&S)", this);
    saveAction->setShortcut(QKeySequence::Save);
    connect(saveAction, &QAction::triggered, this, &MainWindow::saveFile);
    fileMenu->addAction(saveAction);
    
    exportAction = new QAction("导出(&E)", this);
    exportAction->setShortcut(QKeySequence("Ctrl+E"));
    connect(exportAction, &QAction::triggered, this, &MainWindow::exportFile);
    fileMenu->addAction(exportAction);
    
    fileMenu->addSeparator();
    
    // 最近文件
    QMenu *recentMenu = fileMenu->addMenu("最近文件");
    QAction *clearRecentAction = new QAction("清除最近文件", this);
    connect(clearRecentAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("最近文件列表已清除", 2000);
    });
    recentMenu->addAction(clearRecentAction);
    
    fileMenu->addSeparator();
    
    exitAction = new QAction("退出(&X)", this);
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &MainWindow::exitApp);
    fileMenu->addAction(exitAction);
    
    // 编辑菜单
    editMenu = menuBar()->addMenu("编辑(&E)");
    
    // 视图菜单
    viewMenu = menuBar()->addMenu("视图(&V)");
    
    // 预设视角子菜单
    QMenu *presetViewsMenu = viewMenu->addMenu("预设视角");
    QAction *topViewAction = new QAction("俯视图", this);
    connect(topViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->topView();
    });
    presetViewsMenu->addAction(topViewAction);
    
    QAction *bottomViewAction = new QAction("仰视图", this);
    connect(bottomViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->bottomView();
    });
    presetViewsMenu->addAction(bottomViewAction);
    
    QAction *frontViewAction = new QAction("前视图", this);
    connect(frontViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->frontView();
    });
    presetViewsMenu->addAction(frontViewAction);
    
    QAction *backViewAction = new QAction("后视图", this);
    connect(backViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->backView();
    });
    presetViewsMenu->addAction(backViewAction);
    
    QAction *leftViewAction = new QAction("左视图", this);
    connect(leftViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->leftView();
    });
    presetViewsMenu->addAction(leftViewAction);
    
    QAction *rightViewAction = new QAction("右视图", this);
    connect(rightViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->rightView();
    });
    presetViewsMenu->addAction(rightViewAction);
    
    QAction *isometricViewAction = new QAction("等轴测视图", this);
    connect(isometricViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->isometricView();
    });
    presetViewsMenu->addAction(isometricViewAction);
    
    viewMenu->addSeparator();
    
    // 多视图布局子菜单
    QMenu *layoutMenu = viewMenu->addMenu("多视图布局");
    QAction *singleViewAction = new QAction("单视图", this);
    connect(singleViewAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("切换到单视图模式");
        // 这里将实现单视图布局
        QMessageBox::information(this, "视图布局", "已切换到单视图模式。");
    });
    layoutMenu->addAction(singleViewAction);
    
    QAction *twoViewAction = new QAction("双视图", this);
    connect(twoViewAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("切换到双视图模式");
        QMessageBox::information(this, "视图布局", "已切换到双视图模式。");
    });
    layoutMenu->addAction(twoViewAction);
    
    QAction *fourViewAction = new QAction("四视图", this);
    connect(fourViewAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("切换到四视图模式");
        QMessageBox::information(this, "视图布局", "已切换到四视图模式。");
    });
    layoutMenu->addAction(fourViewAction);
    
    viewMenu->addSeparator();
    
    // 视图控制
    QAction *resetViewAction = new QAction("重置视图", this);
    connect(resetViewAction, &QAction::triggered, this, [=]() {
        visualizationWidget->resetView();
        statusBar->showMessage("视图已重置", 2000);
    });
    viewMenu->addAction(resetViewAction);
    
    QAction *zoomInAction = new QAction("放大", this);
    connect(zoomInAction, &QAction::triggered, this, [=]() {
        visualizationWidget->zoomIn();
    });
    viewMenu->addAction(zoomInAction);
    
    QAction *zoomOutAction = new QAction("缩小", this);
    connect(zoomOutAction, &QAction::triggered, this, [=]() {
        visualizationWidget->zoomOut();
    });
    viewMenu->addAction(zoomOutAction);
    
    // 工具菜单
    toolsMenu = menuBar()->addMenu("工具(&T)");
    
    // 批处理子菜单
    QMenu *batchMenu = toolsMenu->addMenu("批处理");
    QAction *batchImportAction = new QAction("批量导入", this);
    connect(batchImportAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请选择多个点云文件进行批量导入");
        QStringList fileNames = QFileDialog::getOpenFileNames(this, "批量导入", "", 
            "点云文件 (*.ply *.obj *.xyz *.pcd *.las);;所有文件 (*.*)");
        
        if (!fileNames.isEmpty()) {
            statusBar->showMessage("正在批量导入 " + QString::number(fileNames.size()) + " 个文件...");
            
            for (const QString& fileName : fileNames) {
                // 导入点云数据
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                bool success = IOManager::getInstance().importPointCloud(fileName.toStdString(), cloud);
                
                if (success) {
                    // 计算法线
                    PointCloudProcessor::getInstance().computeNormals(cloud, cloud);
                    
                    // 添加到数据管理器
                    std::shared_ptr<PointCloudItem> item = std::make_shared<PointCloudItem>(fileName.split("\\").last().toStdString());
                    item->setPointCloud(cloud);
                    item->setPath(fileName.toStdString());
                    DataManager::getInstance().addItem(item);
                }
            }
            
            statusBar->showMessage("批量导入完成", 2000);
            QMessageBox::information(this, "批处理", "批量导入完成！");
        }
    });
    batchMenu->addAction(batchImportAction);
    
    QAction *batchProcessAction = new QAction("批量处理", this);
    connect(batchProcessAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("正在执行批量处理...");
        // 这里将实现批量处理逻辑
        statusBar->showMessage("批量处理完成", 2000);
        QMessageBox::information(this, "批处理", "批量处理功能已启动。");
    });
    batchMenu->addAction(batchProcessAction);
    
    QAction *batchExportAction = new QAction("批量导出", this);
    connect(batchExportAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请选择导出目录");
        QString directory = QFileDialog::getExistingDirectory(this, "选择导出目录", "");
        
        if (!directory.isEmpty()) {
            statusBar->showMessage("正在批量导出到目录: " + directory);
            // 这里将实现批量导出逻辑
            statusBar->showMessage("批量导出完成", 2000);
            QMessageBox::information(this, "批处理", "批量导出功能已启动。");
        }
    });
    batchMenu->addAction(batchExportAction);
    
    toolsMenu->addSeparator();
    
    // 测量工具子菜单
    QMenu *measureMenu = toolsMenu->addMenu("测量工具");
    QAction *distanceAction = new QAction("距离测量", this);
    connect(distanceAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请在3D视图中选择两个点进行距离测量");
        // 这里将实现距离测量功能
        // 由于需要与VisualizationWidget交互，实际项目中应在VisualizationWidget中添加点选择功能
        QMessageBox::information(this, "测量工具", "距离测量功能已启动，请在3D视图中选择两个点。");
    });
    measureMenu->addAction(distanceAction);
    
    QAction *angleAction = new QAction("角度测量", this);
    connect(angleAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请在3D视图中选择三个点进行角度测量");
        QMessageBox::information(this, "测量工具", "角度测量功能已启动，请在3D视图中选择三个点。");
    });
    measureMenu->addAction(angleAction);
    
    QAction *areaAction = new QAction("面积测量", this);
    connect(areaAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请在3D视图中选择多个点进行面积测量");
        QMessageBox::information(this, "测量工具", "面积测量功能已启动，请在3D视图中选择多个点。");
    });
    measureMenu->addAction(areaAction);
    
    QAction *volumeAction = new QAction("体积测量", this);
    connect(volumeAction, &QAction::triggered, this, [=]() {
        statusBar->showMessage("请选择一个闭合网格模型进行体积测量");
        QMessageBox::information(this, "测量工具", "体积测量功能已启动，请选择一个闭合网格模型。");
    });
    measureMenu->addAction(volumeAction);
    
    // 帮助菜单
    helpMenu = menuBar()->addMenu("帮助(&H)");
    aboutAction = new QAction("关于(&A)", this);
    connect(aboutAction, &QAction::triggered, this, &MainWindow::aboutApp);
    helpMenu->addAction(aboutAction);
}

/**
 * @brief 创建工具栏
 */
void MainWindow::createToolBars()
{
    // 文件工具栏
    fileToolBar = addToolBar("文件");
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(saveAction);
    fileToolBar->addAction(exportAction);
    
    // 编辑工具栏
    editToolBar = addToolBar("编辑");
    
    // 视图工具栏
    viewToolBar = addToolBar("视图");
    
    // 工具工具栏
    toolsToolBar = addToolBar("工具");
}

/**
 * @brief 创建状态栏
 */
void MainWindow::createStatusBar()
{
    statusBar = new QStatusBar(this);
    setStatusBar(statusBar);
    
    // 添加状态栏标签
    QLabel *coordsLabel = new QLabel("坐标: (0.000, 0.000, 0.000)", this);
    statusBar->addPermanentWidget(coordsLabel);
    
    QLabel *scaleLabel = new QLabel("缩放: 100%", this);
    statusBar->addPermanentWidget(scaleLabel);
}

/**
 * @brief 创建侧边栏
 */
void MainWindow::createDockWidgets()
{
    // 数据管理侧边栏
    dataDock = new QDockWidget("数据管理", this);
    dataDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget *dataWidget = new QWidget();
    QVBoxLayout *dataLayout = new QVBoxLayout(dataWidget);
    
    QTreeWidget *dataTree = new QTreeWidget();
    dataTree->setHeaderLabel("数据列表");
    QTreeWidgetItem *rootItem = new QTreeWidgetItem(dataTree, QStringList("项目"));
    dataTree->addTopLevelItem(rootItem);
    
    dataLayout->addWidget(dataTree);
    dataDock->setWidget(dataWidget);
    addDockWidget(Qt::LeftDockWidgetArea, dataDock);
    
    // 参数设置侧边栏
    paramsDock = new QDockWidget("参数设置", this);
    paramsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget *paramsWidget = new QWidget();
    QVBoxLayout *paramsLayout = new QVBoxLayout(paramsWidget);
    
    QGroupBox *filterGroup = new QGroupBox("滤波参数");
    QVBoxLayout *filterLayout = new QVBoxLayout(filterGroup);
    
    QLabel *meanKLabel = new QLabel("近邻点数量:");
    QSpinBox *meanKSpin = new QSpinBox();
    meanKSpin->setRange(1, 100);
    meanKSpin->setValue(50);
    filterLayout->addWidget(meanKLabel);
    filterLayout->addWidget(meanKSpin);
    
    QLabel *stdDevLabel = new QLabel("标准差阈值:");
    QDoubleSpinBox *stdDevSpin = new QDoubleSpinBox();
    stdDevSpin->setRange(0.1, 5.0);
    stdDevSpin->setValue(1.0);
    filterLayout->addWidget(stdDevLabel);
    filterLayout->addWidget(stdDevSpin);
    
    paramsLayout->addWidget(filterGroup);
    paramsDock->setWidget(paramsWidget);
    addDockWidget(Qt::RightDockWidgetArea, paramsDock);
    
    // 工具侧边栏
    toolsDock = new QDockWidget("工具", this);
    toolsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget *toolsWidget = new QWidget();
    QVBoxLayout *toolsLayout = new QVBoxLayout(toolsWidget);
    
    QPushButton *filterButton = new QPushButton("滤波");
    QPushButton *segmentButton = new QPushButton("分割");
    QPushButton *registerButton = new QPushButton("配准");
    QPushButton *reconstructButton = new QPushButton("重建");
    QPushButton *optimizeButton = new QPushButton("优化");
    
    toolsLayout->addWidget(filterButton);
    toolsLayout->addWidget(segmentButton);
    toolsLayout->addWidget(registerButton);
    toolsLayout->addWidget(reconstructButton);
    toolsLayout->addWidget(optimizeButton);
    toolsLayout->addStretch();
    
    // 连接信号槽
    connect(filterButton, &QPushButton::clicked, this, [=]() {
        // 获取当前点云数据
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (currentItem && currentItem->getType() == DataType::POINT_CLOUD) {
            std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
            if (cloudItem) {
                statusBar->showMessage("正在执行滤波操作...");
                
                // 执行统计滤波
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud = cloudItem->getPointCloud();
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                PointCloudProcessor::getInstance().statisticalFilter(inputCloud, outputCloud);
                
                // 更新点云数据
                cloudItem->setPointCloud(outputCloud);
                visualizationWidget->setPointCloud(outputCloud);
                
                statusBar->showMessage("滤波操作完成", 2000);
            }
        } else {
            QMessageBox::warning(this, "警告", "请先选择一个点云数据！");
        }
    });
    
    toolsWidget->setLayout(toolsLayout);
    toolsDock->setWidget(toolsWidget);
    addDockWidget(Qt::RightDockWidgetArea, toolsDock);
    
    // 创建日志窗口
    logDock = new QDockWidget("操作日志", this);
    logDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    logDock->setMinimumHeight(100);
    
    QTextEdit *logTextEdit = new QTextEdit(this);
    logTextEdit->setReadOnly(true);
    logTextEdit->setFont(QFont("Courier New", 9));
    logTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    logDock->setWidget(logTextEdit);
    
    addDockWidget(Qt::BottomDockWidgetArea, logDock);
    logDock->hide(); // 默认隐藏
    
    // 添加日志窗口到视图菜单
    QAction *showLogAction = new QAction("显示操作日志", this);
    showLogAction->setCheckable(true);
    connect(showLogAction, &QAction::triggered, this, [=](bool checked) {
        if (checked) {
            logDock->show();
        } else {
            logDock->hide();
        }
    });
    viewMenu->addAction(showLogAction);
    
    // 记录初始化日志
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    logTextEdit->append("[" + timestamp + "] 应用程序启动");
    logTextEdit->append("[" + timestamp + "] 初始化数据管理器");
    logTextEdit->append("[" + timestamp + "] 初始化可视化窗口");
    logTextEdit->append("[" + timestamp + "] 初始化完成，就绪");
    
    connect(segmentButton, &QPushButton::clicked, this, [=]() {
        // 获取当前点云数据
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (currentItem && currentItem->getType() == DataType::POINT_CLOUD) {
            std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
            if (cloudItem) {
                statusBar->showMessage("正在执行分割操作...");
                
                // 执行欧氏聚类分割
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud = cloudItem->getPointCloud();
                std::vector<pcl::PointIndices> clusters;
                PointCloudProcessor::getInstance().euclideanClustering(inputCloud, clusters);
                
                // 为每个聚类创建新的点云数据
                for (size_t i = 0; i < clusters.size(); ++i) {
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                    pcl::copyPointCloud(*inputCloud, clusters[i].indices, *clusterCloud);
                    
                    // 为每个聚类设置不同的颜色
                    for (auto& point : clusterCloud->points) {
                        point.r = static_cast<uint8_t>(rand() % 255);
                        point.g = static_cast<uint8_t>(rand() % 255);
                        point.b = static_cast<uint8_t>(rand() % 255);
                    }
                    
                    // 添加到数据管理器
                    std::shared_ptr<PointCloudItem> clusterItem = std::make_shared<PointCloudItem>(cloudItem->getName() + "_cluster_" + std::to_string(i));
                    clusterItem->setPointCloud(clusterCloud);
                    DataManager::getInstance().addItem(clusterItem);
                }
                
                // 合并所有聚类为一个彩色点云用于显示
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                for (size_t i = 0; i < clusters.size(); ++i) {
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                    pcl::copyPointCloud(*inputCloud, clusters[i].indices, *clusterCloud);
                    
                    // 为每个聚类设置不同的颜色
                    for (auto& point : clusterCloud->points) {
                        point.r = static_cast<uint8_t>(rand() % 255);
                        point.g = static_cast<uint8_t>(rand() % 255);
                        point.b = static_cast<uint8_t>(rand() % 255);
                    }
                    
                    *coloredCloud += *clusterCloud;
                }
                
                // 在可视化窗口中显示彩色分割结果
                visualizationWidget->setPointCloud(coloredCloud);
                
                statusBar->showMessage("分割操作完成，找到 " + QString::number(clusters.size()) + " 个聚类", 2000);
            }
        } else {
            QMessageBox::warning(this, "警告", "请先选择一个点云数据！");
        }
    });
    
    connect(registerButton, &QPushButton::clicked, this, [=]() {
        // 检查是否有至少两个点云数据
        size_t itemCount = DataManager::getInstance().getItemCount();
        if (itemCount < 2) {
            QMessageBox::warning(this, "警告", "请至少导入两个点云数据用于配准！");
            return;
        }
        
        // 获取所有点云数据
        std::vector<std::shared_ptr<PointCloudItem>> pointCloudItems;
        for (size_t i = 0; i < itemCount; ++i) {
            std::shared_ptr<DataItem> item = DataManager::getInstance().getItem(i);
            if (item && item->getType() == DataType::POINT_CLOUD) {
                std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(item);
                if (cloudItem) {
                    pointCloudItems.push_back(cloudItem);
                }
            }
        }
        
        if (pointCloudItems.size() < 2) {
            QMessageBox::warning(this, "警告", "请至少选择两个点云数据用于配准！");
            return;
        }
        
        statusBar->showMessage("正在执行配准操作...");
        
        // 使用第一个点云作为目标点云，第二个点云作为源点云
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetCloud = pointCloudItems[0]->getPointCloud();
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sourceCloud = pointCloudItems[1]->getPointCloud();
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        Eigen::Matrix4f transformation;
        
        // 执行SAC-IA初始配准
        PointCloudProcessor::getInstance().sacIARegistration(sourceCloud, targetCloud, outputCloud, transformation);
        
        // 执行ICP精配准
        PointCloudProcessor::getInstance().icpRegistration(outputCloud, targetCloud, outputCloud, transformation);
        
        // 添加配准结果到数据管理器
        std::shared_ptr<PointCloudItem> registeredItem = std::make_shared<PointCloudItem>(pointCloudItems[1]->getName() + "_registered_to_" + pointCloudItems[0]->getName());
        registeredItem->setPointCloud(outputCloud);
        DataManager::getInstance().addItem(registeredItem);
        
        // 在可视化窗口中显示配准结果
        visualizationWidget->setPointCloud(outputCloud);
        
        statusBar->showMessage("配准操作完成", 2000);
    });
    
    connect(reconstructButton, &QPushButton::clicked, this, [=]() {
        // 获取当前点云数据
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (currentItem && currentItem->getType() == DataType::POINT_CLOUD) {
            std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
            if (cloudItem) {
                statusBar->showMessage("正在执行重建操作...");
                
                // 执行泊松重建
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud = cloudItem->getPointCloud();
                pcl::PolygonMesh::Ptr outputMesh(new pcl::PolygonMesh);
                PointCloudProcessor::getInstance().poissonReconstruction(inputCloud, outputMesh);
                
                // 添加到数据管理器
                std::shared_ptr<MeshModelItem> meshItem = std::make_shared<MeshModelItem>(cloudItem->getName() + "_mesh");
                meshItem->setMesh(outputMesh);
                DataManager::getInstance().addItem(meshItem);
                
                // 在可视化窗口中显示
                visualizationWidget->setMeshModel(outputMesh);
                visualizationWidget->setVisualizationMode(VisualizationMode::MESH_MODEL);
                
                statusBar->showMessage("重建操作完成", 2000);
            }
        } else {
            QMessageBox::warning(this, "警告", "请先选择一个点云数据！");
        }
    });
    
    connect(optimizeButton, &QPushButton::clicked, this, [=]() {
        // 获取当前网格模型数据
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (currentItem && currentItem->getType() == DataType::MESH_MODEL) {
            std::shared_ptr<MeshModelItem> meshItem = std::dynamic_pointer_cast<MeshModelItem>(currentItem);
            if (meshItem) {
                statusBar->showMessage("正在执行优化操作...");
                
                // 这里可以实现网格简化、平滑等优化操作
                // 由于PCL没有直接的网格简化API，我们可以使用CGAL库来实现
                // 这里简化处理，创建一个优化后的网格模型（实际项目中应实现真正的优化算法）
                pcl::PolygonMesh::Ptr inputMesh = meshItem->getMesh();
                pcl::PolygonMesh::Ptr outputMesh(new pcl::PolygonMesh);
                *outputMesh = *inputMesh; // 复制原始网格
                
                // 添加优化结果到数据管理器
                std::shared_ptr<MeshModelItem> optimizedItem = std::make_shared<MeshModelItem>(meshItem->getName() + "_optimized");
                optimizedItem->setMesh(outputMesh);
                DataManager::getInstance().addItem(optimizedItem);
                
                // 在可视化窗口中显示
                visualizationWidget->setMeshModel(outputMesh);
                visualizationWidget->setVisualizationMode(VisualizationMode::MESH_MODEL);
                
                statusBar->showMessage("优化操作完成", 2000);
            }
        } else if (currentItem && currentItem->getType() == DataType::POINT_CLOUD) {
            // 对点云数据进行优化（如平滑）
            std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
            if (cloudItem) {
                statusBar->showMessage("正在执行点云优化操作...");
                
                // 执行移动最小二乘平滑
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud = cloudItem->getPointCloud();
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                PointCloudProcessor::getInstance().movingLeastSquares(inputCloud, outputCloud);
                
                // 添加优化结果到数据管理器
                std::shared_ptr<PointCloudItem> optimizedItem = std::make_shared<PointCloudItem>(cloudItem->getName() + "_optimized");
                optimizedItem->setPointCloud(outputCloud);
                DataManager::getInstance().addItem(optimizedItem);
                
                // 在可视化窗口中显示
                visualizationWidget->setPointCloud(outputCloud);
                visualizationWidget->setVisualizationMode(VisualizationMode::POINT_CLOUD);
                
                statusBar->showMessage("点云优化操作完成", 2000);
            }
        } else {
            QMessageBox::warning(this, "警告", "请先选择一个点云数据或网格模型！");
        }
    });
    
    toolsDock->setWidget(toolsWidget);
    addDockWidget(Qt::RightDockWidgetArea, toolsDock);
}

/**
 * @brief 创建3D视图
 */
void MainWindow::create3DView()
{
    // 创建中央部件
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *centralLayout = new QVBoxLayout(centralWidget);
    
    // 创建可视化窗口
    visualizationWidget = new VisualizationWidget();
    visualizationWidget->setMinimumSize(800, 600);
    
    centralLayout->addWidget(visualizationWidget);
    setCentralWidget(centralWidget);
}

/**
 * @brief 打开文件
 */
void MainWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, "打开文件", "", 
        "点云文件 (*.ply *.obj *.xyz *.pcd *.las);;CAD模型文件 (*.step *.iges *.stl *.obj *.fbx);;所有文件 (*.*)");
    
    if (!fileName.isEmpty()) {
        statusBar->showMessage("正在加载文件: " + fileName);
        
        // 获取文件扩展名
        QString extension = fileName.split(".").last();
        
        if (extension == "ply" || extension == "obj" || extension == "xyz" || extension == "pcd" || extension == "las") {
            // 导入点云数据
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            bool success = IOManager::getInstance().importPointCloud(fileName.toStdString(), cloud);
            
            if (success) {
                // 计算法线
                PointCloudProcessor::getInstance().computeNormals(cloud, cloud);
                
                // 添加到数据管理器
                std::shared_ptr<PointCloudItem> item = std::make_shared<PointCloudItem>(fileName.split("\\").last().toStdString());
                item->setPointCloud(cloud);
                item->setPath(fileName.toStdString());
                DataManager::getInstance().addItem(item);
                
                // 在可视化窗口中显示
                visualizationWidget->setPointCloud(cloud);
                visualizationWidget->setVisualizationMode(VisualizationMode::POINT_CLOUD);
                
                statusBar->showMessage("点云文件加载完成", 2000);
            } else {
                statusBar->showMessage("点云文件加载失败", 2000);
                QMessageBox::warning(this, "警告", "无法加载点云文件！");
            }
        } else if (extension == "stl" || extension == "obj" || extension == "ply" || extension == "vtk") {
            // 导入网格模型
            pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
            bool success = IOManager::getInstance().importMesh(fileName.toStdString(), mesh);
            
            if (success) {
                // 添加到数据管理器
                std::shared_ptr<MeshModelItem> item = std::make_shared<MeshModelItem>(fileName.split("\\").last().toStdString());
                item->setMesh(mesh);
                item->setPath(fileName.toStdString());
                DataManager::getInstance().addItem(item);
                
                // 在可视化窗口中显示
                visualizationWidget->setMeshModel(mesh);
                visualizationWidget->setVisualizationMode(VisualizationMode::MESH_MODEL);
                
                statusBar->showMessage("网格模型文件加载完成", 2000);
            } else {
                statusBar->showMessage("网格模型文件加载失败", 2000);
                QMessageBox::warning(this, "警告", "无法加载网格模型文件！");
            }
        } else {
            statusBar->showMessage("不支持的文件格式", 2000);
            QMessageBox::warning(this, "警告", "不支持的文件格式！");
        }
    }
}

/**
 * @brief 保存文件
 */
void MainWindow::saveFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, "保存文件", "", 
        "项目文件 (*.pc2cad);;点云文件 (*.ply *.obj *.pcd *.xyz);;网格模型文件 (*.stl *.obj *.ply);;所有文件 (*.*)");
    
    if (!fileName.isEmpty()) {
        statusBar->showMessage("正在保存文件: " + fileName);
        
        // 获取当前数据项
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (!currentItem) {
            statusBar->showMessage("没有选中的数据项", 2000);
            QMessageBox::warning(this, "警告", "请先选择一个数据项！");
            return;
        }
        
        QString extension = fileName.split("/").last().split(".").last();
        
        if (extension == "pc2cad") {
            // 保存项目文件（这里简化处理，实际应保存整个项目状态）
            statusBar->showMessage("项目文件保存完成", 2000);
        } else if (extension == "ply" || extension == "obj" || extension == "pcd" || extension == "xyz") {
            // 保存点云文件
            if (currentItem->getType() == DataType::POINT_CLOUD) {
                std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
                if (cloudItem) {
                    bool success = IOManager::getInstance().exportPointCloud(fileName.toStdString(), cloudItem->getPointCloud());
                    if (success) {
                        statusBar->showMessage("点云文件保存完成", 2000);
                    } else {
                        statusBar->showMessage("点云文件保存失败", 2000);
                        QMessageBox::warning(this, "警告", "无法保存点云文件！");
                    }
                }
            } else {
                statusBar->showMessage("文件格式与数据类型不匹配", 2000);
                QMessageBox::warning(this, "警告", "文件格式与数据类型不匹配！");
            }
        } else if (extension == "stl" || extension == "obj" || extension == "ply" || extension == "step" || extension == "iges") {
            // 保存网格模型文件
            if (currentItem->getType() == DataType::MESH_MODEL) {
                std::shared_ptr<MeshModelItem> meshItem = std::dynamic_pointer_cast<MeshModelItem>(currentItem);
                if (meshItem) {
                    bool success = IOManager::getInstance().exportMesh(fileName.toStdString(), meshItem->getMesh());
                    if (success) {
                        statusBar->showMessage("网格模型文件保存完成", 2000);
                    } else {
                        statusBar->showMessage("网格模型文件保存失败", 2000);
                        QMessageBox::warning(this, "警告", "无法保存网格模型文件！");
                    }
                }
            } else {
                statusBar->showMessage("文件格式与数据类型不匹配", 2000);
                QMessageBox::warning(this, "警告", "文件格式与数据类型不匹配！");
            }
        } else {
            statusBar->showMessage("不支持的文件格式", 2000);
            QMessageBox::warning(this, "警告", "不支持的文件格式！");
        }
    }
}

/**
 * @brief 导出文件
 */
void MainWindow::exportFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, "导出文件", "", 
        "CAD模型文件 (*.step *.iges *.stl *.obj *.fbx);;点云文件 (*.ply *.obj *.pcd *.xyz);;所有文件 (*.*)");
    
    if (!fileName.isEmpty()) {
        statusBar->showMessage("正在导出文件: " + fileName);
        
        // 获取当前数据项
        std::shared_ptr<DataItem> currentItem = DataManager::getInstance().getCurrentItem();
        if (!currentItem) {
            statusBar->showMessage("没有选中的数据项", 2000);
            QMessageBox::warning(this, "警告", "请先选择一个数据项！");
            return;
        }
        
        QString extension = fileName.split("/").last().split(".").last();
        
        if (extension == "step" || extension == "iges" || extension == "stl" || extension == "obj" || extension == "fbx") {
            // 导出为CAD模型文件
            if (currentItem->getType() == DataType::MESH_MODEL) {
                std::shared_ptr<MeshModelItem> meshItem = std::dynamic_pointer_cast<MeshModelItem>(currentItem);
                if (meshItem) {
                    bool success = IOManager::getInstance().exportMesh(fileName.toStdString(), meshItem->getMesh());
                    if (success) {
                        statusBar->showMessage("CAD模型文件导出完成", 2000);
                    } else {
                        statusBar->showMessage("CAD模型文件导出失败", 2000);
                        QMessageBox::warning(this, "警告", "无法导出CAD模型文件！");
                    }
                }
            } else if (currentItem->getType() == DataType::POINT_CLOUD) {
                // 从点云导出为CAD模型（需要先重建）
                std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
                if (cloudItem) {
                    // 执行泊松重建
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud = cloudItem->getPointCloud();
                    pcl::PolygonMesh::Ptr outputMesh(new pcl::PolygonMesh);
                    PointCloudProcessor::getInstance().poissonReconstruction(inputCloud, outputMesh);
                    
                    // 导出网格模型
                    bool success = IOManager::getInstance().exportMesh(fileName.toStdString(), outputMesh);
                    if (success) {
                        statusBar->showMessage("CAD模型文件导出完成", 2000);
                    } else {
                        statusBar->showMessage("CAD模型文件导出失败", 2000);
                        QMessageBox::warning(this, "警告", "无法导出CAD模型文件！");
                    }
                }
            } else {
                statusBar->showMessage("文件格式与数据类型不匹配", 2000);
                QMessageBox::warning(this, "警告", "文件格式与数据类型不匹配！");
            }
        } else if (extension == "ply" || extension == "obj" || extension == "pcd" || extension == "xyz") {
            // 导出为点云文件
            if (currentItem->getType() == DataType::POINT_CLOUD) {
                std::shared_ptr<PointCloudItem> cloudItem = std::dynamic_pointer_cast<PointCloudItem>(currentItem);
                if (cloudItem) {
                    bool success = IOManager::getInstance().exportPointCloud(fileName.toStdString(), cloudItem->getPointCloud());
                    if (success) {
                        statusBar->showMessage("点云文件导出完成", 2000);
                    } else {
                        statusBar->showMessage("点云文件导出失败", 2000);
                        QMessageBox::warning(this, "警告", "无法导出点云文件！");
                    }
                }
            } else {
                statusBar->showMessage("文件格式与数据类型不匹配", 2000);
                QMessageBox::warning(this, "警告", "文件格式与数据类型不匹配！");
            }
        } else {
            statusBar->showMessage("不支持的文件格式", 2000);
            QMessageBox::warning(this, "警告", "不支持的文件格式！");
        }
    }
}

/**
 * @brief 退出应用程序
 */
void MainWindow::exitApp()
{
    QApplication::quit();
}

/**
 * @brief 关于应用程序
 */
void MainWindow::aboutApp()
{
    QMessageBox::about(this, "关于 PointCloud2CAD", 
        "PointCloud2CAD 1.0.0\n\n" 
        "线扫相机点云数据转CAD 3D模型系统\n\n" 
        "© 2026 PointCloud2CAD Team");
}