#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QMenu>
#include <QToolBar>
#include <QDockWidget>
#include <QStatusBar>

/**
 * @brief 主窗口类
 * 包含菜单栏、工具栏、3D视图、侧边栏等组件
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口
     */
    MainWindow(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~MainWindow();

private slots:
    /**
     * @brief 打开文件
     */
    void openFile();
    
    /**
     * @brief 保存文件
     */
    void saveFile();
    
    /**
     * @brief 导出文件
     */
    void exportFile();
    
    /**
     * @brief 退出应用程序
     */
    void exitApp();
    
    /**
     * @brief 关于应用程序
     */
    void aboutApp();

private:
    /**
     * @brief 初始化UI组件
     */
    void initUI();
    
    /**
     * @brief 创建菜单栏
     */
    void createMenus();
    
    /**
     * @brief 创建工具栏
     */
    void createToolBars();
    
    /**
     * @brief 创建状态栏
     */
    void createStatusBar();
    
    /**
     * @brief 创建侧边栏
     */
    void createDockWidgets();
    
    /**
     * @brief 创建3D视图
     */
    void create3DView();

private:
    // 菜单栏
    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *viewMenu;
    QMenu *toolsMenu;
    QMenu *helpMenu;
    
    // 工具栏
    QToolBar *fileToolBar;
    QToolBar *editToolBar;
    QToolBar *viewToolBar;
    QToolBar *toolsToolBar;
    
    // 动作
    QAction *openAction;
    QAction *saveAction;
    QAction *exportAction;
    QAction *exitAction;
    QAction *aboutAction;
    
    // 侧边栏
    QDockWidget *dataDock;
    QDockWidget *paramsDock;
    QDockWidget *toolsDock;
    QDockWidget *logDock;
    
    // 状态栏
    QStatusBar *statusBar;
    
    // 可视化窗口
    VisualizationWidget *visualizationWidget;
};

#endif // MAINWINDOW_H