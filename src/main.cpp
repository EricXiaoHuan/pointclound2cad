#include <QApplication>
#include "ui/MainWindow.h"

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 应用程序退出码
 */
int main(int argc, char *argv[])
{
    // 创建Qt应用程序实例
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("PointCloud2CAD");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("PointCloud2CAD Team");
    app.setOrganizationDomain("pointcloud2cad.example.com");
    
    // 创建主窗口
    MainWindow mainWindow;
    
    // 显示主窗口
    mainWindow.show();
    
    // 运行应用程序
    return app.exec();
}