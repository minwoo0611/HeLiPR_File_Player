#include "mainwindow.h"
#include <QApplication>
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include "rclcpp/rclcpp.hpp"
#include <QProcess>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("helipr_file_player");

    QApplication app(argc, argv);

    MainWindow window;
    window.RosInit(node);
    window.show();

    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return app.exec();
}