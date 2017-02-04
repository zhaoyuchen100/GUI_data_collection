#include <include/gui_data_collection/mainwindow.h>
#include <QtWidgets/QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "data_collection_gui");
    ros::Time::init();
    MainWindow w;
    w.show();
    return a.exec();
}
