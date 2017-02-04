#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLCDNumber>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <include/gui_data_collection/qcustomplot.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <gui_data_collection/Bag_info.h>
#include <gui_data_collection/ref_info.h>
#include <../../build/gui_data_collection/ui_data_log_Widget.h>

using namespace gui_data_collection;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void customplot_setup();
    void plotFTdataCB(geometry_msgs::WrenchStampedConstPtr msg);
    void getNumSamplCB(std_msgs::Int32ConstPtr msg);
    void plot_ref_signal(std_msgs::Float64ConstPtr msg);
    void init();
    void loop_rate_calculate();

signals:
    void start_update_plot();

public slots:
       void on_customPlot_start_update_plot();

private slots:
    void on_pushButton_Start_clicked();

    void on_pushButton_Stop_clicked();

    void on_progressBar_valueChanged(int value);

    void set_countdown_display();

    void on_ros_spin();

    void on_pushButton_bias_clicked();

    void on_pushButton_add_generator_clicked();

    void on_pushButton_remove_generator_clicked();

    void on_checkBox_ALL_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);

    void on_bag_record_clicked();

private:
    Ui::MainWindow *ui;
    int num_data_record;
    int trial_counter;
    QString Primitive_mode;
    double lastPointKey;
    double num_sampls;


public:
    QTimer* countdown_timer;
    QTimer* ros_timer;
    int countdown_value;
    int sample_count;
    geometry_msgs::WrenchStamped ft_msg_plot;
    Bag_info bag_srv_param;
    ref_info ref_srv_param;
protected:
     ros::Subscriber ft_sub;
     ros::Subscriber ref_sub;
     ros::Subscriber num_sampl_sub;
     ros::ServiceClient bag_client;
     ros::ServiceClient ref_client;
     ros::NodeHandle n_;
     geometry_msgs::WrenchStamped ft_msg_ref;
};

#endif // MAINWINDOW_H
