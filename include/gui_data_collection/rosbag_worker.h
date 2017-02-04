#ifndef ROSBAG_WORKER_H
#define ROSBAG_WORKER_H

#include <QtCore/QObject>
#include <QtCore/QThread>
#include <QtCore/QDebug>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <QtCore/QTimer>
#include <include/gui_data_collection/EmgStamped.h>
#include <include/gui_data_collection/TfArrayStamped.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gui_data_collection/Bag_info.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ecl/threads.hpp>
#include <iostream>
#include <exception>
using namespace gui_data_collection;
using namespace message_filters;
using ecl::Mutex;
typedef sync_policies::ApproximateTime<ros_myo_rv::EmgStamped, ros_myo_rv::EmgStamped,geometry_msgs::WrenchStamped> MySyncPolicy;
typedef sync_policies::ApproximateTime<ros_myo_rv::EmgStamped, ros_myo_rv::EmgStamped> MySyncPolicy_2;
typedef boost::shared_ptr<Synchronizer<MySyncPolicy> > sync_ptr;
typedef boost::shared_ptr<Synchronizer<MySyncPolicy_2> > sync_ptr_2;
typedef boost::shared_ptr<ros::Rate> loop_rate_ptr;

class rosbag_worker : public QObject
{
    Q_OBJECT
public:
    rosbag::Bag bag;
    QString BAG_filename;
    std::vector<int64_t> bag_op;
    geometry_msgs::WrenchStamped ft_msg_ref;
    int sampl_count;
    int goal;
public:
    explicit rosbag_worker(QObject *parent = 0);
    ~rosbag_worker();
    void DoSetup();

    //ft_revieved(geometry_msgs::WrenchStamped ft_msgs);
    bool param_setup_service(Bag_info::Request& req, Bag_info::Response& res);
    void DoFTBagWork(geometry_msgs::WrenchStampedConstPtr msg);
    void DoWhiteIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_white);
    void DoBlackIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_black);
    void DoOrangeIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_orange);
    void DoViconHandBagWork(const geometry_msgs::TransformStampedConstPtr& msg_vicon_hand);
    void DoMyoWhiteBagWork(const geometry_msgs::TransformStampedConstPtr& msg_myo_w);
    void DoMyoBlackBagWork(const geometry_msgs::TransformStampedConstPtr& msg_myo_b);
    void DojointtfsBagWork(const gui_data_collection::TfArrayStampedConstPtr& msg_joint_tf_sub);
    void ft_myos_cb(const ros_myo_rv::EmgStampedConstPtr& msg1,const ros_myo_rv::EmgStampedConstPtr& msg2,const geometry_msgs::WrenchStampedConstPtr& msg3);
    void myos_cb(const ros_myo_rv::EmgStampedConstPtr& msg1,const ros_myo_rv::EmgStampedConstPtr& msg2);
    void StopWork();
    void loop_rate_calculate();
    void ft_msg_reset(const geometry_msgs::WrenchStampedConstPtr& msg, geometry_msgs::WrenchStamped ref_msg,geometry_msgs::WrenchStamped &ft_msg);
private:
    //bool run_flag;
    ros::NodeHandle n_;
    ros::ServiceServer service;
    ros::Subscriber white_imu_sub;
    ros::Subscriber black_imu_sub;
    ros::Subscriber orange_imu_sub;
    ros::Subscriber vicon_hand_sub;
    ros::Subscriber vicon_myo_w_sub;
    ros::Subscriber vicon_myo_b_sub;
    ros::Subscriber joint_tfs_sub;
    ros::Publisher num_smpl_pub;
    geometry_msgs::WrenchStamped ft_msg;
    message_filters::Subscriber<ros_myo_rv::EmgStamped> emg_black_sub;
    message_filters::Subscriber<ros_myo_rv::EmgStamped> emg_white_sub;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> ft_sub;
    sync_ptr sync;
    sync_ptr_2 sync_2;
    bool stop_bag;
    bool is_finish;
    //loop_rate_ptr loop_rate;
    sensor_msgs::Imu msg_imu_w;
    sensor_msgs::Imu msg_imu_b;
    sensor_msgs::Imu msg_imu_o;
    gui_data_collection::TfArrayStamped msg_joint_tfs;
    geometry_msgs::TransformStamped msg_hand_poses;
    geometry_msgs::TransformStamped msg_myo_w_poses;
    geometry_msgs::TransformStamped msg_myo_b_poses;
};

#endif // ROSBAG_WORKER_H
