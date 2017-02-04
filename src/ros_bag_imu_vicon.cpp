#include <cmath>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <QtCore/QDebug>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_w;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_b;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_o;
rosbag::Bag bag;
ros::Time now_;
static int count=0;
void sync_white_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_white, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    count++;
    now_ = ros::Time::now();
    bag.write<sensor_msgs::Imu>("/myo_white_imu",now_,msg_white);
    bag.write<geometry_msgs::TransformStamped>("/vicon/myo_w/myo_w",now_,vicon_msg);
    //qDebug()<<"white imu"<<count;
}
void sync_black_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_black, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    count++;
    now_ = ros::Time::now();
    bag.write<sensor_msgs::Imu>("/myo_black_imu",now_,msg_black);
    bag.write<geometry_msgs::TransformStamped>("/vicon/myo_b/myo_b",now_,vicon_msg);
    //qDebug()<<"black imu"<<count;
}

void sync_orange_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_orange, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    count++;
    now_ = ros::Time::now();
    bag.write<sensor_msgs::Imu>("/orange_imu",now_,msg_orange);
    bag.write<geometry_msgs::TransformStamped>("/vicon/hand/hand",now_,vicon_msg);
    //qDebug()<<"orange imu"<<count;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosbag_imu_vicon");
    ros::NodeHandle n_;
    bag.close();
    QString BAG_filename = QString(argv[1]);
    bag.open(BAG_filename.append(".bag").toStdString().c_str(),rosbag::bagmode::Write);// reopen bag will generate SIGNET problem. try not to reopen it or close it before doing that.
    qDebug()<<BAG_filename;
    message_filters::Subscriber<sensor_msgs::Imu> sub_black;
    message_filters::Subscriber<sensor_msgs::Imu> sub_white;
    message_filters::Subscriber<sensor_msgs::Imu> sub_orange;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_w;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_b;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_o;

    Synchronizer<MySyncPolicy_v_w>* sync_v_w;
    Synchronizer<MySyncPolicy_v_b>* sync_v_b;
    Synchronizer<MySyncPolicy_v_o>* sync_v_o;
    sub_black.subscribe(n_,"myo_black_imu",10);
    sub_white.subscribe(n_,"myo_white_imu",10);
    sub_orange.subscribe(n_,"orange_imu",10);
    sub_vicon_w.subscribe(n_,"vicon/myo_w/myo_w",10);
    sub_vicon_b.subscribe(n_,"vicon/myo_b/myo_b",10);
    sub_vicon_o.subscribe(n_,"vicon/hand/hand",10);
    sync_v_w = new Synchronizer<MySyncPolicy_v_w>(MySyncPolicy_v_w(10), sub_white, sub_vicon_w);
    sync_v_b = new Synchronizer<MySyncPolicy_v_b>(MySyncPolicy_v_b(10), sub_black, sub_vicon_b);
    sync_v_o = new Synchronizer<MySyncPolicy_v_o>(MySyncPolicy_v_o(10), sub_orange, sub_vicon_o);

    sync_v_w->registerCallback(boost::bind(sync_white_imu_vicon_cb, _1, _2));
    sync_v_b->registerCallback(boost::bind(sync_black_imu_vicon_cb, _1, _2));
    sync_v_o->registerCallback(boost::bind(sync_orange_imu_vicon_cb, _1, _2));
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    bag.close();
    return 0;
}
