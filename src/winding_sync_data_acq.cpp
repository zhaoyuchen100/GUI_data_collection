#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <include/gui_data_collection/EmgStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <QtCore/QDebug>
#include <geometry_msgs/TransformStamped.h>
#include <gui_data_collection/Int32Stamped.h>
#include <sensor_msgs/Imu.h>
#include <QtCore/QObject>

using namespace message_filters;
geometry_msgs::TransformStamped msg_hand_poses;
sensor_msgs::Imu msg_imu_o;
ros::Publisher tension_pub;
rosbag::Bag bag;
//ros::Publisher pub;
void loop_rate_calculate()
{
    static int i;
    static double sampl_counter;
    static ros::Time begin;
    if(i==0)
    {
        sampl_counter ++;
        begin = ros::Time::now();
        i++;
    }
    else
    {
        sampl_counter ++;
        ros::Time now = ros::Time::now();
        if (now.toSec()-begin.toSec()>ros::Duration(10).toSec())
        {
            //qDebug()<< now.toNSec();
            qDebug()<< "sampling rate= :" << sampl_counter/((now.toNSec()-begin.toNSec())/1e+9);
            qDebug()<<sampl_counter;
            begin = now;
            sampl_counter = 0;
        }
    }

}

void sync_tension_data_cb(const ros_myo_rv::EmgStampedConstPtr& msg_white,const gui_data_collection::Int32StampedConstPtr& msg_tension)
{
    ros::Time now_ = ros::Time::now();
    bag.write<ros_myo_rv::EmgStamped>("/myo_white_emg",now_,msg_white);
    bag.write<gui_data_collection::Int32Stamped>("/tension_stamped",now_,msg_tension);
    //bag.write<geometry_msgs::TransformStamped>("/vicon/hand/hand",now_,msg_hand_poses);
    bag.write<sensor_msgs::Imu>("orange_imu",now_,msg_imu_o);
    loop_rate_calculate();
}
void sync_tension_data_cb2(const ros_myo_rv::EmgStampedConstPtr& msg_white,const ros_myo_rv::EmgStampedConstPtr& msg_black,const gui_data_collection::Int32StampedConstPtr& msg_tension)
{
    ros::Time now_ = ros::Time::now();
    bag.write<ros_myo_rv::EmgStamped>("/myo_white_emg",now_,msg_white);
    bag.write<ros_myo_rv::EmgStamped>("/myo_black_emg",now_,msg_black);
    bag.write<gui_data_collection::Int32Stamped>("/tension_stamped",now_,msg_tension);
    //bag.write<geometry_msgs::TransformStamped>("/vicon/hand/hand",now_,msg_hand_poses);
    bag.write<sensor_msgs::Imu>("orange_imu",now_,msg_imu_o);
    loop_rate_calculate();
}

void DoViconHandBagWork(const geometry_msgs::TransformStampedConstPtr& msg_vicon_hand)
{
    msg_hand_poses.transform = msg_vicon_hand.get()->transform;
}
void tension_msg_cb(const std_msgs::Int32ConstPtr& tension_msg)
{
    gui_data_collection::Int32Stamped tensionStemped_msg;
    tensionStemped_msg.header.stamp = ros::Time::now();
    tensionStemped_msg.tension.data = tension_msg.get()->data;
    tension_pub.publish(tensionStemped_msg);
}
void IMU_cb(const sensor_msgs::ImuConstPtr& msg_imu_orange)
{
    msg_imu_o.orientation = msg_imu_orange.get()->orientation;
    msg_imu_o.linear_acceleration = msg_imu_orange.get()->linear_acceleration;
    msg_imu_o.angular_velocity = msg_imu_orange.get()->angular_velocity;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"imu_sync_data_pub");
    ros::NodeHandle n;
    QString BAG_filename;
    BAG_filename = QString(argv[2]);
    message_filters::Subscriber<ros_myo_rv::EmgStamped> sub_white(n,"myo_white_emg",1);
    message_filters::Subscriber<ros_myo_rv::EmgStamped> sub_black(n,"myo_black_emg",1);
    message_filters::Subscriber<gui_data_collection::Int32Stamped> sub_tension(n,"tension_stamped",1);
    //ros::Subscriber vicon_hand_sub = n.subscribe<geometry_msgs::TransformStamped>("/vicon/hand/hand",1,DoViconHandBagWork);
    ros::Subscriber tension_sub = n.subscribe<std_msgs::Int32>("/tension",1,tension_msg_cb);
    ros::Subscriber Imu_sub = n.subscribe<sensor_msgs::Imu>("/orange_imu",1,IMU_cb);
    tension_pub = n.advertise<gui_data_collection::Int32Stamped>("/tension_stamped",1);
    ros::Rate loop_rate(1500);
    bag.open(BAG_filename.append(".bag").toStdString().c_str(),rosbag::bagmode::Write);// reopen bag will generate SIGNET problem. try not to reopen it or close it before doing that.
    typedef sync_policies::ApproximateTime<ros_myo_rv::EmgStamped, gui_data_collection::Int32Stamped> MySyncPolicy;
    typedef sync_policies::ApproximateTime<ros_myo_rv::EmgStamped, ros_myo_rv::EmgStamped, gui_data_collection::Int32Stamped> MySyncPolicy2;
    typedef boost::shared_ptr<Synchronizer<MySyncPolicy> > sync_ptr;
    typedef boost::shared_ptr<Synchronizer<MySyncPolicy2> > sync_ptr2;
    sync_ptr sync;
    sync_ptr2 sync2;
    if (QString(argv[1]).compare(QString("1"))==0)
    {
        sync_ptr snyc_tmp(new Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_white, sub_tension));
        sync = snyc_tmp;
        sync.get()->registerCallback(boost::bind(sync_tension_data_cb, _1, _2));
        qDebug()<< "only one MYO in use!";
    }
    else
    {
        sync_ptr2 snyc_tmp(new Synchronizer<MySyncPolicy2>(MySyncPolicy2(20), sub_white, sub_black, sub_tension));
        sync2 = snyc_tmp;
        sync2.get()->registerCallback(boost::bind(sync_tension_data_cb2, _1, _2, _3));
        qDebug()<< "Two MYO sin use!";
    }

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    bag.close();
}
