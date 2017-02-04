#include <include/gui_data_collection/rosbag_worker.h>
rosbag_worker::rosbag_worker(QObject *parent) : QObject(parent)
{
    BAG_filename = QString("FT");
    sampl_count = 0;
    ft_msg_ref.wrench.force.x = 0;
    ft_msg_ref.wrench.force.y = 0;
    ft_msg_ref.wrench.force.z = 0;
    ft_msg_ref.wrench.torque.x = 0;
    ft_msg_ref.wrench.torque.y = 0;
    ft_msg_ref.wrench.torque.z = 0;
    service = n_.advertiseService("bag_config_service",&rosbag_worker::param_setup_service,this);
    num_smpl_pub  = n_.advertise<std_msgs::Int32>("sample_recorded",1);
    stop_bag = false;
    is_finish = false;
    //loop_rate_ptr  loop_rate_tmp(new ros::Rate(200));
    //loop_rate = loop_rate_tmp;
}

rosbag_worker::~rosbag_worker()
{
    //delete mtimer;
}

void rosbag_worker::DoSetup()
{
    //initialze subsriber
    //delete sync;
    bag.close();
    sampl_count = 0;
    bag.open(BAG_filename.append(".bag").toStdString().c_str(),rosbag::bagmode::Write);// reopen bag will generate SIGNET problem. try not to reopen it or close it before doing that.
    qDebug()<<BAG_filename;
    is_finish = false;
    qDebug()<<bag_op[0]<<bag_op[1]<<bag_op[2]<<bag_op[3]<<bag_op[4]<<bag_op[5]<<bag_op[6]<<bag_op[7];

    if (bag_op[0] == 1) //FT
    {
        ft_sub.subscribe(n_,"ft_message",20);
    }
    if (bag_op[1] == 1) //MYO_BLACK
    {
        emg_black_sub.subscribe(n_,"myo_black_emg",20);
    }
    if (bag_op[2] == 1) //BLACK_IMU
    {
        black_imu_sub = n_.subscribe<sensor_msgs::Imu>("/myo_black_imu",1,&rosbag_worker::DoBlackIMUBagWork,this);
    }
    if (bag_op[3] == 1) //WHITE
    {
        emg_white_sub.subscribe(n_,"myo_white_emg",20);
    }
    if (bag_op[4] == 1) // WHITE_IMU
    {
        white_imu_sub = n_.subscribe<sensor_msgs::Imu>("/myo_white_imu",1,&rosbag_worker::DoWhiteIMUBagWork,this);
    }
    if (bag_op[5] == 1) //ORANGE
    {
        orange_imu_sub = n_.subscribe<sensor_msgs::Imu>("/orange_imu",1,&rosbag_worker::DoOrangeIMUBagWork,this);
    }
    if (bag_op[6] == 1) //VICON
    {
        vicon_hand_sub = n_.subscribe<geometry_msgs::TransformStamped>("/vicon/hand/hand",1,&rosbag_worker::DoViconHandBagWork,this);
        vicon_myo_w_sub = n_.subscribe<geometry_msgs::TransformStamped>("/vicon/myo_w/myo_w",1,&rosbag_worker::DoMyoWhiteBagWork,this);
        vicon_myo_b_sub = n_.subscribe<geometry_msgs::TransformStamped>("/vicon/myo_b/myo_b",1,&rosbag_worker::DoMyoBlackBagWork,this);
    }
    if (bag_op[7] == 1) //JOINT_TFS
    {
        joint_tfs_sub = n_.subscribe<gui_data_collection::TfArrayStamped>("/joint_tfs",1,&rosbag_worker::DojointtfsBagWork,this);
    }
    if (bag_op[0] == 1&&bag_op[1] == 1&&bag_op[3] == 1)
    {
        sync_ptr sync_tmp(new Synchronizer<MySyncPolicy>(MySyncPolicy(100), emg_black_sub, emg_white_sub,ft_sub));
        sync = sync_tmp;
        sync->registerCallback(boost::bind(&rosbag_worker::ft_myos_cb,this, _1, _2, _3));
    }
    if (bag_op[1] == 1&&bag_op[3] == 1&&bag_op[0] == 0)
    {
        sync_ptr_2 sync_tmp(new Synchronizer<MySyncPolicy_2>(MySyncPolicy_2(100), emg_black_sub, emg_white_sub));
        sync_2 = sync_tmp;
        sync_2->registerCallback(boost::bind(&rosbag_worker::myos_cb,this, _1, _2));
    }

}

bool rosbag_worker::param_setup_service(Bag_info::Request &req, Bag_info::Response &res)// be careful about the Capital spelling of Response. response will not work!
{
    BAG_filename = QString(req.bag_name.c_str());
    bag_op.resize(8);
    bag_op = req.bag_option;
    goal = req.goal;
    stop_bag = req.stop;
    ft_msg_ref= req.ft_msg_ref;

    res.ack = true;

    if (stop_bag == false)
    {
        DoSetup();
    }
    else
    {
        StopWork();
    }
    return true;
}



void rosbag_worker::DoWhiteIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_white)
{
    //qDebug()<<"i am bug white";
    msg_imu_w.orientation = msg_imu_white.get()->orientation;
    msg_imu_w.linear_acceleration = msg_imu_white.get()->linear_acceleration;
    msg_imu_w.angular_velocity = msg_imu_white.get()->angular_velocity;
}

void rosbag_worker::DoBlackIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_black)
{
    //qDebug()<<"i am bug black";
    msg_imu_b.orientation = msg_imu_black.get()->orientation;
    msg_imu_b.linear_acceleration = msg_imu_black.get()->linear_acceleration;
    msg_imu_b.angular_velocity = msg_imu_black.get()->angular_velocity;
}
void rosbag_worker::DoOrangeIMUBagWork(const sensor_msgs::ImuConstPtr& msg_imu_orange)
{
    msg_imu_o.orientation = msg_imu_orange.get()->orientation;
    msg_imu_o.linear_acceleration = msg_imu_orange.get()->linear_acceleration;
    msg_imu_o.angular_velocity = msg_imu_orange.get()->angular_velocity;
    //qDebug()<<msg_imu_o.angular_velocity.z;
}

void rosbag_worker::DoViconHandBagWork(const geometry_msgs::TransformStampedConstPtr &msg_vicon_hand)
{
    msg_hand_poses.transform=msg_vicon_hand.get()->transform;
    //qDebug()<<msg_hand_poses.transform.rotation.x;

}

void rosbag_worker::DoMyoWhiteBagWork(const geometry_msgs::TransformStampedConstPtr &msg_myo_w)
{
    msg_myo_w_poses.transform= msg_myo_w.get()->transform;
}

void rosbag_worker::DoMyoBlackBagWork(const geometry_msgs::TransformStampedConstPtr &msg_myo_b)
{
    msg_myo_b_poses.transform= msg_myo_b.get()->transform;
}
void rosbag_worker::DojointtfsBagWork(const gui_data_collection::TfArrayStampedConstPtr& msg_joint_tf_sub)
{
    //qDebug()<<"i am bug joint tfs";
    msg_joint_tfs.tfs.resize(3);
    msg_joint_tfs.tfs[0] = msg_joint_tf_sub.get()->tfs[0];
    msg_joint_tfs.tfs[1] = msg_joint_tf_sub.get()->tfs[1];
    msg_joint_tfs.tfs[2] = msg_joint_tf_sub.get()->tfs[2];
}

void rosbag_worker::myos_cb(const ros_myo_rv::EmgStampedConstPtr &msg1, const ros_myo_rv::EmgStampedConstPtr &msg2)
{
    Mutex mutex;
   try{
       mutex.lock();
       ros::Time now_ = ros::Time::now();
       if (stop_bag==true)
       {
           StopWork();
           std::cout<<"bag stop recording, terminated by user."<<std::endl;
           return;
       }
       if (is_finish == false)
       {
           loop_rate_calculate();
           sampl_count++;
           if (sampl_count > goal)
           {
               StopWork();
               std::cout<<"bag stop recording, acheived goal."<<std::endl;
               return;
           }
           bag.write<ros_myo_rv::EmgStamped>("/myo_black_emg",now_,msg1);
           bag.write<ros_myo_rv::EmgStamped>("/myo_white_emg",now_,msg2);
           if (bag_op[2]==1)
           {
               bag.write<sensor_msgs::Imu>("/myo_black_imu",now_,msg_imu_b);
           }
           if (bag_op[4]==1)
           {
               bag.write<sensor_msgs::Imu>("/myo_white_imu",now_,msg_imu_w);
           }
           if (bag_op[5])
           {
               bag.write<sensor_msgs::Imu>("/orange_imu",now_,msg_imu_o);
           }
           if (bag_op[6])
           {
               bag.write<geometry_msgs::TransformStamped>("/vicon/hand/hand",now_,msg_hand_poses);
               bag.write<geometry_msgs::TransformStamped>("/vicon/myo_w/myo_w",now_,msg_myo_w_poses);
               bag.write<geometry_msgs::TransformStamped>("/vicon/myo_b/myo_b",now_,msg_myo_b_poses);
           }
           if (bag_op[7])
           {
               bag.write<gui_data_collection::TfArrayStamped>("/joint_tfs",now_,msg_joint_tfs);
           }

           //qDebug()<<msg_imu_o.angular_velocity.z;
           std_msgs::Int32 msg_;
           msg_.data = sampl_count;
           num_smpl_pub.publish(msg_);
       }

       mutex.unlock();
    }
    catch (std::exception &e){
        std::cerr << "exception is :" << e.what()<<std::endl;
    }
}



void rosbag_worker::ft_myos_cb(const ros_myo_rv::EmgStampedConstPtr& msg1,const ros_myo_rv::EmgStampedConstPtr& msg2,const geometry_msgs::WrenchStampedConstPtr& msg3)
{
     Mutex mutex;
    try{
        mutex.lock();
        ros::Time now_ = ros::Time::now();
        if (stop_bag==true)
        {
            StopWork();
            std::cout<<"bag stop recording, terminated by user."<<std::endl;
            return;
        }
        if (is_finish == false)
        {
            loop_rate_calculate();
            sampl_count++;
            if (sampl_count > goal)
            {
                StopWork();
                std::cout<<"bag stop recording, acheived goal."<<std::endl;
                return;
            }
            bag.write<ros_myo_rv::EmgStamped>("/myo_black_emg",now_,msg1);
            bag.write<ros_myo_rv::EmgStamped>("/myo_white_emg",now_,msg2);
            geometry_msgs::WrenchStamped ft_msg;
            ft_msg_reset(msg3,ft_msg_ref,ft_msg);
            bag.write<geometry_msgs::WrenchStamped>("/ft_message",now_,ft_msg);
            if (bag_op[2]==1)
            {
                bag.write<sensor_msgs::Imu>("/myo_black_imu",now_,msg_imu_b);
            }
            if (bag_op[4]==1)
            {
                bag.write<sensor_msgs::Imu>("/myo_white_imu",now_,msg_imu_w);
            }
            if (bag_op[5])
            {
                bag.write<sensor_msgs::Imu>("/orange_imu",now_,msg_imu_o);
            }
            if (bag_op[6])
            {
                bag.write<geometry_msgs::TransformStamped>("/vicon/hand/hand",now_,msg_hand_poses);
                bag.write<geometry_msgs::TransformStamped>("/vicon/myo_w/myo_w",now_,msg_myo_w_poses);
                bag.write<geometry_msgs::TransformStamped>("/vicon/myo_b/myo_b",now_,msg_myo_b_poses);
            }
            if (bag_op[7])
            {
                bag.write<gui_data_collection::TfArrayStamped>("/joint_tfs",now_,msg_joint_tfs);
            }

            //qDebug()<<msg_imu_o.angular_velocity.z;
            std_msgs::Int32 msg_;
            msg_.data = sampl_count;
            num_smpl_pub.publish(msg_);
        }

        mutex.unlock();
    }
    catch (std::exception &e){
        std::cerr << "exception is :" << e.what()<<std::endl;
    }
    //loop_rate.get()->sleep();
}

void rosbag_worker::StopWork()
{
    white_imu_sub.shutdown();
    black_imu_sub.shutdown();
    orange_imu_sub.shutdown();
    joint_tfs_sub.shutdown();
    emg_black_sub.unsubscribe();
    emg_white_sub.unsubscribe();
    ft_sub.unsubscribe();
    sampl_count = 0;
    is_finish = true;
    bag.close();
}

void rosbag_worker::loop_rate_calculate()
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

void rosbag_worker::ft_msg_reset(const geometry_msgs::WrenchStampedConstPtr& msg, geometry_msgs::WrenchStamped ref_msg,geometry_msgs::WrenchStamped &ft_msg)
{
    ft_msg.wrench.force.x = msg.get()->wrench.force.x - ref_msg.wrench.force.x;
    ft_msg.wrench.force.y = msg.get()->wrench.force.y - ref_msg.wrench.force.y;
    ft_msg.wrench.force.z = msg.get()->wrench.force.z - ref_msg.wrench.force.z;
    ft_msg.wrench.torque.x = msg.get()->wrench.torque.x - ref_msg.wrench.torque.x;
    ft_msg.wrench.torque.y = msg.get()->wrench.torque.y - ref_msg.wrench.torque.y;
    ft_msg.wrench.torque.z = msg.get()->wrench.torque.z - ref_msg.wrench.torque.z;
}
