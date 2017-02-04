#include <ros/ros.h>
#include <include/gui_data_collection/rosbag_worker.h>
#include <gui_data_collection/Bag_info.h>
using namespace gui_data_collection;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bag_woker");
    ros::NodeHandle n;
    rosbag_worker Bag_worker;
    ros::Rate loop_rate(2000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin();
    //Bag_worker.DoSetup();

    return 0;

}
