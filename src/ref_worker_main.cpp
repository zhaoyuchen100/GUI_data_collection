#include <ros/ros.h>
#include <QDebug>
#include <include/gui_data_collection/refsiggenerator.h>
#include <gui_data_collection/ref_info.h>
#include <std_msgs/Float64.h>
using namespace gui_data_collection;
double peak_amp;
bool stop_ref_signal = true;
QString ref_signal_type;
RefSigGenerator ref_worker;
bool param_setup_service(ref_info::Request &req, ref_info::Response &res)// be careful about the Capital spelling of Response. response will not work!
{
    peak_amp = req.peak_amp;
    stop_ref_signal = req.stop;
    ref_signal_type = QString(req.ref_signal_type.c_str());
    ref_worker.peak_amp = peak_amp;
    ref_worker.ref_signal_type = ref_signal_type;

    res.ack = true;
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ref_woker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("ft_ref_follow",1);
    ros::ServiceServer service = n.advertiseService("ref_config_service",param_setup_service);

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        std_msgs::Float64 msg;
        if (ref_signal_type == QString("slope_step") && stop_ref_signal==false)
        {
            msg.data = ref_worker.slopstep_prim_gen();
            pub.publish(msg);

        }
        else if (ref_signal_type == QString ("sine") && stop_ref_signal ==false)
        {
            //qDebug()<<"i am here";
            msg.data = ref_worker.sin_wave_prim_gen();
            pub.publish(msg);
        }
        else
        {
            ref_worker.reset();
            stop_ref_signal = true;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    //Bag_worker.DoSetup();

    return 0;

}
