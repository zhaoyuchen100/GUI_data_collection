#include <include/gui_data_collection/refsiggenerator.h>

RefSigGenerator::RefSigGenerator(QObject *parent) : QObject(parent)
{
    peak_amp = 0;
    start_smp = 0;
    ref_signal_type = QString("");

}

RefSigGenerator::~RefSigGenerator()
{

}

double RefSigGenerator::slopstep_prim_gen()
{
    double slope;double interval;
    interval = 200;
    slope = peak_amp/(interval);
    if (start_smp <= interval)
    {
        ref_signal = slope*start_smp;
        start_smp ++;
    }
    else if (start_smp <= (interval +300))
    {
        start_smp ++;
    }
    else if (start_smp <= (interval +300+interval))
    {
        ref_signal = -slope*(start_smp-interval-300)+peak_amp;
        start_smp ++;
    }
    else
    {
        start_smp = 1;
    }
    //qDebug()<<peak_amp;
    return ref_signal;

}

double RefSigGenerator::sin_wave_prim_gen()
{
  double amp;
  amp = peak_amp;
  double f = double(1)/double(600);
  ref_signal = fabs(amp*sin(2*M_PI*f*start_smp));
  start_smp ++;
  //qDebug()<<ref_signal;
  return ref_signal;

}

void RefSigGenerator::gen_run()
{
    if (ref_signal_type == QString("slope_step"))
    {
        slopstep_prim_gen();
        //qDebug()<< ref_signal;
    }
    if (ref_signal_type == QString ("sine"))
    {
        sin_wave_prim_gen();
    }
}

void RefSigGenerator::reset()
{
       start_smp = 0;
       //peak_amp = peak_amp+rand()%10+1;
}

