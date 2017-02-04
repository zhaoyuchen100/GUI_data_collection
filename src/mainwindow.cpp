#include <include/gui_data_collection/mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),num_data_record(0)
{
    ui->setupUi(this);
    // initialize ref_sig_gen
    ui->Generator_type->setText(QString("slope_step"));
    ui->Num_sampls->setText(QString("1000"));
    ui->Peak_amp->setText(QString("10"));
    //initialize rosbag_worker_obj

    //initialize ft_msg_ref
    ft_msg_ref.wrench.force.x = 0;
    ft_msg_ref.wrench.force.y = 0;
    ft_msg_ref.wrench.force.z = 0;
    ft_msg_ref.wrench.torque.x = 0;
    ft_msg_ref.wrench.torque.y = 0;
    ft_msg_ref.wrench.torque.z = 0;
    // initialize count down timer
    countdown_timer = new QTimer;
    // do plot staff
    ros_timer = new QTimer;
    sample_count = 0;
    customplot_setup();
    qRegisterMetaType<geometry_msgs::WrenchStamped>("geometry_msgs::WrenchStamped");
    // set check box for bag files
    ui->checkBox_BLCAK->setCheckState(Qt::Unchecked);
    ui->checkBox_WHITE->setCheckState(Qt::Unchecked);
    ui->checkBox_FT->setCheckState(Qt::Unchecked);
    ui->checkBox_ALL->setCheckState(Qt::Unchecked);
    ui->checkBox_BLACK_IMU->setCheckState(Qt::Unchecked);
    ui->checkBox_WHITE_IMU->setCheckState(Qt::Unchecked);
    ui->checkBox_VICON->setCheckState(Qt::Unchecked);
    ui->checkBox_JOINT_TFS->setCheckState(Qt::Unchecked);
    //initilize trial counter
    ui->sampleCount->setPalette(Qt::black);
    ui->countdown->setPalette(Qt::blue);
    ui->trialCounter->setPalette(Qt::black);
    ui->trialCounter->display(0);
    trial_counter = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
    delete countdown_timer;
    delete ros_timer;
}

void MainWindow::init()
{
    connect(ros_timer, SIGNAL(timeout()),this,SLOT(on_ros_spin()),Qt::UniqueConnection);
    connect(countdown_timer,SIGNAL(timeout()),this,SLOT(set_countdown_display()),Qt::UniqueConnection);
    // initialize count down;
    countdown_value = 3;
    ui->countdown->display(countdown_value);
    countdown_timer->start(1000);
    // initialize ref signal generator connect ref signal to slots
    ui->progressBar->setValue(0);
    ui->sampleCount->display(0);
    ui->trialCounter->display(0);
    //initialize num of data record
    num_data_record = 0;
    //initialize primitive mode
    Primitive_mode = ui->comboBox->currentText();
    trial_counter = 0;
}

void MainWindow::loop_rate_calculate()
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
        if (now.toSec()-begin.toSec()>5)
        {
            qDebug()<< "sampling rate= :" << sampl_counter/(now.toSec()-begin.toSec());
            begin = now;
            sampl_counter = 0;
        }
    }
}

void MainWindow::customplot_setup()
{
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue));
    ui->customPlot->graph(0)->setAntialiasedFill(false);
    ui->customPlot->xAxis->setLabel("Samples");
    ui->customPlot->yAxis->setLabel("Reference Force Signal in N");
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot->axisRect()->setupFullAxesBox();
    connect(ui->customPlot->xAxis,SIGNAL(rangeChanged(QCPRange)),ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
    connect(this,SIGNAL(start_update_plot()),this,SLOT(on_customPlot_start_update_plot()),Qt::UniqueConnection);

}

void MainWindow::plotFTdataCB(geometry_msgs::WrenchStampedConstPtr msg)
{
    //loop_rate_calculate();
    ft_msg_plot.header = msg.get()->header;
    ft_msg_plot.wrench = msg.get()->wrench;
    //qDebug()<<"here";
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
    static double lastPointKey;
    double value;
    if (Primitive_mode.compare(QString("Press down"))==0)
    {
        value = (ft_msg_plot.wrench.force.z-ft_msg_ref.wrench.force.z)/1000000;
    }
    else if (Primitive_mode.compare(QString("Left"))==0 || Primitive_mode.compare(QString("Right"))==0)
    {
        value = (ft_msg_plot.wrench.force.x-ft_msg_ref.wrench.force.x)/1000000;
    }
    else if (Primitive_mode.compare(QString("Push forward"))==0 || Primitive_mode.compare(QString("Pull back"))==0)
    {
        value = (ft_msg_plot.wrench.force.y-ft_msg_ref.wrench.force.y)/1000000;
    }
    if (key-lastPointKey > 0.02) // at most add point every 10 ms
    {
      // add data to lines:
      ui->customPlot->graph(0)->addData(sample_count, value);
      // remove data of lines that's outside visible range:
      // rescale value (vertical) axis to fit the current data:
      ui->customPlot->graph(0)->removeDataBefore(sample_count-500);
      ui->customPlot->graph(0)->rescaleValueAxis();
      lastPointKey = key;
      sample_count++;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    ui->customPlot->xAxis->setRange(sample_count+0.25, 200, Qt::AlignRight);
    ui->customPlot->replot();

    // calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
      ui->statusBar->showMessage(
            QString("%1 FPS, Total Data points: %2")
            .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
            .arg(ui->customPlot->graph(0)->data()->count())
            , 0);
      lastFpsKey = key;
      frameCount = 0;
    }
}

void MainWindow::getNumSamplCB(std_msgs::Int32ConstPtr msg)
{
    ui->progressBar->setValue(msg.get()->data);
}

void MainWindow::plot_ref_signal(std_msgs::Float64ConstPtr msg)
{
    double value = msg.get()->data;
    ui->customPlot->graph(1)->addData(sample_count, value);
    ui->customPlot->graph(1)->removeDataBefore(sample_count-500);
    //ui->customPlot->graph(1)->rescaleValueAxis(true);
}


void MainWindow::on_pushButton_Start_clicked()
{
   init();
}

void MainWindow::on_pushButton_Stop_clicked()
{
    ros_timer->stop();
    countdown_timer->stop();
    disconnect(countdown_timer,SIGNAL(timeout()),this,SLOT(set_countdown_display()));
    disconnect(ros_timer, SIGNAL(timeout()),this,SLOT(on_ros_spin()));
    disconnect(countdown_timer,SIGNAL(timeout()),this,SLOT(set_countdown_display()));
    bag_srv_param.request.stop = true;
    if (bag_client.call(bag_srv_param))
    {
        std::cout<<"bag service stopped!"<<std::endl;
    }
    else
    {
        std::cout<<"Failed to talk to bag service"<<std::endl;

    }

    ref_srv_param.request.stop = true;
    if (ref_client.call(ref_srv_param))
    {
        std::cout<<"reference service stopped !"<<std::endl;
    }
    else
    {
        std::cout<<"Faild to talk to reference service"<<std::endl;
    }


}

void MainWindow::on_progressBar_valueChanged(int value)
{
    ui->sampleCount->display(value);
}

void MainWindow::on_customPlot_start_update_plot()
{
    ft_sub = n_.subscribe<geometry_msgs::WrenchStampedConstPtr>("ft_message",1,&MainWindow::plotFTdataCB,this);
    num_sampl_sub = n_.subscribe<std_msgs::Int32ConstPtr>("sample_recorded",1,&MainWindow::getNumSamplCB,this);
    ros_timer->start(10);
}


void MainWindow::set_countdown_display()
{
    countdown_value = countdown_value - 1;
    ui->countdown->display(countdown_value);
    if(countdown_value == 0)
    {
        countdown_timer->stop();
        emit start_update_plot();
        disconnect(countdown_timer,SIGNAL(timeout()),this,SLOT(set_countdown_display()));
    }
}

void MainWindow::on_ros_spin()
{
    ros::spinOnce();
    //qDebug()<<" i spin once !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
}

void MainWindow::on_pushButton_remove_generator_clicked()
{
    ui->customPlot->removeGraph(1);
    ref_sub.shutdown();
    /// stop the ref here!
    ref_srv_param.request.stop = true;
    if (ref_client.call(ref_srv_param))
    {
        std::cout<<"Talking to reference service!"<<std::endl;
    }
    else
    {
        std::cout<<"Failed to talk to reference service"<<std::endl;

    }

}


void MainWindow::on_checkBox_ALL_clicked()
{
    if (ui->checkBox_ALL->isChecked())
    {
        ui->checkBox_BLCAK->setCheckState(Qt::Checked);
        ui->checkBox_BLACK_IMU->setCheckState(Qt::Checked);
        ui->checkBox_WHITE->setCheckState(Qt::Checked);
        ui->checkBox_WHITE_IMU->setCheckState(Qt::Checked);
        ui->checkBox_FT->setCheckState(Qt::Checked);
        ui->checkBox_VICON->setCheckState(Qt::Checked);
        ui->checkBox_JOINT_TFS->setCheckState(Qt::Checked);
        ui->checkBox_orange_imu->setCheckState(Qt::Checked);
    }
    else
    {
        ui->checkBox_BLCAK->setCheckState(Qt::Unchecked);
        ui->checkBox_BLACK_IMU->setCheckState(Qt::Unchecked);
        ui->checkBox_WHITE->setCheckState(Qt::Unchecked);
        ui->checkBox_WHITE_IMU->setCheckState(Qt::Unchecked);
        ui->checkBox_FT->setCheckState(Qt::Unchecked);
        ui->checkBox_VICON->setCheckState(Qt::Unchecked);
        ui->checkBox_JOINT_TFS->setCheckState(Qt::Unchecked);
        ui->checkBox_orange_imu->setCheckState(Qt::Unchecked);
    }
}

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
    Primitive_mode = arg1;
}

void MainWindow::on_bag_record_clicked()
{
    ui->progressBar->setValue(0);
    ui->sampleCount->display(0);
    trial_counter++;
    ui->trialCounter->display(trial_counter);
    bag_srv_param.request.bag_option.resize(8);
    std::vector<int64_t> vec_8_0 (8,0);
    std::vector<int64_t> vec_8_1 (8,1);
    bag_srv_param.request.bag_option = vec_8_0;
    // initialize file name and setup bag worker;
    // parsing the bag file options
    if (ui->checkBox_ALL->isChecked())
    {
        bag_srv_param.request.bag_option = vec_8_1;
    }
    if (ui->checkBox_FT->isChecked() )
    {
        bag_srv_param.request.bag_option[0] = 1;
    }
    if (ui->checkBox_BLCAK->isChecked())
    {
        bag_srv_param.request.bag_option[1] = 1;
    }
    if (ui->checkBox_BLACK_IMU->isChecked())
    {
        bag_srv_param.request.bag_option[2] = 1;
    }
    if (ui->checkBox_WHITE->isChecked())
    {
        bag_srv_param.request.bag_option[3] = 1;
    }
    if (ui->checkBox_WHITE_IMU->isChecked())
    {
        bag_srv_param.request.bag_option[4] = 1;
    }
    if (ui->checkBox_orange_imu->isChecked())
    {
        bag_srv_param.request.bag_option[5] = 1;
    }
    if (ui->checkBox_VICON->isChecked())
    {
        bag_srv_param.request.bag_option[6] = 1;
    }
    if (ui->checkBox_JOINT_TFS->isChecked())
    {
        bag_srv_param.request.bag_option[7] = 1;
    }

    bag_srv_param.request.bag_name = ui->BAG_file_name->text().toStdString().c_str();
    bag_srv_param.request.ft_msg_ref = ft_msg_ref;
    bag_srv_param.request.goal = ui->Num_sampls->text().toInt();
    bag_srv_param.request.stop = false;
    bag_client = n_.serviceClient<Bag_info>("bag_config_service");
    //initialize progress bar
    num_sampls = ui->Num_sampls->text().toDouble();
    ui->progressBar->setMaximum(num_sampls);

    if (bag_client.call(bag_srv_param))
    {
        std::cout<<"bag service started !"<<std::endl;
    }
    else
    {
        std::cout<<"Faild to talk to bag service"<<std::endl;
    }
}


void MainWindow::on_pushButton_bias_clicked()
{
    ft_msg_ref = ft_msg_plot;
}

void MainWindow::on_pushButton_add_generator_clicked()
{
     ///start adding ref signal here !
    ref_sub = n_.subscribe<std_msgs::Float64ConstPtr>("ft_ref_follow",1,&MainWindow::plot_ref_signal,this);
    ref_srv_param.request.ref_signal_type = ui->Generator_type->text().toStdString().c_str();
    ref_srv_param.request.peak_amp = ui->Peak_amp->text().toDouble();
    ref_srv_param.request.stop = false;
    // add ref signal to front panel
    QPen redPen;
    redPen.setColor(QColor(255, 40, 30, 150));
    redPen.setStyle(Qt::SolidLine);
    redPen.setWidthF(4);
    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setPen(redPen);
    ui->customPlot->graph(1)->setAntialiasedFill(false);
    ref_client = n_.serviceClient<ref_info>("ref_config_service");

    if (ref_client.call(ref_srv_param))
    {
        std::cout<<"Talking to reference service!"<<std::endl;
    }
    else
    {
        std::cout<<"Faild to talk to reference service"<<std::endl;
    }

}
