#include "widget.h"
#include <QString>
#include <QDebug>
#include <QMessageBox>
#include <std_msgs/Int8.h>
#include <string>


Widget::Widget(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    QObject::connect(ui->pushButton_spawn_startboxsign, SIGNAL(clicked()), this, SLOT(slot_btn_spawn_startboxsign()));
    QObject::connect(ui->pushButton_delete_startboxsign, SIGNAL(clicked()), this, SLOT(slot_btn_delete_startboxsign()));
    QObject::connect(ui->pushButton_quit, SIGNAL(clicked()), this, SLOT(slot_btn_quit()));

    init_ros(argc, argv);




}

Widget::~Widget()
{
    delete ui;
}


void Widget::barcodeCb(const std_msgs::String::ConstPtr& msg)
{
    std::string message(msg->data.c_str());
    std::string txt = message + " " + std::to_string(stop_counter_);

    std::cout << txt << std::endl;
    ui->lineEdit->setText(txt.c_str() );
    stop_counter_++;

    if(stop_counter_>99) stop_counter_ = 1;
}




void Widget::init_ros(int argc, char **argv)
{
    ros::init(argc, argv, "ros_cmake");
    ros::NodeHandle private_nh("~");

    barcode_sub_ = private_nh.subscribe("/barcode", 1, &Widget::barcodeCb, this);
    std::cout << "subscribe" << std::endl;

    thread.reset(new std::thread([](){ static ros::MultiThreadedSpinner spinner; spinner.spin(); }));


}

void Widget::slot_btn_spawn_startboxsign()
{

    std::string str = "rosrun gazebo_ros spawn_model -sdf -file /home/tb/.gazebo/models/startboxsign/model.sdf -model startboxsign_ -x 0.0 -y 0.44 -z 0.1 -R -3.14159 -P -1.570795 -Y -1.570795;";
    const char *command = str.c_str();
    system(command);

}






void Widget::slot_btn_delete_startboxsign()
{

    std::string str = "rosservice call gazebo/delete_model '{model_name: startboxsign_}';";
    const char *command = str.c_str();
    system(command);

}

void Widget::slot_btn_quit()
{

    this->close();
}

void Widget::closeEvent(QCloseEvent *ev)
{
    int result = QMessageBox::question(this, QString("Tips"), QString("Dialog is closing..."));
    if(QMessageBox::Yes == result)
        ev->accept();
    else
        ev->ignore();
}
