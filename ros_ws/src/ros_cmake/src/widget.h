#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QCloseEvent>
#include <QTimer>
#include <ros/ros.h>
#include "ui_widget.h"
#include "std_msgs/String.h"
#include <thread>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:

    int stop_counter_ = 1;

    explicit Widget(int argc, char **argv, QWidget *parent=0);
    ~Widget();

    void init_ros(int argc, char **argv);

private slots:
    void slot_btn_spawn_startboxsign();
    void slot_btn_delete_startboxsign();
    void slot_btn_quit();
    void barcodeCb(const std_msgs::String::ConstPtr& msg);


private:
    //ros::Publisher test_pub_;

    std::unique_ptr<std::thread> thread;
    ros::Subscriber barcode_sub_;
    Ui::Widget *ui;

protected:
    virtual void closeEvent(QCloseEvent *ev);
};

#endif // WIDGET_H
