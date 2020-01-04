/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_spawn_startboxsign;
    QPushButton *pushButton_delete_startboxsign;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_quit;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLineEdit *lineEdit;
    QSpacerItem *verticalSpacer_3;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QStringLiteral("Widget"));
        Widget->resize(372, 294);
        verticalLayout_2 = new QVBoxLayout(Widget);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        pushButton_spawn_startboxsign = new QPushButton(Widget);
        pushButton_spawn_startboxsign->setObjectName(QStringLiteral("pushButton_spawn_startboxsign"));

        horizontalLayout->addWidget(pushButton_spawn_startboxsign);
        pushButton_delete_startboxsign = new QPushButton(Widget);
        pushButton_delete_startboxsign->setObjectName(QStringLiteral("pushButton_delete_startboxsign"));

        horizontalLayout->addWidget(pushButton_delete_startboxsign);


        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton_quit = new QPushButton(Widget);
        pushButton_quit->setObjectName(QStringLiteral("pushButton_quit"));

        horizontalLayout->addWidget(pushButton_quit);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label = new QLabel(Widget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_2->addWidget(label);

        //QPalette *palette = new QPalette();
        //palette->setColor(QPalette::Base,Qt::gray);
        //palette->setColor(QPalette::Text,Qt::darkGray);
        //lineEdit->setPalette(*palette);

        lineEdit = new QLineEdit(Widget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setEnabled(false);

        lineEdit->setStyleSheet("QLineEdit { background: rgb(0, 0, 0); selection-background-color: rgb(233, 99, 0); }");

        horizontalLayout_2->addWidget(lineEdit);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "Gazebo GUI", Q_NULLPTR));
        pushButton_spawn_startboxsign->setText(QApplication::translate("Widget", "Spawn Startboxsign", Q_NULLPTR));
        pushButton_delete_startboxsign->setText(QApplication::translate("Widget", "Delete Startboxsign", Q_NULLPTR));
        pushButton_quit->setText(QApplication::translate("Widget", "Quit", Q_NULLPTR));
        label->setText(QApplication::translate("Widget", "Barcode Topic data:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
