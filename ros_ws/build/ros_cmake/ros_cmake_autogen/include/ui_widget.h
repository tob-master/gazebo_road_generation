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
    QPushButton *pushButton_quit;
    QHBoxLayout *horizontalLayout_2;
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

        verticalLayout->addLayout(horizontalLayout);

        pushButton_spawn_startboxsign = new QPushButton(Widget);
        pushButton_spawn_startboxsign->setObjectName(QStringLiteral("pushButton_spawn_startboxsign"));

        verticalLayout->addWidget(pushButton_spawn_startboxsign);

        pushButton_delete_startboxsign = new QPushButton(Widget);
        pushButton_delete_startboxsign->setObjectName(QStringLiteral("pushButton_delete_startboxsign"));

        verticalLayout->addWidget(pushButton_delete_startboxsign);

        pushButton_quit = new QPushButton(Widget);
        pushButton_quit->setObjectName(QStringLiteral("pushButton_quit"));

        verticalLayout->addWidget(pushButton_quit);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));

        verticalLayout->addLayout(horizontalLayout_2);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "QWidget", Q_NULLPTR));
        pushButton_spawn_startboxsign->setText(QApplication::translate("Widget", "Spawn Startboxsign", Q_NULLPTR));
        pushButton_delete_startboxsign->setText(QApplication::translate("Widget", "Delete Startboxsign", Q_NULLPTR));
        pushButton_quit->setText(QApplication::translate("Widget", "Quit", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
