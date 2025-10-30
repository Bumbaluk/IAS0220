/********************************************************************************
** Form generated from reading UI file 'publisher_select_dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PUBLISHER_SELECT_DIALOG_H
#define UI_PUBLISHER_SELECT_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PublisherSelect
{
public:
    QVBoxLayout *verticalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_3;
    QFormLayout *formLayout;
    QRadioButton *radioButtonClock;
    QRadioButton *radioButtonHeaderStamp;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButtonSelect;
    QPushButton *pushButtonDeselect;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PublisherSelect)
    {
        if (PublisherSelect->objectName().isEmpty())
            PublisherSelect->setObjectName(QString::fromUtf8("PublisherSelect"));
        PublisherSelect->resize(400, 300);
        PublisherSelect->setMinimumSize(QSize(400, 300));
        verticalLayout = new QVBoxLayout(PublisherSelect);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        scrollArea = new QScrollArea(PublisherSelect);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 380, 158));
        verticalLayout_3 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));

        verticalLayout_3->addLayout(formLayout);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        radioButtonClock = new QRadioButton(PublisherSelect);
        radioButtonClock->setObjectName(QString::fromUtf8("radioButtonClock"));
        radioButtonClock->setFocusPolicy(Qt::NoFocus);
        radioButtonClock->setChecked(true);

        verticalLayout->addWidget(radioButtonClock);

        radioButtonHeaderStamp = new QRadioButton(PublisherSelect);
        radioButtonHeaderStamp->setObjectName(QString::fromUtf8("radioButtonHeaderStamp"));
        radioButtonHeaderStamp->setFocusPolicy(Qt::NoFocus);

        verticalLayout->addWidget(radioButtonHeaderStamp);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        pushButtonSelect = new QPushButton(PublisherSelect);
        pushButtonSelect->setObjectName(QString::fromUtf8("pushButtonSelect"));
        pushButtonSelect->setFocusPolicy(Qt::NoFocus);

        horizontalLayout->addWidget(pushButtonSelect);

        pushButtonDeselect = new QPushButton(PublisherSelect);
        pushButtonDeselect->setObjectName(QString::fromUtf8("pushButtonDeselect"));
        pushButtonDeselect->setFocusPolicy(Qt::NoFocus);

        horizontalLayout->addWidget(pushButtonDeselect);


        verticalLayout->addLayout(horizontalLayout);

        buttonBox = new QDialogButtonBox(PublisherSelect);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(PublisherSelect);
        QObject::connect(buttonBox, SIGNAL(accepted()), PublisherSelect, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), PublisherSelect, SLOT(reject()));

        QMetaObject::connectSlotsByName(PublisherSelect);
    } // setupUi

    void retranslateUi(QDialog *PublisherSelect)
    {
        PublisherSelect->setWindowTitle(QCoreApplication::translate("PublisherSelect", "Select topics to be published", nullptr));
#if QT_CONFIG(tooltip)
        radioButtonClock->setToolTip(QCoreApplication::translate("PublisherSelect", "<html><head/><body><p><span style=\" color:#2e3436;\">Publish the topic [/clock].</span><span style=\" font-family:'monospace'; color:#2e3436;\"/></p><p><span style=\" font-family:'monospace'; color:#2e3436;\">You should set rosparam use_sim_time = true</span></p></body></html>", nullptr));
#endif // QT_CONFIG(tooltip)
        radioButtonClock->setText(QCoreApplication::translate("PublisherSelect", "Keep original timestamp and publish [/clock]", nullptr));
#if QT_CONFIG(tooltip)
        radioButtonHeaderStamp->setToolTip(QCoreApplication::translate("PublisherSelect", "<html><head/><body><p><span style=\" color:#2e3436;\">Pretend it is a new message.</span></p><p><span style=\" font-family:'monospace'; color:#2e3436;\">The timestamp of the original message will be overwritten with ros::Time::Now()</span></p></body></html>", nullptr));
#endif // QT_CONFIG(tooltip)
        radioButtonHeaderStamp->setText(QCoreApplication::translate("PublisherSelect", "Overwrite timestamp [std_msgs/Header/stamp]", nullptr));
        pushButtonSelect->setText(QCoreApplication::translate("PublisherSelect", "SelectAll", nullptr));
        pushButtonDeselect->setText(QCoreApplication::translate("PublisherSelect", "Deselect All", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PublisherSelect: public Ui_PublisherSelect {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PUBLISHER_SELECT_DIALOG_H
