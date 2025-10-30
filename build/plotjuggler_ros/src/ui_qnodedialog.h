/********************************************************************************
** Form generated from reading UI file 'qnodedialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QNODEDIALOG_H
#define UI_QNODEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_QNodeDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLineEdit *lineEditMaster;
    QLabel *label_2;
    QLineEdit *lineEditHost;
    QCheckBox *checkBoxUseEnvironment;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButtonCancel;
    QPushButton *pushButtonConnect;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QDialog *QNodeDialog)
    {
        if (QNodeDialog->objectName().isEmpty())
            QNodeDialog->setObjectName(QString::fromUtf8("QNodeDialog"));
        QNodeDialog->resize(295, 182);
        verticalLayout = new QVBoxLayout(QNodeDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(QNodeDialog);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        lineEditMaster = new QLineEdit(QNodeDialog);
        lineEditMaster->setObjectName(QString::fromUtf8("lineEditMaster"));
        lineEditMaster->setEnabled(false);

        verticalLayout->addWidget(lineEditMaster);

        label_2 = new QLabel(QNodeDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        lineEditHost = new QLineEdit(QNodeDialog);
        lineEditHost->setObjectName(QString::fromUtf8("lineEditHost"));
        lineEditHost->setEnabled(false);

        verticalLayout->addWidget(lineEditHost);

        checkBoxUseEnvironment = new QCheckBox(QNodeDialog);
        checkBoxUseEnvironment->setObjectName(QString::fromUtf8("checkBoxUseEnvironment"));
        checkBoxUseEnvironment->setFocusPolicy(Qt::NoFocus);
        checkBoxUseEnvironment->setChecked(true);

        verticalLayout->addWidget(checkBoxUseEnvironment);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButtonCancel = new QPushButton(QNodeDialog);
        pushButtonCancel->setObjectName(QString::fromUtf8("pushButtonCancel"));
        pushButtonCancel->setEnabled(true);
        pushButtonCancel->setFocusPolicy(Qt::NoFocus);

        horizontalLayout->addWidget(pushButtonCancel);

        pushButtonConnect = new QPushButton(QNodeDialog);
        pushButtonConnect->setObjectName(QString::fromUtf8("pushButtonConnect"));
        pushButtonConnect->setFocusPolicy(Qt::NoFocus);

        horizontalLayout->addWidget(pushButtonConnect);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(QNodeDialog);

        QMetaObject::connectSlotsByName(QNodeDialog);
    } // setupUi

    void retranslateUi(QDialog *QNodeDialog)
    {
        QNodeDialog->setWindowTitle(QCoreApplication::translate("QNodeDialog", "Find ROS master", nullptr));
        label->setText(QCoreApplication::translate("QNodeDialog", "ROS Master URL", nullptr));
        label_2->setText(QCoreApplication::translate("QNodeDialog", "Host address", nullptr));
        checkBoxUseEnvironment->setText(QCoreApplication::translate("QNodeDialog", "Use environment variables", nullptr));
        pushButtonCancel->setText(QCoreApplication::translate("QNodeDialog", "Cancel", nullptr));
        pushButtonConnect->setText(QCoreApplication::translate("QNodeDialog", "Connect", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QNodeDialog: public Ui_QNodeDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QNODEDIALOG_H
