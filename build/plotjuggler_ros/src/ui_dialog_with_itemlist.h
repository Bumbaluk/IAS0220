/********************************************************************************
** Form generated from reading UI file 'dialog_with_itemlist.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_WITH_ITEMLIST_H
#define UI_DIALOG_WITH_ITEMLIST_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DialogWithItemList
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QListWidget *listWidget;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *DialogWithItemList)
    {
        if (DialogWithItemList->objectName().isEmpty())
            DialogWithItemList->setObjectName(QString::fromUtf8("DialogWithItemList"));
        DialogWithItemList->resize(320, 253);
        verticalLayout = new QVBoxLayout(DialogWithItemList);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(DialogWithItemList);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        listWidget = new QListWidget(DialogWithItemList);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));

        verticalLayout->addWidget(listWidget);

        buttonBox = new QDialogButtonBox(DialogWithItemList);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(DialogWithItemList);

        QMetaObject::connectSlotsByName(DialogWithItemList);
    } // setupUi

    void retranslateUi(QDialog *DialogWithItemList)
    {
        DialogWithItemList->setWindowTitle(QCoreApplication::translate("DialogWithItemList", "Dialog", nullptr));
        label->setText(QCoreApplication::translate("DialogWithItemList", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DialogWithItemList: public Ui_DialogWithItemList {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_WITH_ITEMLIST_H
