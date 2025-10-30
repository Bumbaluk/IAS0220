/********************************************************************************
** Form generated from reading UI file 'dialog_select_ros_topics.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_SELECT_ROS_TOPICS_H
#define UI_DIALOG_SELECT_ROS_TOPICS_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_dialogSelectRosTopics
{
public:
    QVBoxLayout *verticalLayout_3;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QCheckBox *checkBoxTimestamp;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QSpinBox *spinBoxArraySize;
    QToolButton *maximumSizeHelp;
    QSpacerItem *horizontalSpacer;
    QLabel *labelBigNumber;
    QRadioButton *radioMaxDiscard;
    QRadioButton *radioMaxClamp;
    QFrame *line;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_3;
    QLabel *label_4;
    QLineEdit *lineEditFilter;
    QTableWidget *listRosTopics;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_3;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_5;
    QCheckBox *checkBoxStringSuffix;
    QCheckBox *checkBoxStringBoolean;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *dialogSelectRosTopics)
    {
        if (dialogSelectRosTopics->objectName().isEmpty())
            dialogSelectRosTopics->setObjectName(QString::fromUtf8("dialogSelectRosTopics"));
        dialogSelectRosTopics->resize(905, 641);
        dialogSelectRosTopics->setMinimumSize(QSize(640, 480));
        verticalLayout_3 = new QVBoxLayout(dialogSelectRosTopics);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        tabWidget = new QTabWidget(dialogSelectRosTopics);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        checkBoxTimestamp = new QCheckBox(tab);
        checkBoxTimestamp->setObjectName(QString::fromUtf8("checkBoxTimestamp"));
        checkBoxTimestamp->setFocusPolicy(Qt::NoFocus);

        verticalLayout->addWidget(checkBoxTimestamp);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);

        spinBoxArraySize = new QSpinBox(tab);
        spinBoxArraySize->setObjectName(QString::fromUtf8("spinBoxArraySize"));
        spinBoxArraySize->setMaximumSize(QSize(100, 16777215));
        spinBoxArraySize->setMinimum(10);
        spinBoxArraySize->setMaximum(9999);
        spinBoxArraySize->setValue(100);

        horizontalLayout_2->addWidget(spinBoxArraySize);

        maximumSizeHelp = new QToolButton(tab);
        maximumSizeHelp->setObjectName(QString::fromUtf8("maximumSizeHelp"));
        maximumSizeHelp->setFocusPolicy(Qt::NoFocus);

        horizontalLayout_2->addWidget(maximumSizeHelp);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        labelBigNumber = new QLabel(tab);
        labelBigNumber->setObjectName(QString::fromUtf8("labelBigNumber"));
        labelBigNumber->setStyleSheet(QString::fromUtf8(""));

        horizontalLayout_2->addWidget(labelBigNumber);

        radioMaxDiscard = new QRadioButton(tab);
        radioMaxDiscard->setObjectName(QString::fromUtf8("radioMaxDiscard"));
        radioMaxDiscard->setFocusPolicy(Qt::NoFocus);
        radioMaxDiscard->setChecked(true);

        horizontalLayout_2->addWidget(radioMaxDiscard);

        radioMaxClamp = new QRadioButton(tab);
        radioMaxClamp->setObjectName(QString::fromUtf8("radioMaxClamp"));
        radioMaxClamp->setFocusPolicy(Qt::NoFocus);

        horizontalLayout_2->addWidget(radioMaxClamp);

        horizontalLayout_2->setStretch(1, 1);

        verticalLayout->addLayout(horizontalLayout_2);

        line = new QFrame(tab);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_2 = new QLabel(tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_2->setFont(font);

        horizontalLayout_5->addWidget(label_2);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_3);

        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_5->addWidget(label_4);

        lineEditFilter = new QLineEdit(tab);
        lineEditFilter->setObjectName(QString::fromUtf8("lineEditFilter"));
        lineEditFilter->setMaximumSize(QSize(300, 16777215));
        lineEditFilter->setClearButtonEnabled(true);

        horizontalLayout_5->addWidget(lineEditFilter);


        verticalLayout->addLayout(horizontalLayout_5);

        listRosTopics = new QTableWidget(tab);
        if (listRosTopics->columnCount() < 2)
            listRosTopics->setColumnCount(2);
        listRosTopics->setObjectName(QString::fromUtf8("listRosTopics"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(listRosTopics->sizePolicy().hasHeightForWidth());
        listRosTopics->setSizePolicy(sizePolicy);
        listRosTopics->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        listRosTopics->setEditTriggers(QAbstractItemView::NoEditTriggers);
        listRosTopics->setProperty("showDropIndicator", QVariant(false));
        listRosTopics->setDragDropOverwriteMode(false);
        listRosTopics->setSelectionMode(QAbstractItemView::MultiSelection);
        listRosTopics->setSelectionBehavior(QAbstractItemView::SelectRows);
        listRosTopics->setTextElideMode(Qt::ElideRight);
        listRosTopics->setSortingEnabled(true);
        listRosTopics->setColumnCount(2);
        listRosTopics->horizontalHeader()->setDefaultSectionSize(300);
        listRosTopics->horizontalHeader()->setProperty("showSortIndicator", QVariant(true));
        listRosTopics->horizontalHeader()->setStretchLastSection(true);
        listRosTopics->verticalHeader()->setMinimumSectionSize(18);
        listRosTopics->verticalHeader()->setDefaultSectionSize(21);

        verticalLayout->addWidget(listRosTopics);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);

        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        QFont font1;
        font1.setPointSize(10);
        label_3->setFont(font1);

        horizontalLayout_4->addWidget(label_3);


        verticalLayout->addLayout(horizontalLayout_4);


        verticalLayout_2->addLayout(verticalLayout);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_4 = new QVBoxLayout(tab_2);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font);

        verticalLayout_4->addWidget(label_5);

        checkBoxStringSuffix = new QCheckBox(tab_2);
        checkBoxStringSuffix->setObjectName(QString::fromUtf8("checkBoxStringSuffix"));

        verticalLayout_4->addWidget(checkBoxStringSuffix);

        checkBoxStringBoolean = new QCheckBox(tab_2);
        checkBoxStringBoolean->setObjectName(QString::fromUtf8("checkBoxStringBoolean"));

        verticalLayout_4->addWidget(checkBoxStringBoolean);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);

        tabWidget->addTab(tab_2, QString());

        verticalLayout_3->addWidget(tabWidget);

        buttonBox = new QDialogButtonBox(dialogSelectRosTopics);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setEnabled(false);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout_3->addWidget(buttonBox);


        retranslateUi(dialogSelectRosTopics);
        QObject::connect(buttonBox, SIGNAL(accepted()), dialogSelectRosTopics, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), dialogSelectRosTopics, SLOT(reject()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(dialogSelectRosTopics);
    } // setupUi

    void retranslateUi(QDialog *dialogSelectRosTopics)
    {
        dialogSelectRosTopics->setWindowTitle(QCoreApplication::translate("dialogSelectRosTopics", "Select ROS messages", nullptr));
        checkBoxTimestamp->setText(QCoreApplication::translate("dialogSelectRosTopics", "If present, use the timestamp in the field [header.stamp]", nullptr));
        label->setText(QCoreApplication::translate("dialogSelectRosTopics", "Maximum size of arrays:", nullptr));
#if QT_CONFIG(tooltip)
        spinBoxArraySize->setToolTip(QCoreApplication::translate("dialogSelectRosTopics", "MAX size of an array", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        spinBoxArraySize->setWhatsThis(QCoreApplication::translate("dialogSelectRosTopics", "<html><head/><body><p>It any field of a message contains a vector which size is larger than this number, the entire vector will we skipped.</p><p><br/></p></body></html>", nullptr));
#endif // QT_CONFIG(whatsthis)
        maximumSizeHelp->setText(QCoreApplication::translate("dialogSelectRosTopics", "?", nullptr));
        labelBigNumber->setText(QCoreApplication::translate("dialogSelectRosTopics", "If larger:", nullptr));
#if QT_CONFIG(tooltip)
        radioMaxDiscard->setToolTip(QCoreApplication::translate("dialogSelectRosTopics", "If the size of an array exceeds MAX, discard the entire array.", nullptr));
#endif // QT_CONFIG(tooltip)
        radioMaxDiscard->setText(QCoreApplication::translate("dialogSelectRosTopics", "discard", nullptr));
#if QT_CONFIG(tooltip)
        radioMaxClamp->setToolTip(QCoreApplication::translate("dialogSelectRosTopics", "If the size of an array exceeds MAX, keep only the kist MAX elements.", nullptr));
#endif // QT_CONFIG(tooltip)
        radioMaxClamp->setText(QCoreApplication::translate("dialogSelectRosTopics", "clamp", nullptr));
        label_2->setText(QCoreApplication::translate("dialogSelectRosTopics", "Select one or multiple topics:", nullptr));
        label_4->setText(QCoreApplication::translate("dialogSelectRosTopics", "Filter: ", nullptr));
        label_3->setText(QCoreApplication::translate("dialogSelectRosTopics", "Tip: [CTRL+A] select all, [CTRL+SHIFT+A] deselect all", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("dialogSelectRosTopics", "Topic Selector", nullptr));
        label_5->setText(QCoreApplication::translate("dialogSelectRosTopics", "<html><head/><body><p>String parsing <span style=\" font-weight:400;\">(used for instance in </span>diagnostic_msgs/DiagnosticArray<span style=\" font-weight:400;\">)</span></p></body></html>", nullptr));
        checkBoxStringSuffix->setText(QCoreApplication::translate("dialogSelectRosTopics", "remove suffixes from numbers (example: \"100ms\" -> 100)", nullptr));
        checkBoxStringBoolean->setText(QCoreApplication::translate("dialogSelectRosTopics", "convert boolean strings (\"false\"/\"true\") to numbers (0/1)", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("dialogSelectRosTopics", "Advanced Options", nullptr));
    } // retranslateUi

};

namespace Ui {
    class dialogSelectRosTopics: public Ui_dialogSelectRosTopics {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_SELECT_ROS_TOPICS_H
