/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QGroupBox *groupBox;
    QVTKWidget *qvtkWidget;
    QGroupBox *groupBox_2;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *tab_2;
    QLineEdit *lineEdit;
    QLineEdit *lineEdit_2;
    QRadioButton *radioButton;
    QPushButton *pushButton;
    QToolButton *toolButton;
    QToolButton *toolButton_2;
    QCheckBox *checkBox;
    QLabel *label;
    QLabel *label_2;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_3;
    QRadioButton *radioButton_4;
    QCheckBox *checkBox_2;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QWidget *tab_3;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QPushButton *pushButton_8;
    QPushButton *pushButton_9;
    QPushButton *pushButton_10;
    QLabel *label_3;
    QLabel *label_4;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_4;
    QToolButton *toolButton_3;
    QToolButton *toolButton_4;
    QWidget *tab_4;
    QRadioButton *radioButton_5;
    QRadioButton *radioButton_6;
    QPushButton *pushButton_11;
    QPushButton *pushButton_12;
    QPushButton *pushButton_13;
    QLabel *label_5;
    QLabel *label_6;
    QLineEdit *lineEdit_5;
    QLineEdit *lineEdit_6;
    QToolButton *toolButton_5;
    QToolButton *toolButton_6;
    QWidget *tab_5;
    QListView *listView;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(1900, 1200);
        groupBox = new QGroupBox(Dialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(610, 10, 1241, 1111));
        qvtkWidget = new QVTKWidget(groupBox);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, 20, 1241, 1091));
        groupBox_2 = new QGroupBox(Dialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 10, 581, 1111));
        tabWidget = new QTabWidget(groupBox_2);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 20, 581, 381));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        lineEdit = new QLineEdit(tab_2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(80, 50, 431, 25));
        lineEdit_2 = new QLineEdit(tab_2);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        lineEdit_2->setGeometry(QRect(80, 90, 431, 25));
        radioButton = new QRadioButton(tab_2);
        radioButton->setObjectName(QString::fromUtf8("radioButton"));
        radioButton->setGeometry(QRect(20, 130, 112, 23));
        pushButton = new QPushButton(tab_2);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(20, 310, 531, 25));
        toolButton = new QToolButton(tab_2);
        toolButton->setObjectName(QString::fromUtf8("toolButton"));
        toolButton->setGeometry(QRect(530, 50, 26, 24));
        toolButton_2 = new QToolButton(tab_2);
        toolButton_2->setObjectName(QString::fromUtf8("toolButton_2"));
        toolButton_2->setGeometry(QRect(530, 90, 26, 24));
        checkBox = new QCheckBox(tab_2);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(440, 130, 92, 23));
        label = new QLabel(tab_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 46, 67, 31));
        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 80, 67, 31));
        radioButton_2 = new QRadioButton(tab_2);
        radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));
        radioButton_2->setGeometry(QRect(20, 160, 112, 23));
        radioButton_3 = new QRadioButton(tab_2);
        radioButton_3->setObjectName(QString::fromUtf8("radioButton_3"));
        radioButton_3->setGeometry(QRect(20, 190, 112, 23));
        radioButton_4 = new QRadioButton(tab_2);
        radioButton_4->setObjectName(QString::fromUtf8("radioButton_4"));
        radioButton_4->setGeometry(QRect(20, 220, 112, 23));
        checkBox_2 = new QCheckBox(tab_2);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(440, 220, 92, 23));
        pushButton_2 = new QPushButton(tab_2);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(320, 270, 89, 25));
        pushButton_3 = new QPushButton(tab_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(150, 270, 131, 25));
        pushButton_4 = new QPushButton(tab_2);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(20, 270, 89, 25));
        pushButton_5 = new QPushButton(tab_2);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(430, 270, 121, 25));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        pushButton_6 = new QPushButton(tab_3);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setGeometry(QRect(20, 230, 151, 25));
        pushButton_7 = new QPushButton(tab_3);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        pushButton_7->setGeometry(QRect(410, 230, 151, 25));
        pushButton_8 = new QPushButton(tab_3);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        pushButton_8->setGeometry(QRect(20, 270, 151, 25));
        pushButton_9 = new QPushButton(tab_3);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));
        pushButton_9->setGeometry(QRect(410, 270, 151, 25));
        pushButton_10 = new QPushButton(tab_3);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));
        pushButton_10->setGeometry(QRect(20, 310, 541, 25));
        label_3 = new QLabel(tab_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 30, 67, 17));
        label_4 = new QLabel(tab_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 90, 67, 17));
        lineEdit_3 = new QLineEdit(tab_3);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        lineEdit_3->setGeometry(QRect(80, 30, 421, 25));
        lineEdit_4 = new QLineEdit(tab_3);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));
        lineEdit_4->setGeometry(QRect(80, 80, 421, 25));
        toolButton_3 = new QToolButton(tab_3);
        toolButton_3->setObjectName(QString::fromUtf8("toolButton_3"));
        toolButton_3->setGeometry(QRect(530, 30, 26, 24));
        toolButton_4 = new QToolButton(tab_3);
        toolButton_4->setObjectName(QString::fromUtf8("toolButton_4"));
        toolButton_4->setGeometry(QRect(530, 80, 26, 24));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        radioButton_5 = new QRadioButton(tab_4);
        radioButton_5->setObjectName(QString::fromUtf8("radioButton_5"));
        radioButton_5->setGeometry(QRect(250, 280, 231, 23));
        radioButton_6 = new QRadioButton(tab_4);
        radioButton_6->setObjectName(QString::fromUtf8("radioButton_6"));
        radioButton_6->setGeometry(QRect(250, 210, 112, 23));
        pushButton_11 = new QPushButton(tab_4);
        pushButton_11->setObjectName(QString::fromUtf8("pushButton_11"));
        pushButton_11->setGeometry(QRect(470, 210, 89, 25));
        pushButton_12 = new QPushButton(tab_4);
        pushButton_12->setObjectName(QString::fromUtf8("pushButton_12"));
        pushButton_12->setGeometry(QRect(30, 240, 89, 25));
        pushButton_13 = new QPushButton(tab_4);
        pushButton_13->setObjectName(QString::fromUtf8("pushButton_13"));
        pushButton_13->setGeometry(QRect(470, 280, 89, 25));
        label_5 = new QLabel(tab_4);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(30, 40, 67, 17));
        label_6 = new QLabel(tab_4);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(30, 110, 67, 17));
        lineEdit_5 = new QLineEdit(tab_4);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));
        lineEdit_5->setGeometry(QRect(100, 40, 401, 25));
        lineEdit_6 = new QLineEdit(tab_4);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));
        lineEdit_6->setGeometry(QRect(100, 100, 401, 25));
        toolButton_5 = new QToolButton(tab_4);
        toolButton_5->setObjectName(QString::fromUtf8("toolButton_5"));
        toolButton_5->setGeometry(QRect(530, 40, 26, 24));
        toolButton_6 = new QToolButton(tab_4);
        toolButton_6->setObjectName(QString::fromUtf8("toolButton_6"));
        toolButton_6->setGeometry(QRect(530, 100, 26, 24));
        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        tabWidget->addTab(tab_5, QString());
        listView = new QListView(groupBox_2);
        listView->setObjectName(QString::fromUtf8("listView"));
        listView->setGeometry(QRect(0, 440, 581, 671));

        retranslateUi(Dialog);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QCoreApplication::translate("Dialog", "3D Reconstruction", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Dialog", "Map", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("Dialog", "Control", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("Dialog", "Input", nullptr));
        radioButton->setText(QCoreApplication::translate("Dialog", "SIFT", nullptr));
        pushButton->setText(QCoreApplication::translate("Dialog", "SequentialPipeline", nullptr));
        toolButton->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        toolButton_2->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        checkBox->setText(QCoreApplication::translate("Dialog", "MaxPair", nullptr));
        label->setText(QCoreApplication::translate("Dialog", "Putin", nullptr));
        label_2->setText(QCoreApplication::translate("Dialog", "Putout", nullptr));
        radioButton_2->setText(QCoreApplication::translate("Dialog", "AKAZE", nullptr));
        radioButton_3->setText(QCoreApplication::translate("Dialog", "ORB", nullptr));
        radioButton_4->setText(QCoreApplication::translate("Dialog", "HARRIS", nullptr));
        checkBox_2->setText(QCoreApplication::translate("Dialog", "Stellar", nullptr));
        pushButton_2->setText(QCoreApplication::translate("Dialog", "Match", nullptr));
        pushButton_3->setText(QCoreApplication::translate("Dialog", "ComputeFeatures", nullptr));
        pushButton_4->setText(QCoreApplication::translate("Dialog", "SfMInit", nullptr));
        pushButton_5->setText(QCoreApplication::translate("Dialog", "IncrementalSfM", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("Dialog", "SFM", nullptr));
        pushButton_6->setText(QCoreApplication::translate("Dialog", "DensifyPointCloud", nullptr));
        pushButton_7->setText(QCoreApplication::translate("Dialog", "ReconstructMesh", nullptr));
        pushButton_8->setText(QCoreApplication::translate("Dialog", "RefineMesh", nullptr));
        pushButton_9->setText(QCoreApplication::translate("Dialog", "TextureMesh", nullptr));
        pushButton_10->setText(QCoreApplication::translate("Dialog", "MVSPipeline", nullptr));
        label_3->setText(QCoreApplication::translate("Dialog", "Putin", nullptr));
        label_4->setText(QCoreApplication::translate("Dialog", "Putout", nullptr));
        toolButton_3->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        toolButton_4->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("Dialog", "MVS", nullptr));
        radioButton_5->setText(QCoreApplication::translate("Dialog", "Delaunay Tetrahedralization", nullptr));
        radioButton_6->setText(QCoreApplication::translate("Dialog", "Poisson", nullptr));
        pushButton_11->setText(QCoreApplication::translate("Dialog", "Compute", nullptr));
        pushButton_12->setText(QCoreApplication::translate("Dialog", "original", nullptr));
        pushButton_13->setText(QCoreApplication::translate("Dialog", "Save", nullptr));
        label_5->setText(QCoreApplication::translate("Dialog", "Putin", nullptr));
        label_6->setText(QCoreApplication::translate("Dialog", "Putout", nullptr));
        toolButton_5->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        toolButton_6->setText(QCoreApplication::translate("Dialog", "...", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("Dialog", "Viewer", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QCoreApplication::translate("Dialog", "ICP Cloud", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
