/********************************************************************************
** Form generated from reading UI file 'movieguiRE8354.ui'
**
** Created: Tue Jun 5 10:30:30 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MOVIEGUIRE8354_H
#define MOVIEGUIRE8354_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QGridLayout *gridLayout_2;
    QFrame *frame;
    QGridLayout *gridLayout;
    QPushButton *BrowseButton;
    QPushButton *NewButton;
    QPushButton *ImportObjectButton;
    QLineEdit *statehistoryEdit;
    QLineEdit *framerateEdit;
    QPushButton *DeleteObjectButton;
    QLabel *label_2;
    QLabel *label;
    QCheckBox *SaveCheckBox;
    QPushButton *playMovieButton;
    QPushButton *SaveButton;
    QPushButton *importStateFile;
    QTableWidget *tableWidget;
    QLabel *label_3;
    QLineEdit *worldScaleEdit;
    QLabel *statusBar;
    QPushButton *convertButton;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(830, 308);
        gridLayout_2 = new QGridLayout(Dialog);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        frame = new QFrame(Dialog);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::NoFrame);
        frame->setFrameShadow(QFrame::Plain);
        gridLayout = new QGridLayout(frame);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        BrowseButton = new QPushButton(frame);
        BrowseButton->setObjectName(QString::fromUtf8("BrowseButton"));

        gridLayout->addWidget(BrowseButton, 3, 5, 1, 2);

        NewButton = new QPushButton(frame);
        NewButton->setObjectName(QString::fromUtf8("NewButton"));

        gridLayout->addWidget(NewButton, 0, 5, 1, 2);

        ImportObjectButton = new QPushButton(frame);
        ImportObjectButton->setObjectName(QString::fromUtf8("ImportObjectButton"));

        gridLayout->addWidget(ImportObjectButton, 5, 5, 1, 2);

        statehistoryEdit = new QLineEdit(frame);
        statehistoryEdit->setObjectName(QString::fromUtf8("statehistoryEdit"));

        gridLayout->addWidget(statehistoryEdit, 9, 1, 1, 4);

        framerateEdit = new QLineEdit(frame);
        framerateEdit->setObjectName(QString::fromUtf8("framerateEdit"));

        gridLayout->addWidget(framerateEdit, 10, 1, 1, 1);

        DeleteObjectButton = new QPushButton(frame);
        DeleteObjectButton->setObjectName(QString::fromUtf8("DeleteObjectButton"));

        gridLayout->addWidget(DeleteObjectButton, 6, 5, 1, 2);

        label_2 = new QLabel(frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 10, 0, 1, 1);

        label = new QLabel(frame);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 9, 0, 1, 1);

        SaveCheckBox = new QCheckBox(frame);
        SaveCheckBox->setObjectName(QString::fromUtf8("SaveCheckBox"));

        gridLayout->addWidget(SaveCheckBox, 10, 4, 1, 1);

        playMovieButton = new QPushButton(frame);
        playMovieButton->setObjectName(QString::fromUtf8("playMovieButton"));

        gridLayout->addWidget(playMovieButton, 10, 5, 1, 2);

        SaveButton = new QPushButton(frame);
        SaveButton->setObjectName(QString::fromUtf8("SaveButton"));

        gridLayout->addWidget(SaveButton, 9, 5, 1, 2);

        importStateFile = new QPushButton(frame);
        importStateFile->setObjectName(QString::fromUtf8("importStateFile"));

        gridLayout->addWidget(importStateFile, 4, 5, 1, 2);

        tableWidget = new QTableWidget(frame);
        if (tableWidget->columnCount() < 2)
            tableWidget->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));

        gridLayout->addWidget(tableWidget, 0, 0, 9, 5);

        label_3 = new QLabel(frame);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 10, 2, 1, 1);

        worldScaleEdit = new QLineEdit(frame);
        worldScaleEdit->setObjectName(QString::fromUtf8("worldScaleEdit"));

        gridLayout->addWidget(worldScaleEdit, 10, 3, 1, 1);


        gridLayout_2->addWidget(frame, 0, 0, 1, 2);

        statusBar = new QLabel(Dialog);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));

        gridLayout_2->addWidget(statusBar, 1, 0, 1, 1);

        convertButton = new QPushButton(Dialog);
        convertButton->setObjectName(QString::fromUtf8("convertButton"));

        gridLayout_2->addWidget(convertButton, 1, 1, 1, 1);


        retranslateUi(Dialog);

        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0, QApplication::UnicodeUTF8));
        BrowseButton->setText(QApplication::translate("Dialog", "Open Project", 0, QApplication::UnicodeUTF8));
        NewButton->setText(QApplication::translate("Dialog", "New...", 0, QApplication::UnicodeUTF8));
        ImportObjectButton->setText(QApplication::translate("Dialog", "Import Object", 0, QApplication::UnicodeUTF8));
        DeleteObjectButton->setText(QApplication::translate("Dialog", "Delete Object", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Dialog", "Frame Rate", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Dialog", "Statehistory File:", 0, QApplication::UnicodeUTF8));
        SaveCheckBox->setText(QApplication::translate("Dialog", "Save JPEGS", 0, QApplication::UnicodeUTF8));
        playMovieButton->setText(QApplication::translate("Dialog", "Play Movie", 0, QApplication::UnicodeUTF8));
        SaveButton->setText(QApplication::translate("Dialog", "Save", 0, QApplication::UnicodeUTF8));
        importStateFile->setText(QApplication::translate("Dialog", "Import State File", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("Dialog", "Object Name", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("Dialog", "Scale", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Dialog", "World Scale", 0, QApplication::UnicodeUTF8));
        statusBar->setText(QApplication::translate("Dialog", "Multi Object Visualizer with Interactive Environment(MOVIE) - Created: Carlos Montalvo March 2012", 0, QApplication::UnicodeUTF8));
        convertButton->setText(QApplication::translate("Dialog", "BMP2AVI", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MOVIEGUIRE8354_H
