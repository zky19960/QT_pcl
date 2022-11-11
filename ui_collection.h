#pragma once

#ifndef _UI_COLLECTION_H_
#define _UI_COLLECTION_H_

#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")
#endif

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

class UI
{
	public://控件指针
		QAction *action;
		QAction *action_2;
		QAction *action_3;
		QAction *action_4;
		QAction *action_5;
		QWidget *centralwidget;
		QWidget *layoutWidget;
		QHBoxLayout *horizontalLayout_3;
		QSpacerItem *horizontalSpacer_2;
		QVBoxLayout *verticalLayout_7;
		QSpacerItem *verticalSpacer;
		QGridLayout *gridLayout_2;
		QHBoxLayout *horizontalLayout;
		QLabel *label;
		QComboBox *comboBox;
		QSpacerItem *horizontalSpacer;
		QGridLayout *gridLayout;
		QVBoxLayout *verticalLayout_5;
		QLabel *label_8;
		QTabWidget *tabWidget_3;
		QWidget *tab_9;
		QWidget *verticalLayoutWidget_3;
		QVBoxLayout *verticalLayout_3;
		QVTKWidget *qvtkWidget2_10;
		QWidget *tab_10;
		QWidget *verticalLayoutWidget_2;
		QVBoxLayout *verticalLayout;
		QVTKWidget *qvtkWidget2_11;
		QWidget *tab_11;
		QWidget *horizontalLayoutWidget;
		QHBoxLayout *horizontalLayout_2;
		QVTKWidget *qvtkWidget2_12;
		QWidget *tab_12;
		QWidget *verticalLayoutWidget;
		QVBoxLayout *verticalLayout_8;
		QVTKWidget *qvtkWidget2_13;
		QLabel *label_9;
		QVBoxLayout *verticalLayout_2;
		QLabel *label_2;
		QSpacerItem *verticalSpacer_4;
		QTableWidget *tableWidget;
		QVBoxLayout *verticalLayout_6;
		QLabel *label_10;
		QSpacerItem *verticalSpacer_3;
		QTableWidget *tableWidget_3;
		QLineEdit *lineEdit;
		QSpacerItem *verticalSpacer_2;
		QSpacerItem *horizontalSpacer_3;
		QMenuBar *menubar;
		QMenu *menu;
		QMenu *menu_2;
		QMenu *menu_3;
		QMenu *menu_4;
		QMenu *menu_5;
		QStatusBar *statusbar;

	void InitUI(QMainWindow* MainWindow)
	{
		SetupUI(MainWindow);
		retranslateUi(MainWindow);
		QMetaObject::connectSlotsByName(MainWindow);
		
		QFont font;
		font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
		font.setPointSize(14);
		layoutWidget->setFont(font);

		centralwidget->setLayout(horizontalLayout_3);
		tab_9->setLayout(verticalLayout_3);
		tab_10->setLayout(verticalLayout);
		tab_11->setLayout(horizontalLayout_2);
		tab_12->setLayout(verticalLayout_8);
		tabWidget_3->setCurrentIndex(0);
	}

	private:
	void SetupUI(QMainWindow* MainWindow)
	{
		if (MainWindow->objectName().isEmpty())
			MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
		MainWindow->resize(920, 642);
		QFont font;
		font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
		font.setPointSize(18);
		MainWindow->setFont(font);
		action = new QAction(MainWindow);
		action->setObjectName(QString::fromUtf8("action"));
		action->setFont(font);
		action_2 = new QAction(MainWindow);
		action_2->setObjectName(QString::fromUtf8("action_2"));
		action_2->setFont(font);
		action_3 = new QAction(MainWindow);
		action_3->setObjectName(QString::fromUtf8("action_3"));
		action_3->setFont(font);
		action_4 = new QAction(MainWindow);
		action_4->setObjectName(QString::fromUtf8("action_4"));
		action_4->setFont(font);
		action_5 = new QAction(MainWindow);
		action_5->setObjectName(QString::fromUtf8("action_5"));
		action_5->setFont(font);
		centralwidget = new QWidget(MainWindow);
		centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
		centralwidget->setFont(font);
		layoutWidget = new QWidget(centralwidget);
		layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
		layoutWidget->setGeometry(QRect(0, 10, 891, 571));
		horizontalLayout_3 = new QHBoxLayout(layoutWidget);
		horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
		horizontalLayout_3->setSizeConstraint(QLayout::SetDefaultConstraint);
		horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
		horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

		horizontalLayout_3->addItem(horizontalSpacer_2);

		verticalLayout_7 = new QVBoxLayout();
		verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
		verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

		verticalLayout_7->addItem(verticalSpacer);

		gridLayout_2 = new QGridLayout();
		gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
		gridLayout_2->setSizeConstraint(QLayout::SetMinimumSize);
		gridLayout_2->setVerticalSpacing(7);
		horizontalLayout = new QHBoxLayout();
		horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
		label = new QLabel(layoutWidget);
		label->setObjectName(QString::fromUtf8("label"));

		horizontalLayout->addWidget(label);

		comboBox = new QComboBox(layoutWidget);
		comboBox->setObjectName(QString::fromUtf8("comboBox"));

		horizontalLayout->addWidget(comboBox);

		horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

		horizontalLayout->addItem(horizontalSpacer);

		horizontalLayout->setStretch(1, 20);
		horizontalLayout->setStretch(2, 80);

		gridLayout_2->addLayout(horizontalLayout, 0, 0, 1, 1);

		gridLayout = new QGridLayout();
		gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
		gridLayout->setHorizontalSpacing(7);
		verticalLayout_5 = new QVBoxLayout();
		verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
		verticalLayout_5->setSizeConstraint(QLayout::SetMinAndMaxSize);
		label_8 = new QLabel(layoutWidget);
		label_8->setObjectName(QString::fromUtf8("label_8"));
		label_8->setAlignment(Qt::AlignCenter);

		verticalLayout_5->addWidget(label_8);

		tabWidget_3 = new QTabWidget(layoutWidget);
		tabWidget_3->setObjectName(QString::fromUtf8("tabWidget_3"));
		tab_9 = new QWidget();
		tab_9->setObjectName(QString::fromUtf8("tab_9"));
		verticalLayoutWidget_3 = new QWidget(tab_9);
		verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
		verticalLayoutWidget_3->setGeometry(QRect(10, 10, 331, 411));
		verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
		verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
		verticalLayout_3->setSizeConstraint(QLayout::SetDefaultConstraint);
		verticalLayout_3->setContentsMargins(0, 0, 0, 0);
		qvtkWidget2_10 = new QVTKWidget(verticalLayoutWidget_3);
		qvtkWidget2_10->setObjectName(QString::fromUtf8("qvtkWidget2_10"));

		verticalLayout_3->addWidget(qvtkWidget2_10);

		tabWidget_3->addTab(tab_9, QString());
		tab_10 = new QWidget();
		tab_10->setObjectName(QString::fromUtf8("tab_10"));
		verticalLayoutWidget_2 = new QWidget(tab_10);
		verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
		verticalLayoutWidget_2->setGeometry(QRect(0, 9, 351, 411));
		verticalLayout = new QVBoxLayout(verticalLayoutWidget_2);
		verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
		verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
		verticalLayout->setContentsMargins(0, 0, 0, 0);
		qvtkWidget2_11 = new QVTKWidget(verticalLayoutWidget_2);
		qvtkWidget2_11->setObjectName(QString::fromUtf8("qvtkWidget2_11"));

		verticalLayout->addWidget(qvtkWidget2_11);

		tabWidget_3->addTab(tab_10, QString());
		tab_11 = new QWidget();
		tab_11->setObjectName(QString::fromUtf8("tab_11"));
		horizontalLayoutWidget = new QWidget(tab_11);
		horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
		horizontalLayoutWidget->setGeometry(QRect(0, -1, 351, 421));
		horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget);
		horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
		horizontalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
		horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
		qvtkWidget2_12 = new QVTKWidget(horizontalLayoutWidget);
		qvtkWidget2_12->setObjectName(QString::fromUtf8("qvtkWidget2_12"));

		horizontalLayout_2->addWidget(qvtkWidget2_12);

		tabWidget_3->addTab(tab_11, QString());
		tab_12 = new QWidget();
		tab_12->setObjectName(QString::fromUtf8("tab_12"));
		verticalLayoutWidget = new QWidget(tab_12);
		verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
		verticalLayoutWidget->setGeometry(QRect(0, 0, 301, 401));
		verticalLayout_8 = new QVBoxLayout(verticalLayoutWidget);
		verticalLayout_8->setSpacing(7);
		verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
		verticalLayout_8->setSizeConstraint(QLayout::SetDefaultConstraint);
		verticalLayout_8->setContentsMargins(0, 0, 0, 0);
		qvtkWidget2_13 = new QVTKWidget(verticalLayoutWidget);
		qvtkWidget2_13->setObjectName(QString::fromUtf8("qvtkWidget2_13"));

		verticalLayout_8->addWidget(qvtkWidget2_13);

		label_9 = new QLabel(verticalLayoutWidget);
		label_9->setObjectName(QString::fromUtf8("label_9"));
		label_9->setAlignment(Qt::AlignCenter);

		verticalLayout_8->addWidget(label_9);

		verticalLayout_8->setStretch(0, 16);
		verticalLayout_8->setStretch(1, 1);
		tabWidget_3->addTab(tab_12, QString());

		verticalLayout_5->addWidget(tabWidget_3);


		gridLayout->addLayout(verticalLayout_5, 0, 1, 1, 1);

		verticalLayout_2 = new QVBoxLayout();
		verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
		label_2 = new QLabel(layoutWidget);
		label_2->setObjectName(QString::fromUtf8("label_2"));
		label_2->setAlignment(Qt::AlignCenter);

		verticalLayout_2->addWidget(label_2);

		verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

		verticalLayout_2->addItem(verticalSpacer_4);

		tableWidget = new QTableWidget(layoutWidget);
		if (tableWidget->columnCount() < 2)
			tableWidget->setColumnCount(2);
		QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
		tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
		QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
		tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
		if (tableWidget->rowCount() < 13)
			tableWidget->setRowCount(13);
		QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(0, __qtablewidgetitem2);
		QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(1, __qtablewidgetitem3);
		QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(2, __qtablewidgetitem4);
		QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(3, __qtablewidgetitem5);
		QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(4, __qtablewidgetitem6);
		QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(5, __qtablewidgetitem7);
		QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(6, __qtablewidgetitem8);
		QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(7, __qtablewidgetitem9);
		QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(8, __qtablewidgetitem10);
		QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(9, __qtablewidgetitem11);
		QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(10, __qtablewidgetitem12);
		QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(11, __qtablewidgetitem13);
		QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
		tableWidget->setVerticalHeaderItem(12, __qtablewidgetitem14);
		QFont font1;
		font1.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
		font1.setPointSize(14);
		QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
		__qtablewidgetitem15->setFont(font1);
		tableWidget->setItem(0, 0, __qtablewidgetitem15);
		QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
		__qtablewidgetitem16->setFont(font1);
		tableWidget->setItem(0, 1, __qtablewidgetitem16);
		QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
		__qtablewidgetitem17->setFont(font1);
		tableWidget->setItem(1, 0, __qtablewidgetitem17);
		QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
		__qtablewidgetitem18->setFont(font1);
		tableWidget->setItem(1, 1, __qtablewidgetitem18);
		QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
		__qtablewidgetitem19->setFont(font1);
		tableWidget->setItem(2, 0, __qtablewidgetitem19);
		QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
		__qtablewidgetitem20->setFont(font1);
		tableWidget->setItem(2, 1, __qtablewidgetitem20);
		QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
		__qtablewidgetitem21->setFont(font1);
		tableWidget->setItem(3, 0, __qtablewidgetitem21);
		QTableWidgetItem *__qtablewidgetitem22 = new QTableWidgetItem();
		__qtablewidgetitem22->setFont(font1);
		tableWidget->setItem(3, 1, __qtablewidgetitem22);
		QTableWidgetItem *__qtablewidgetitem23 = new QTableWidgetItem();
		__qtablewidgetitem23->setFont(font1);
		tableWidget->setItem(4, 0, __qtablewidgetitem23);
		QTableWidgetItem *__qtablewidgetitem24 = new QTableWidgetItem();
		__qtablewidgetitem24->setFont(font1);
		tableWidget->setItem(4, 1, __qtablewidgetitem24);
		QTableWidgetItem *__qtablewidgetitem25 = new QTableWidgetItem();
		__qtablewidgetitem25->setFont(font1);
		tableWidget->setItem(5, 0, __qtablewidgetitem25);
		QTableWidgetItem *__qtablewidgetitem26 = new QTableWidgetItem();
		__qtablewidgetitem26->setFont(font1);
		tableWidget->setItem(5, 1, __qtablewidgetitem26);
		QTableWidgetItem *__qtablewidgetitem27 = new QTableWidgetItem();
		__qtablewidgetitem27->setFont(font1);
		tableWidget->setItem(6, 0, __qtablewidgetitem27);
		QTableWidgetItem *__qtablewidgetitem28 = new QTableWidgetItem();
		__qtablewidgetitem28->setFont(font1);
		tableWidget->setItem(6, 1, __qtablewidgetitem28);
		QTableWidgetItem *__qtablewidgetitem29 = new QTableWidgetItem();
		__qtablewidgetitem29->setFont(font1);
		tableWidget->setItem(7, 0, __qtablewidgetitem29);
		QTableWidgetItem *__qtablewidgetitem30 = new QTableWidgetItem();
		__qtablewidgetitem30->setFont(font1);
		tableWidget->setItem(7, 1, __qtablewidgetitem30);
		QTableWidgetItem *__qtablewidgetitem31 = new QTableWidgetItem();
		__qtablewidgetitem31->setFont(font1);
		tableWidget->setItem(8, 0, __qtablewidgetitem31);
		QTableWidgetItem *__qtablewidgetitem32 = new QTableWidgetItem();
		__qtablewidgetitem32->setFont(font1);
		tableWidget->setItem(8, 1, __qtablewidgetitem32);
		QTableWidgetItem *__qtablewidgetitem33 = new QTableWidgetItem();
		__qtablewidgetitem33->setFont(font1);
		tableWidget->setItem(9, 0, __qtablewidgetitem33);
		QTableWidgetItem *__qtablewidgetitem34 = new QTableWidgetItem();
		__qtablewidgetitem34->setFont(font1);
		tableWidget->setItem(9, 1, __qtablewidgetitem34);
		QTableWidgetItem *__qtablewidgetitem35 = new QTableWidgetItem();
		__qtablewidgetitem35->setFont(font1);
		tableWidget->setItem(10, 0, __qtablewidgetitem35);
		QTableWidgetItem *__qtablewidgetitem36 = new QTableWidgetItem();
		__qtablewidgetitem36->setFont(font1);
		tableWidget->setItem(10, 1, __qtablewidgetitem36);
		QTableWidgetItem *__qtablewidgetitem37 = new QTableWidgetItem();
		__qtablewidgetitem37->setFont(font1);
		tableWidget->setItem(11, 0, __qtablewidgetitem37);
		QTableWidgetItem *__qtablewidgetitem38 = new QTableWidgetItem();
		__qtablewidgetitem38->setFont(font1);
		tableWidget->setItem(11, 1, __qtablewidgetitem38);
		tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
		tableWidget->setFont(font1);
		tableWidget->horizontalHeader()->setStretchLastSection(true);
		tableWidget->verticalHeader()->setStretchLastSection(true);

		verticalLayout_2->addWidget(tableWidget);

		verticalLayout_2->setStretch(0, 1);
		verticalLayout_2->setStretch(1, 1);
		verticalLayout_2->setStretch(2, 50);

		gridLayout->addLayout(verticalLayout_2, 0, 0, 1, 1);

		verticalLayout_6 = new QVBoxLayout();
		verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
		label_10 = new QLabel(layoutWidget);
		label_10->setObjectName(QString::fromUtf8("label_10"));
		label_10->setAlignment(Qt::AlignCenter);

		verticalLayout_6->addWidget(label_10);

		verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

		verticalLayout_6->addItem(verticalSpacer_3);

		tableWidget_3 = new QTableWidget(layoutWidget);
		if (tableWidget_3->columnCount() < 5)
			tableWidget_3->setColumnCount(5);
		QTableWidgetItem *__qtablewidgetitem39 = new QTableWidgetItem();
		tableWidget_3->setHorizontalHeaderItem(0, __qtablewidgetitem39);
		QTableWidgetItem *__qtablewidgetitem40 = new QTableWidgetItem();
		tableWidget_3->setHorizontalHeaderItem(1, __qtablewidgetitem40);
		QTableWidgetItem *__qtablewidgetitem41 = new QTableWidgetItem();
		tableWidget_3->setHorizontalHeaderItem(2, __qtablewidgetitem41);
		QTableWidgetItem *__qtablewidgetitem42 = new QTableWidgetItem();
		tableWidget_3->setHorizontalHeaderItem(3, __qtablewidgetitem42);
		QTableWidgetItem *__qtablewidgetitem43 = new QTableWidgetItem();
		tableWidget_3->setHorizontalHeaderItem(4, __qtablewidgetitem43);
		if (tableWidget_3->rowCount() < 13)
			tableWidget_3->setRowCount(13);
		QTableWidgetItem *__qtablewidgetitem44 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(0, __qtablewidgetitem44);
		QTableWidgetItem *__qtablewidgetitem45 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(1, __qtablewidgetitem45);
		QTableWidgetItem *__qtablewidgetitem46 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(2, __qtablewidgetitem46);
		QTableWidgetItem *__qtablewidgetitem47 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(3, __qtablewidgetitem47);
		QTableWidgetItem *__qtablewidgetitem48 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(4, __qtablewidgetitem48);
		QTableWidgetItem *__qtablewidgetitem49 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(5, __qtablewidgetitem49);
		QTableWidgetItem *__qtablewidgetitem50 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(6, __qtablewidgetitem50);
		QTableWidgetItem *__qtablewidgetitem51 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(7, __qtablewidgetitem51);
		QTableWidgetItem *__qtablewidgetitem52 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(8, __qtablewidgetitem52);
		QTableWidgetItem *__qtablewidgetitem53 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(9, __qtablewidgetitem53);
		QTableWidgetItem *__qtablewidgetitem54 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(10, __qtablewidgetitem54);
		QTableWidgetItem *__qtablewidgetitem55 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(11, __qtablewidgetitem55);
		QTableWidgetItem *__qtablewidgetitem56 = new QTableWidgetItem();
		tableWidget_3->setVerticalHeaderItem(12, __qtablewidgetitem56);
		QTableWidgetItem *__qtablewidgetitem57 = new QTableWidgetItem();
		__qtablewidgetitem57->setFont(font1);
		tableWidget_3->setItem(0, 0, __qtablewidgetitem57);
		QTableWidgetItem *__qtablewidgetitem58 = new QTableWidgetItem();
		__qtablewidgetitem58->setFont(font1);
		tableWidget_3->setItem(0, 1, __qtablewidgetitem58);
		QTableWidgetItem *__qtablewidgetitem59 = new QTableWidgetItem();
		__qtablewidgetitem59->setFont(font1);
		tableWidget_3->setItem(0, 2, __qtablewidgetitem59);
		QTableWidgetItem *__qtablewidgetitem60 = new QTableWidgetItem();
		__qtablewidgetitem60->setFont(font1);
		tableWidget_3->setItem(0, 3, __qtablewidgetitem60);
		QTableWidgetItem *__qtablewidgetitem61 = new QTableWidgetItem();
		__qtablewidgetitem61->setFont(font1);
		tableWidget_3->setItem(0, 4, __qtablewidgetitem61);
		QTableWidgetItem *__qtablewidgetitem62 = new QTableWidgetItem();
		__qtablewidgetitem62->setFont(font1);
		tableWidget_3->setItem(1, 0, __qtablewidgetitem62);
		QTableWidgetItem *__qtablewidgetitem63 = new QTableWidgetItem();
		__qtablewidgetitem63->setFont(font1);
		tableWidget_3->setItem(1, 1, __qtablewidgetitem63);
		QTableWidgetItem *__qtablewidgetitem64 = new QTableWidgetItem();
		__qtablewidgetitem64->setFont(font1);
		tableWidget_3->setItem(1, 2, __qtablewidgetitem64);
		QTableWidgetItem *__qtablewidgetitem65 = new QTableWidgetItem();
		__qtablewidgetitem65->setFont(font1);
		tableWidget_3->setItem(1, 3, __qtablewidgetitem65);
		QTableWidgetItem *__qtablewidgetitem66 = new QTableWidgetItem();
		__qtablewidgetitem66->setFont(font1);
		tableWidget_3->setItem(1, 4, __qtablewidgetitem66);
		QTableWidgetItem *__qtablewidgetitem67 = new QTableWidgetItem();
		__qtablewidgetitem67->setFont(font1);
		tableWidget_3->setItem(2, 0, __qtablewidgetitem67);
		QTableWidgetItem *__qtablewidgetitem68 = new QTableWidgetItem();
		__qtablewidgetitem68->setFont(font1);
		tableWidget_3->setItem(2, 1, __qtablewidgetitem68);
		QTableWidgetItem *__qtablewidgetitem69 = new QTableWidgetItem();
		__qtablewidgetitem69->setFont(font1);
		tableWidget_3->setItem(2, 2, __qtablewidgetitem69);
		QTableWidgetItem *__qtablewidgetitem70 = new QTableWidgetItem();
		__qtablewidgetitem70->setFont(font1);
		tableWidget_3->setItem(2, 3, __qtablewidgetitem70);
		QTableWidgetItem *__qtablewidgetitem71 = new QTableWidgetItem();
		__qtablewidgetitem71->setFont(font1);
		tableWidget_3->setItem(2, 4, __qtablewidgetitem71);
		QTableWidgetItem *__qtablewidgetitem72 = new QTableWidgetItem();
		__qtablewidgetitem72->setFont(font1);
		tableWidget_3->setItem(3, 0, __qtablewidgetitem72);
		QTableWidgetItem *__qtablewidgetitem73 = new QTableWidgetItem();
		__qtablewidgetitem73->setFont(font1);
		tableWidget_3->setItem(3, 1, __qtablewidgetitem73);
		QTableWidgetItem *__qtablewidgetitem74 = new QTableWidgetItem();
		__qtablewidgetitem74->setFont(font1);
		tableWidget_3->setItem(3, 2, __qtablewidgetitem74);
		QTableWidgetItem *__qtablewidgetitem75 = new QTableWidgetItem();
		__qtablewidgetitem75->setFont(font1);
		tableWidget_3->setItem(3, 3, __qtablewidgetitem75);
		QTableWidgetItem *__qtablewidgetitem76 = new QTableWidgetItem();
		__qtablewidgetitem76->setFont(font1);
		tableWidget_3->setItem(3, 4, __qtablewidgetitem76);
		QTableWidgetItem *__qtablewidgetitem77 = new QTableWidgetItem();
		__qtablewidgetitem77->setFont(font1);
		tableWidget_3->setItem(4, 1, __qtablewidgetitem77);
		QTableWidgetItem *__qtablewidgetitem78 = new QTableWidgetItem();
		__qtablewidgetitem78->setFont(font1);
		tableWidget_3->setItem(4, 2, __qtablewidgetitem78);
		QTableWidgetItem *__qtablewidgetitem79 = new QTableWidgetItem();
		__qtablewidgetitem79->setFont(font1);
		tableWidget_3->setItem(4, 3, __qtablewidgetitem79);
		QTableWidgetItem *__qtablewidgetitem80 = new QTableWidgetItem();
		__qtablewidgetitem80->setFont(font1);
		tableWidget_3->setItem(4, 4, __qtablewidgetitem80);
		QTableWidgetItem *__qtablewidgetitem81 = new QTableWidgetItem();
		__qtablewidgetitem81->setFont(font1);
		tableWidget_3->setItem(5, 0, __qtablewidgetitem81);
		QTableWidgetItem *__qtablewidgetitem82 = new QTableWidgetItem();
		__qtablewidgetitem82->setFont(font1);
		tableWidget_3->setItem(5, 1, __qtablewidgetitem82);
		QTableWidgetItem *__qtablewidgetitem83 = new QTableWidgetItem();
		__qtablewidgetitem83->setFont(font1);
		tableWidget_3->setItem(5, 2, __qtablewidgetitem83);
		QTableWidgetItem *__qtablewidgetitem84 = new QTableWidgetItem();
		__qtablewidgetitem84->setFont(font1);
		tableWidget_3->setItem(5, 3, __qtablewidgetitem84);
		QTableWidgetItem *__qtablewidgetitem85 = new QTableWidgetItem();
		__qtablewidgetitem85->setFont(font1);
		tableWidget_3->setItem(5, 4, __qtablewidgetitem85);
		QTableWidgetItem *__qtablewidgetitem86 = new QTableWidgetItem();
		__qtablewidgetitem86->setFont(font1);
		tableWidget_3->setItem(6, 0, __qtablewidgetitem86);
		QTableWidgetItem *__qtablewidgetitem87 = new QTableWidgetItem();
		__qtablewidgetitem87->setFont(font1);
		tableWidget_3->setItem(6, 1, __qtablewidgetitem87);
		QTableWidgetItem *__qtablewidgetitem88 = new QTableWidgetItem();
		__qtablewidgetitem88->setFont(font1);
		tableWidget_3->setItem(6, 2, __qtablewidgetitem88);
		QTableWidgetItem *__qtablewidgetitem89 = new QTableWidgetItem();
		__qtablewidgetitem89->setFont(font1);
		tableWidget_3->setItem(6, 3, __qtablewidgetitem89);
		QTableWidgetItem *__qtablewidgetitem90 = new QTableWidgetItem();
		__qtablewidgetitem90->setFont(font1);
		tableWidget_3->setItem(6, 4, __qtablewidgetitem90);
		QTableWidgetItem *__qtablewidgetitem91 = new QTableWidgetItem();
		__qtablewidgetitem91->setFont(font1);
		tableWidget_3->setItem(7, 0, __qtablewidgetitem91);
		QTableWidgetItem *__qtablewidgetitem92 = new QTableWidgetItem();
		__qtablewidgetitem92->setFont(font1);
		tableWidget_3->setItem(7, 1, __qtablewidgetitem92);
		QTableWidgetItem *__qtablewidgetitem93 = new QTableWidgetItem();
		__qtablewidgetitem93->setFont(font1);
		tableWidget_3->setItem(7, 2, __qtablewidgetitem93);
		QTableWidgetItem *__qtablewidgetitem94 = new QTableWidgetItem();
		__qtablewidgetitem94->setFont(font1);
		tableWidget_3->setItem(7, 3, __qtablewidgetitem94);
		QTableWidgetItem *__qtablewidgetitem95 = new QTableWidgetItem();
		__qtablewidgetitem95->setFont(font1);
		tableWidget_3->setItem(7, 4, __qtablewidgetitem95);
		QTableWidgetItem *__qtablewidgetitem96 = new QTableWidgetItem();
		__qtablewidgetitem96->setFont(font1);
		tableWidget_3->setItem(8, 0, __qtablewidgetitem96);
		QTableWidgetItem *__qtablewidgetitem97 = new QTableWidgetItem();
		__qtablewidgetitem97->setFont(font1);
		tableWidget_3->setItem(8, 1, __qtablewidgetitem97);
		QTableWidgetItem *__qtablewidgetitem98 = new QTableWidgetItem();
		__qtablewidgetitem98->setFont(font1);
		tableWidget_3->setItem(8, 2, __qtablewidgetitem98);
		QTableWidgetItem *__qtablewidgetitem99 = new QTableWidgetItem();
		__qtablewidgetitem99->setFont(font1);
		tableWidget_3->setItem(8, 3, __qtablewidgetitem99);
		QTableWidgetItem *__qtablewidgetitem100 = new QTableWidgetItem();
		__qtablewidgetitem100->setFont(font1);
		tableWidget_3->setItem(8, 4, __qtablewidgetitem100);
		QTableWidgetItem *__qtablewidgetitem101 = new QTableWidgetItem();
		__qtablewidgetitem101->setFont(font1);
		tableWidget_3->setItem(9, 0, __qtablewidgetitem101);
		QTableWidgetItem *__qtablewidgetitem102 = new QTableWidgetItem();
		__qtablewidgetitem102->setFont(font1);
		tableWidget_3->setItem(9, 1, __qtablewidgetitem102);
		QTableWidgetItem *__qtablewidgetitem103 = new QTableWidgetItem();
		__qtablewidgetitem103->setFont(font1);
		tableWidget_3->setItem(9, 2, __qtablewidgetitem103);
		QTableWidgetItem *__qtablewidgetitem104 = new QTableWidgetItem();
		__qtablewidgetitem104->setFont(font1);
		tableWidget_3->setItem(9, 3, __qtablewidgetitem104);
		QTableWidgetItem *__qtablewidgetitem105 = new QTableWidgetItem();
		__qtablewidgetitem105->setFont(font1);
		tableWidget_3->setItem(9, 4, __qtablewidgetitem105);
		QTableWidgetItem *__qtablewidgetitem106 = new QTableWidgetItem();
		__qtablewidgetitem106->setFont(font1);
		tableWidget_3->setItem(10, 0, __qtablewidgetitem106);
		QTableWidgetItem *__qtablewidgetitem107 = new QTableWidgetItem();
		__qtablewidgetitem107->setFont(font1);
		tableWidget_3->setItem(10, 1, __qtablewidgetitem107);
		QTableWidgetItem *__qtablewidgetitem108 = new QTableWidgetItem();
		__qtablewidgetitem108->setFont(font1);
		tableWidget_3->setItem(10, 2, __qtablewidgetitem108);
		QTableWidgetItem *__qtablewidgetitem109 = new QTableWidgetItem();
		__qtablewidgetitem109->setFont(font1);
		tableWidget_3->setItem(10, 3, __qtablewidgetitem109);
		QTableWidgetItem *__qtablewidgetitem110 = new QTableWidgetItem();
		__qtablewidgetitem110->setFont(font1);
		tableWidget_3->setItem(10, 4, __qtablewidgetitem110);
		QTableWidgetItem *__qtablewidgetitem111 = new QTableWidgetItem();
		__qtablewidgetitem111->setFont(font1);
		tableWidget_3->setItem(11, 0, __qtablewidgetitem111);
		QTableWidgetItem *__qtablewidgetitem112 = new QTableWidgetItem();
		__qtablewidgetitem112->setFont(font1);
		tableWidget_3->setItem(11, 1, __qtablewidgetitem112);
		QTableWidgetItem *__qtablewidgetitem113 = new QTableWidgetItem();
		__qtablewidgetitem113->setFont(font1);
		tableWidget_3->setItem(11, 2, __qtablewidgetitem113);
		QTableWidgetItem *__qtablewidgetitem114 = new QTableWidgetItem();
		__qtablewidgetitem114->setFont(font1);
		tableWidget_3->setItem(11, 3, __qtablewidgetitem114);
		QTableWidgetItem *__qtablewidgetitem115 = new QTableWidgetItem();
		__qtablewidgetitem115->setFont(font1);
		tableWidget_3->setItem(11, 4, __qtablewidgetitem115);
		tableWidget_3->setObjectName(QString::fromUtf8("tableWidget_3"));
		tableWidget_3->setFont(font1);
		tableWidget_3->horizontalHeader()->setStretchLastSection(true);
		tableWidget_3->verticalHeader()->setStretchLastSection(true);

		verticalLayout_6->addWidget(tableWidget_3);

		verticalLayout_6->setStretch(0, 1);
		verticalLayout_6->setStretch(1, 1);
		verticalLayout_6->setStretch(2, 50);

		gridLayout->addLayout(verticalLayout_6, 0, 2, 1, 1);

		gridLayout->setColumnStretch(0, 2);
		gridLayout->setColumnStretch(1, 3);
		gridLayout->setColumnStretch(2, 2);

		gridLayout_2->addLayout(gridLayout, 1, 0, 1, 1);

		lineEdit = new QLineEdit(layoutWidget);
		lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
		QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		sizePolicy.setHorizontalStretch(0);
		sizePolicy.setVerticalStretch(0);
		sizePolicy.setHeightForWidth(lineEdit->sizePolicy().hasHeightForWidth());
		lineEdit->setSizePolicy(sizePolicy);

		gridLayout_2->addWidget(lineEdit, 2, 0, 1, 1);

		gridLayout_2->setRowStretch(0, 1);
		gridLayout_2->setRowStretch(1, 50);
		gridLayout_2->setRowStretch(2, 7);

		verticalLayout_7->addLayout(gridLayout_2);

		verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

		verticalLayout_7->addItem(verticalSpacer_2);

		verticalLayout_7->setStretch(0, 1);
		verticalLayout_7->setStretch(1, 100);
		verticalLayout_7->setStretch(2, 1);

		horizontalLayout_3->addLayout(verticalLayout_7);

		horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

		horizontalLayout_3->addItem(horizontalSpacer_3);

		horizontalLayout_3->setStretch(0, 1);
		horizontalLayout_3->setStretch(1, 100);
		horizontalLayout_3->setStretch(2, 1);
		MainWindow->setCentralWidget(centralwidget);
		menubar = new QMenuBar(MainWindow);
		menubar->setObjectName(QString::fromUtf8("menubar"));
		menubar->setGeometry(QRect(0, 0, 920, 30));
		menubar->setFont(font);
		menu = new QMenu(menubar);
		menu->setObjectName(QString::fromUtf8("menu"));
		menu->setFont(font);
		menu_2 = new QMenu(menubar);
		menu_2->setObjectName(QString::fromUtf8("menu_2"));
		menu_3 = new QMenu(menubar);
		menu_3->setObjectName(QString::fromUtf8("menu_3"));
		menu_4 = new QMenu(menubar);
		menu_4->setObjectName(QString::fromUtf8("menu_4"));
		menu_5 = new QMenu(menubar);
		menu_5->setObjectName(QString::fromUtf8("menu_5"));
		MainWindow->setMenuBar(menubar);
		statusbar = new QStatusBar(MainWindow);
		statusbar->setObjectName(QString::fromUtf8("statusbar"));
		MainWindow->setStatusBar(statusbar);

		menubar->addAction(menu->menuAction());
		menubar->addAction(menu_2->menuAction());
		menubar->addAction(menu_3->menuAction());
		menubar->addAction(menu_4->menuAction());
		menubar->addAction(menu_5->menuAction());
		menu->addAction(action);
		menu_2->addAction(action_2);
		menu_3->addAction(action_3);
		menu_4->addAction(action_4);
		menu_5->addAction(action_5);
	}

	void retranslateUi(QMainWindow* MainWindow)
	{
		MainWindow->setWindowTitle(QCoreApplication::translate("高压电缆接头表面质量检测系统", "高压电缆接头表面质量检测系统", nullptr));
		action->setText(QCoreApplication::translate("MainWindow", "\345\257\274\345\205\245\347\202\271\344\272\221", nullptr));
		action_2->setText(QCoreApplication::translate("MainWindow", "\346\216\245\345\244\264\345\210\206\345\211\262\344\270\216\345\260\272\345\257\270\346\265\213\351\207\217", nullptr));
		action_3->setText(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\346\243\200\346\265\213\351\207\217\345\214\226", nullptr));
		action_4->setText(QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\346\270\205\351\231\244", nullptr));
		action_5->setText(QCoreApplication::translate("MainWindow", "\345\257\274\345\207\272\346\212\245\345\221\212", nullptr));
		label->setText(QCoreApplication::translate("MainWindow", "\347\224\265\347\274\206\347\274\226\345\217\267\357\274\232", nullptr));
		label_8->setText(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\346\230\276\347\244\272", nullptr));
		tabWidget_3->setTabText(tabWidget_3->indexOf(tab_9), QCoreApplication::translate("MainWindow", "\346\216\245\345\244\264\347\202\271\344\272\221", nullptr));
		tabWidget_3->setTabText(tabWidget_3->indexOf(tab_10), QCoreApplication::translate("MainWindow", "\346\216\245\345\244\264\347\202\271\344\272\221\345\210\206\345\211\262\347\273\223\346\236\234", nullptr));
		tabWidget_3->setTabText(tabWidget_3->indexOf(tab_11), QCoreApplication::translate("MainWindow", "\344\270\273\347\273\235\347\274\230\347\202\271\344\272\221", nullptr));
		label_9->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
		tabWidget_3->setTabText(tabWidget_3->indexOf(tab_12), QCoreApplication::translate("MainWindow", "\344\270\273\347\273\235\347\274\230\350\241\250\351\235\242\347\274\272\351\231\267\346\243\200\346\265\213\347\273\223\346\236\234", nullptr));
		label_2->setText(QCoreApplication::translate("MainWindow", "\345\217\202\346\225\260\346\265\213\351\207\217\347\273\223\346\236\234", nullptr));
		QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
		___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "\350\242\253\346\265\213\351\207\217\345\220\215\347\247\260", nullptr));
		QTableWidgetItem *___qtablewidgetitem1 = tableWidget->horizontalHeaderItem(1);
		___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "\346\265\213\351\207\217\345\200\274", nullptr));
		QTableWidgetItem *___qtablewidgetitem2 = tableWidget->verticalHeaderItem(0);
		___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "1", nullptr));
		QTableWidgetItem *___qtablewidgetitem3 = tableWidget->verticalHeaderItem(1);
		___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "2", nullptr));
		QTableWidgetItem *___qtablewidgetitem4 = tableWidget->verticalHeaderItem(2);
		___qtablewidgetitem4->setText(QCoreApplication::translate("MainWindow", "3", nullptr));
		QTableWidgetItem *___qtablewidgetitem5 = tableWidget->verticalHeaderItem(3);
		___qtablewidgetitem5->setText(QCoreApplication::translate("MainWindow", "4", nullptr));
		QTableWidgetItem *___qtablewidgetitem6 = tableWidget->verticalHeaderItem(4);
		___qtablewidgetitem6->setText(QCoreApplication::translate("MainWindow", "5", nullptr));
		QTableWidgetItem *___qtablewidgetitem7 = tableWidget->verticalHeaderItem(5);
		___qtablewidgetitem7->setText(QCoreApplication::translate("MainWindow", "6", nullptr));
		QTableWidgetItem *___qtablewidgetitem8 = tableWidget->verticalHeaderItem(6);
		___qtablewidgetitem8->setText(QCoreApplication::translate("MainWindow", "7", nullptr));
		QTableWidgetItem *___qtablewidgetitem9 = tableWidget->verticalHeaderItem(7);
		___qtablewidgetitem9->setText(QCoreApplication::translate("MainWindow", "8", nullptr));
		QTableWidgetItem *___qtablewidgetitem10 = tableWidget->verticalHeaderItem(8);
		___qtablewidgetitem10->setText(QCoreApplication::translate("MainWindow", "9", nullptr));
		QTableWidgetItem *___qtablewidgetitem11 = tableWidget->verticalHeaderItem(9);
		___qtablewidgetitem11->setText(QCoreApplication::translate("MainWindow", "10", nullptr));
		QTableWidgetItem *___qtablewidgetitem12 = tableWidget->verticalHeaderItem(10);
		___qtablewidgetitem12->setText(QCoreApplication::translate("MainWindow", "11", nullptr));
		QTableWidgetItem *___qtablewidgetitem13 = tableWidget->verticalHeaderItem(11);
		___qtablewidgetitem13->setText(QCoreApplication::translate("MainWindow", "12", nullptr));

		const bool __sortingEnabled = tableWidget->isSortingEnabled();
		tableWidget->setSortingEnabled(false);
		QTableWidgetItem *___qtablewidgetitem14 = tableWidget->item(0, 0);
		___qtablewidgetitem14->setText(QCoreApplication::translate("MainWindow", "\345\244\226\345\215\212\345\257\274\347\224\265\350\277\207\346\270\241\345\270\246\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem15 = tableWidget->item(1, 0);
		___qtablewidgetitem15->setText(QCoreApplication::translate("MainWindow", "XLPE\344\270\273\347\273\235\347\274\230\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem16 = tableWidget->item(2, 0);
		___qtablewidgetitem16->setText(QCoreApplication::translate("MainWindow", "\345\217\215\345\272\224\345\212\233\351\224\245\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem17 = tableWidget->item(3, 0);
		___qtablewidgetitem17->setText(QCoreApplication::translate("MainWindow", "\345\206\205\345\215\212\345\257\274\347\224\265\345\261\202\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem18 = tableWidget->item(4, 0);
		___qtablewidgetitem18->setText(QCoreApplication::translate("MainWindow", "\345\257\274\344\275\223\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem19 = tableWidget->item(5, 0);
		___qtablewidgetitem19->setText(QCoreApplication::translate("MainWindow", "\346\216\245\345\244\264\345\274\200\347\272\277\350\275\264\345\220\221\351\253\230\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem20 = tableWidget->item(6, 0);
		___qtablewidgetitem20->setText(QCoreApplication::translate("MainWindow", "\345\244\226\345\215\212\345\257\274\347\224\265\345\261\202\345\214\272\345\237\237\345\244\226\345\276\204", nullptr));
		QTableWidgetItem *___qtablewidgetitem21 = tableWidget->item(7, 0);
		___qtablewidgetitem21->setText(QCoreApplication::translate("MainWindow", "\344\272\244\350\201\224\344\270\273\347\273\235\347\274\230\345\214\272\345\237\237\345\244\226\345\276\204", nullptr));
		QTableWidgetItem *___qtablewidgetitem22 = tableWidget->item(8, 0);
		___qtablewidgetitem22->setText(QCoreApplication::translate("MainWindow", "\345\206\205\345\215\212\345\257\274\347\224\265\345\261\202\345\214\272\345\237\237\345\244\226\345\276\204", nullptr));
		QTableWidgetItem *___qtablewidgetitem23 = tableWidget->item(9, 0);
		___qtablewidgetitem23->setText(QCoreApplication::translate("MainWindow", "\345\257\274\344\275\223\345\214\272\345\237\237\345\244\226\345\276\204", nullptr));
		QTableWidgetItem *___qtablewidgetitem24 = tableWidget->item(10, 0);
		___qtablewidgetitem24->setText(QCoreApplication::translate("MainWindow", "\345\244\226\345\215\212\345\257\274\347\224\265\350\277\207\346\270\241\345\270\246\345\200\276\346\226\234\350\247\222\345\272\246", nullptr));
		QTableWidgetItem *___qtablewidgetitem25 = tableWidget->item(11, 0);
		___qtablewidgetitem25->setText(QCoreApplication::translate("MainWindow", "\345\217\215\345\272\224\345\212\233\351\224\245\345\200\276\346\226\234\350\247\222\345\272\246", nullptr));
		tableWidget->setSortingEnabled(__sortingEnabled);

		label_10->setText(QCoreApplication::translate("MainWindow", "\344\270\273\347\273\235\347\274\230\350\241\250\351\235\242\347\274\272\351\231\267\351\207\217\345\214\226\347\273\223\346\236\234", nullptr));
		QTableWidgetItem *___qtablewidgetitem26 = tableWidget_3->horizontalHeaderItem(0);
		___qtablewidgetitem26->setText(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\350\241\250\351\235\242\347\247\257", nullptr));
		QTableWidgetItem *___qtablewidgetitem27 = tableWidget_3->horizontalHeaderItem(1);
		___qtablewidgetitem27->setText(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\345\234\206\346\237\261\351\235\242\347\247\257", nullptr));
		QTableWidgetItem *___qtablewidgetitem28 = tableWidget_3->horizontalHeaderItem(2);
		___qtablewidgetitem28->setText(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\344\275\223\347\247\257", nullptr));
		QTableWidgetItem *___qtablewidgetitem29 = tableWidget_3->horizontalHeaderItem(3);
		___qtablewidgetitem29->setText(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\345\210\260\351\241\266\347\253\257\350\267\235\347\246\273", nullptr));
		QTableWidgetItem *___qtablewidgetitem30 = tableWidget_3->horizontalHeaderItem(4);
		___qtablewidgetitem30->setText(QCoreApplication::translate("MainWindow", "\345\207\271(\345\207\270)\345\200\274", nullptr));
		QTableWidgetItem *___qtablewidgetitem31 = tableWidget_3->verticalHeaderItem(0);
		___qtablewidgetitem31->setText(QCoreApplication::translate("MainWindow", "1", nullptr));
		QTableWidgetItem *___qtablewidgetitem32 = tableWidget_3->verticalHeaderItem(1);
		___qtablewidgetitem32->setText(QCoreApplication::translate("MainWindow", "2", nullptr));
		QTableWidgetItem *___qtablewidgetitem33 = tableWidget_3->verticalHeaderItem(2);
		___qtablewidgetitem33->setText(QCoreApplication::translate("MainWindow", "3", nullptr));
		QTableWidgetItem *___qtablewidgetitem34 = tableWidget_3->verticalHeaderItem(3);
		___qtablewidgetitem34->setText(QCoreApplication::translate("MainWindow", "4", nullptr));
		QTableWidgetItem *___qtablewidgetitem35 = tableWidget_3->verticalHeaderItem(4);
		___qtablewidgetitem35->setText(QCoreApplication::translate("MainWindow", "5", nullptr));
		QTableWidgetItem *___qtablewidgetitem36 = tableWidget_3->verticalHeaderItem(5);
		___qtablewidgetitem36->setText(QCoreApplication::translate("MainWindow", "6", nullptr));
		QTableWidgetItem *___qtablewidgetitem37 = tableWidget_3->verticalHeaderItem(6);
		___qtablewidgetitem37->setText(QCoreApplication::translate("MainWindow", "7", nullptr));
		QTableWidgetItem *___qtablewidgetitem38 = tableWidget_3->verticalHeaderItem(7);
		___qtablewidgetitem38->setText(QCoreApplication::translate("MainWindow", "8", nullptr));
		QTableWidgetItem *___qtablewidgetitem39 = tableWidget_3->verticalHeaderItem(8);
		___qtablewidgetitem39->setText(QCoreApplication::translate("MainWindow", "9", nullptr));
		QTableWidgetItem *___qtablewidgetitem40 = tableWidget_3->verticalHeaderItem(9);
		___qtablewidgetitem40->setText(QCoreApplication::translate("MainWindow", "10", nullptr));
		QTableWidgetItem *___qtablewidgetitem41 = tableWidget_3->verticalHeaderItem(10);
		___qtablewidgetitem41->setText(QCoreApplication::translate("MainWindow", "11", nullptr));
		QTableWidgetItem *___qtablewidgetitem42 = tableWidget_3->verticalHeaderItem(11);
		___qtablewidgetitem42->setText(QCoreApplication::translate("MainWindow", "12", nullptr));

		const bool __sortingEnabled1 = tableWidget_3->isSortingEnabled();
		tableWidget_3->setSortingEnabled(false);
		tableWidget_3->setSortingEnabled(__sortingEnabled1);

		menu->setTitle(QCoreApplication::translate("MainWindow", "\345\257\274\345\205\245\347\202\271\344\272\221", nullptr));
		menu_2->setTitle(QCoreApplication::translate("MainWindow", "\346\216\245\345\244\264\345\210\206\345\211\262\344\270\216\345\260\272\345\257\270\346\265\213\351\207\217", nullptr));
		menu_3->setTitle(QCoreApplication::translate("MainWindow", "\347\274\272\351\231\267\346\243\200\346\265\213\351\207\217\345\214\226", nullptr));
		menu_4->setTitle(QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\346\270\205\351\231\244", nullptr));
		menu_5->setTitle(QCoreApplication::translate("MainWindow", "\345\257\274\345\207\272\346\212\245\345\221\212", nullptr));
	} // retranslateUi
};

#endif