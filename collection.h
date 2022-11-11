#pragma once

#ifndef _COLLECTION_H_
#define _COLLECTION_H_

#include "ui_collection.h"
#include "cable.h"

#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QAction>
#include <QDebug>
#include <QDesktopServices>
#include <QFileInfo>
#include <ActiveQt/QAxObject>
#include <QDateTime>
#include <QBuffer>

class Collection : public QMainWindow
{
	Q_OBJECT

	public:
		Collection(QWidget* parent = Q_NULLPTR);
		~Collection();

	private:
		UI ui;
		QString str;

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;//开四个vtk窗口
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;//原始点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Seg_cloud;//原始点云分割结果
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr zhujueyuan;//主绝缘点云

		bool cableflag = false;
		bool canshuflag = false;
		bool detectflag = false;

		std::vector<double> region_start;  //用来存外半导电过渡带、第一个铅笔头、第二个铅笔头、内半导电、线芯的起点
		std::vector<double> region_end;		////用来存外半导电过渡带、第一个铅笔头、第二个铅笔头、内半导电、线芯的终点

	private slots:
		void LoadCloudPoint();
		void clear();
		void SegDetect();
		void BadDetect();
		void FileOutput();

	private:
		Cable *cable = nullptr;

	private:
		void Soft_Init();
};


#endif