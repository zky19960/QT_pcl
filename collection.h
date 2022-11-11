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

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;//���ĸ�vtk����
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;//ԭʼ����
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Seg_cloud;//ԭʼ���Ʒָ���
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr zhujueyuan;//����Ե����

		bool cableflag = false;
		bool canshuflag = false;
		bool detectflag = false;

		std::vector<double> region_start;  //��������뵼����ɴ�����һ��Ǧ��ͷ���ڶ���Ǧ��ͷ���ڰ뵼�硢��о�����
		std::vector<double> region_end;		////��������뵼����ɴ�����һ��Ǧ��ͷ���ڶ���Ǧ��ͷ���ڰ뵼�硢��о���յ�

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