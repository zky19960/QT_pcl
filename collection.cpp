#include "collection.h"

#include <vtkOutputWindow.h>
#include <vtkRenderWindow.h>

Collection::Collection(QWidget *parent) : QMainWindow(parent)
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);//不弹出vtkOutputWindow窗口

	ui.InitUI(this);//初始化界面
	Soft_Init();//

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr zhujueyuan_detect;

	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	Seg_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	zhujueyuan.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	zhujueyuan_detect.reset(new pcl::PointCloud<pcl::PointXYZHSV>);

	viewer1.reset(new pcl::visualization::PCLVisualizer("viewer1", false));
	viewer2.reset(new pcl::visualization::PCLVisualizer("viewer2", false));
	viewer3.reset(new pcl::visualization::PCLVisualizer("viewer3", false));
	viewer4.reset(new pcl::visualization::PCLVisualizer("viewer4", false));

	ui.qvtkWidget2_10->SetRenderWindow(viewer1->getRenderWindow());
	viewer1->setupInteractor(ui.qvtkWidget2_10->GetInteractor(), ui.qvtkWidget2_10->GetRenderWindow());
	ui.qvtkWidget2_11->SetRenderWindow(viewer2->getRenderWindow());
	viewer2->setupInteractor(ui.qvtkWidget2_11->GetInteractor(), ui.qvtkWidget2_11->GetRenderWindow());
	ui.qvtkWidget2_12->SetRenderWindow(viewer3->getRenderWindow());
	viewer3->setupInteractor(ui.qvtkWidget2_12->GetInteractor(), ui.qvtkWidget2_12->GetRenderWindow());
	ui.qvtkWidget2_13->SetRenderWindow(viewer4->getRenderWindow());
	viewer4->setupInteractor(ui.qvtkWidget2_13->GetInteractor(), ui.qvtkWidget2_13->GetRenderWindow());

	viewer1->addPointCloud(cloud, "cloud");
	viewer2->addPointCloud(Seg_cloud, "Seg_cloud");
	viewer3->addPointCloud(zhujueyuan, "zhujueyuan");

	pcl::visualization::PointCloudColorHandlerHSVField<pcl::PointXYZHSV> hsv(zhujueyuan_detect);
	viewer4->addPointCloud(zhujueyuan_detect,hsv,"zhujueyuan_detect");

	viewer1->resetCamera();
	viewer2->resetCamera();
	viewer3->resetCamera();
	viewer4->resetCamera();

	viewer1->setBackgroundColor(0, 0, 0);
	viewer2->setBackgroundColor(0, 0, 0);
	viewer3->setBackgroundColor(0, 0, 0);
	viewer4->setBackgroundColor(0, 0, 0);

	ui.qvtkWidget2_10->update();
	ui.qvtkWidget2_11->update();
	ui.qvtkWidget2_12->update();
	ui.qvtkWidget2_13->update();

	QPixmap *pixmap = new QPixmap("22.png");
	pixmap->scaled(ui.label_9->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
	ui.label_9->setScaledContents(true);
	ui.label_9->setPixmap(*pixmap);
}

Collection::~Collection()
{
	ui.qvtkWidget2_10->deleteLater();
	ui.qvtkWidget2_11->deleteLater();
	ui.qvtkWidget2_12->deleteLater();
	ui.qvtkWidget2_13-> deleteLater();
}

void Collection::Soft_Init()//槽函数链接
{
	connect(ui.action, &QAction::triggered, this, &Collection::LoadCloudPoint);
	connect(ui.action_4, &QAction::triggered, this, &Collection::clear);
	connect(ui.action_2, &QAction::triggered, this, &Collection::SegDetect);
	connect(ui.action_3, &QAction::triggered, this, &Collection::BadDetect);
	connect(ui.action_5, &QAction::triggered, this, &Collection::FileOutput);
}

void Collection::LoadCloudPoint()
{
	QString stringcp = QFileDialog::getOpenFileName(this, tr("CloudPoint File"), "", tr("file (*.ply)"));
	QFileInfo FileInfo(stringcp);
	str = FileInfo.fileName();
	str.chop(4);

	if (!stringcp.isEmpty() && pcl::io::loadPLYFile(std::string(stringcp.toLocal8Bit()), *cloud) == -1)
	{
		QMessageBox::critical(this, tr("警告"), tr("未能打开点云文件！"));
	}

	else if (stringcp.isEmpty()) {}

	else
	{
		if (cloud->empty())
		{
			QMessageBox::critical(this, tr("警告"), tr("点云文件为空！"));
		}

		else
		{
			if(cableflag == false)
			{
				for (size_t i = 0; i < cloud->points.size(); ++i)
				{
					cloud->points[i].r = 255;
					cloud->points[i].g = 255;
					cloud->points[i].b = 255;
				}

				ui.lineEdit->setText("加载点云完成！");
				cableflag = true;
				cable = new Cable;
				ui.comboBox->addItem(str);
				viewer1->updatePointCloud(cloud, "cloud");
			}

			else
			{
				QMessageBox::critical(this, tr("警告"), tr("重复加载点云文件！"));
			}
		}
	}
}

void Collection::clear()
{
	if (cableflag = true)
	{
		cableflag = false;
		canshuflag = false;
		detectflag = false;

		for (int i = 0; i < 12; i++)
		{
			ui.tableWidget->setItem(i, 1, new QTableWidgetItem(""));
		}

		for (int i = 0; i < 12; i++)
		{
			ui.tableWidget_3->setItem(i, 0, new QTableWidgetItem(""));
			ui.tableWidget_3->setItem(i, 1, new QTableWidgetItem(""));
			ui.tableWidget_3->setItem(i, 2, new QTableWidgetItem(""));
			ui.tableWidget_3->setItem(i, 3, new QTableWidgetItem(""));
			ui.tableWidget_3->setItem(i, 4, new QTableWidgetItem(""));
		}

		cloud->clear();
		Seg_cloud->clear();
		zhujueyuan->clear();

		viewer1->updatePointCloud(cloud, "cloud");
		viewer2->updatePointCloud(Seg_cloud, "Seg_cloud");
		viewer3->updatePointCloud(zhujueyuan, "zhujueyuan");
		
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr zhujueyuan_detect(new pcl::PointCloud<pcl::PointXYZHSV>);
		pcl::visualization::PointCloudColorHandlerHSVField<pcl::PointXYZHSV> hsv(zhujueyuan_detect);
		viewer4->updatePointCloud(zhujueyuan_detect, hsv, "zhujueyuan_detect");

		ui.qvtkWidget2_10->update();
		ui.qvtkWidget2_11->update();
		ui.qvtkWidget2_12->update();
		ui.qvtkWidget2_13->update();
		
		ui.lineEdit->clear();
		ui.comboBox->clear();

		region_start.clear(); region_end.clear();
		delete cable;
	}

	else
	{
		ui.lineEdit->setText("没有点云文件加载！");
	}
}

void Collection::SegDetect() //参数测量
{
	if (cableflag = true)
	{
		//ui.lineEdit->setText("开始参数测量！");

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr __Seg__ = cloud->makeShared();
		cable->getcp(__Seg__);
		cable->filtering(__Seg__);

		double each_circle_distance = 0.5;
		cable->z_value_circle_fitting(__Seg__, each_circle_distance);  //对点云按z值进行圆拟合

		cable->rough_segmentation(__Seg__, each_circle_distance, region_start, region_end);

		Seg_cloud = __Seg__;
		viewer2->updatePointCloud(Seg_cloud, "Seg_cloud");

		ui.tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(region_end.at(0) - region_start.at(0)) + "mm"));//外半导电过渡带轴向高度：
		ui.tableWidget->item(0, 1)->setForeground(Qt::black);
		ui.tableWidget->item(0, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(1, 1, new QTableWidgetItem(QString::number(region_start.at(1) - region_end.at(0)) + "mm"));//XLPE主绝缘轴向高度：
		ui.tableWidget->item(1, 1)->setForeground(Qt::black);
		ui.tableWidget->item(1, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(2, 1, new QTableWidgetItem(QString::number(region_end.at(1) - region_start.at(1)) + "mm"));//反应力锥轴向高度
		ui.tableWidget->item(2, 1)->setForeground(Qt::black);
		ui.tableWidget->item(2, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(3, 1, new QTableWidgetItem(QString::number(region_start.at(3) - region_start.at(2)) + "mm"));//内半导电层轴向高度：
		ui.tableWidget->item(3, 1)->setForeground(Qt::black);
		ui.tableWidget->item(3, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(4, 1, new QTableWidgetItem(QString::number(region_end.at(3) - region_start.at(3)) + "mm"));//导体轴向长度：
		ui.tableWidget->item(4, 1)->setForeground(Qt::black);
		ui.tableWidget->item(4, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(5, 1, new QTableWidgetItem(QString::number(cable->getpara().cable_z_max_value - region_end.at(0)) + "mm"));//接头开线轴向高度：：
		ui.tableWidget->item(5, 1)->setForeground(Qt::black);
		ui.tableWidget->item(5, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(6, 1, new QTableWidgetItem(QString::number(2 * cable->getpara().outer_semiconducting_circle_radius.at(static_cast<int>(cable->getpara().outer_semiconducting_circle_radius.size()/2))) + "mm"));//外半导电区域外径：
		ui.tableWidget->item(6, 1)->setForeground(Qt::black);
		ui.tableWidget->item(6, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(7, 1, new QTableWidgetItem(QString::number(2 * cable->getpara().XLPE_insulation_circle_radius.at(static_cast<int>(cable->getpara().XLPE_insulation_circle_radius.size() / 2))) + "mm"));//交联主绝缘区域外径：
		ui.tableWidget->item(7, 1)->setForeground(Qt::black);
		ui.tableWidget->item(7, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(8, 1, new QTableWidgetItem(QString::number(2 * cable->getpara().inner_semiconducting_circle_radius.at(static_cast<int>(cable->getpara().inner_semiconducting_circle_radius.size() / 2))) + "mm"));//内半导电层区域外径：：
		ui.tableWidget->item(8, 1)->setForeground(Qt::black);
		ui.tableWidget->item(8, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(9, 1, new QTableWidgetItem(QString::number(2 * cable->getpara().conduct_circle_radius.at(static_cast<int>(cable->getpara().conduct_circle_radius.size() / 2))) + "mm"));//导体区域外径：：
		ui.tableWidget->item(9, 1)->setForeground(Qt::black);
		ui.tableWidget->item(9, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(10, 1, new QTableWidgetItem(QString::number( //外半导电过渡带倾斜角度：：：
			std::atan2(( cable->getpara().outer_semiconducting_circle_radius.at(static_cast<int>(cable->getpara().outer_semiconducting_circle_radius.size()/2)) -
			cable->getpara().XLPE_insulation_circle_radius.at(static_cast<int>(cable->getpara().XLPE_insulation_circle_radius.size()/2))) , region_end.at(0) - region_start.at(0))* 180  / M_PI 
		    ) + "°"));
		ui.tableWidget->item(10, 1)->setForeground(Qt::black);
		ui.tableWidget->item(10, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.tableWidget->setItem(11, 1, new QTableWidgetItem(QString::number( //反应力锥倾斜角度：
			atan2(cable->getpara().XLPE_insulation_circle_radius.at(static_cast<int>(cable->getpara().XLPE_insulation_circle_radius.size() / 2)) -
			cable->getpara().inner_semiconducting_circle_radius.at(static_cast<int>(cable->getpara().inner_semiconducting_circle_radius.size() / 2)),
			region_end.at(1) - region_start.at(1)) * 180 / M_PI)
			+ "°"));	
		ui.tableWidget->item(11, 1)->setForeground(Qt::black);
		ui.tableWidget->item(11, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

		ui.lineEdit->setText("参数测量完成！");
		canshuflag = true;
	}

	else
	{
		ui.lineEdit->setText("没有点云文件加载！");
	}
}

void Collection::BadDetect() //缺陷检测
{
	if (cableflag = true)
	{
		if (canshuflag = true)
		{
			detectflag = true;
			float each_strip_angle = 1;
			double slice_height = 0.5;

			for (int n = 0; n < cloud->size(); n++)
			{
				if (cloud->at(n).z > region_end.at(0) && cloud->at(n).z < region_start.at(1))
				{
					zhujueyuan->push_back(cloud->at(n));
				}
			}

			viewer3->updatePointCloud(zhujueyuan, "zhujueyuan");

			double conductEnd_2_outEnd_distance = region_end.back() - region_end.front();
			std::vector<defection_quantification> Defection_quantification;
			std::vector<defection_quantification> Defection_quantification_concave;

			cable->concave_convex_defect_detect(zhujueyuan, each_strip_angle, slice_height, conductEnd_2_outEnd_distance, 
				Defection_quantification, Defection_quantification_concave);

			int hang = 0;

			for (int i = 0; i < Defection_quantification.size(); i++)
			{
				ui.tableWidget_3->setItem(i, 0, new QTableWidgetItem(QString::number(
				Defection_quantification.at(i).defection_area) + "mm^2"));//表面积：
				ui.tableWidget_3->item(i, 0)->setForeground(Qt::black);
				ui.tableWidget_3->item(i, 0)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i, 1, new QTableWidgetItem(QString::number(
					Defection_quantification.at(i).defection_area_defection) + "mm^2"));//圆柱表面积：
				ui.tableWidget_3->item(i, 1)->setForeground(Qt::black);
				ui.tableWidget_3->item(i, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i, 2, new QTableWidgetItem(QString::number(
				Defection_quantification.at(i).defection_volume) + "mm^3"));//缺陷体积：
				//ui.tableWidget->item(i, 2)->setForeground(Qt::black);
				//ui.tableWidget->item(i, 2)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i, 4, new QTableWidgetItem(QString::number(
				Defection_quantification.at(i).defection_deepest_point) + "mm"));//凹凸值：
				//ui.tableWidget->item(i, 3)->setForeground(Qt::black);
				//ui.tableWidget->item(i, 3)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
				
				ui.tableWidget_3->setItem(i, 3, new QTableWidgetItem(QString::number(
				Defection_quantification.at(i).conductEnd_2_outEnd_distance1) + "mm"));//凹凸值：
				
				hang++;
			}

			for (int i = 0; i < Defection_quantification_concave.size(); i++)
			{
				ui.tableWidget_3->setItem(i + hang, 0, new QTableWidgetItem(QString::number(
					Defection_quantification_concave.at(i).defection_area) + "mm^2"));//表面积：
				ui.tableWidget_3->item(i + hang, 0)->setForeground(Qt::black);
				ui.tableWidget_3->item(i + hang, 0)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i + hang, 1, new QTableWidgetItem(QString::number(
				Defection_quantification_concave.at(i).defection_area_defection) + "mm^2"));//圆柱表面积：
				ui.tableWidget_3->item(i + hang, 1)->setForeground(Qt::black);
				ui.tableWidget_3->item(i + hang, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i + hang, 2, new QTableWidgetItem(QString::number(
				Defection_quantification_concave.at(i).defection_volume) + "mm^3"));//缺陷体积：
				//ui.tableWidget->item(i + hang, 2)->setForeground(Qt::black);
				//ui.tableWidget->item(i + hang, 2)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				ui.tableWidget_3->setItem(i + hang, 4, new QTableWidgetItem(QString::number(
				Defection_quantification_concave.at(i).defection_deepest_point) + "mm"));//凹凸值：
				//ui.tableWidget->item(i + hang, 3)->setForeground(Qt::black);
				//ui.tableWidget->item(i + hang, 3)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
				ui.tableWidget_3->setItem(i + hang, 3, new QTableWidgetItem(QString::number(
				Defection_quantification_concave.at(i).conductEnd_2_outEnd_distance1) + "mm"));//凹凸值：
			}

			//zhujueyuan_detect = cable->__colour_point_cloud_hsv_all__;
			pcl::PointCloud<pcl::PointXYZHSV>::Ptr zhujueyuan_detect = cable->__colour_point_cloud_hsv_all__;
			pcl::visualization::PointCloudColorHandlerHSVField<pcl::PointXYZHSV> hsv(zhujueyuan_detect);
			viewer4->updatePointCloud(zhujueyuan_detect, hsv, "zhujueyuan_detect");

			ui.lineEdit->setText("缺陷检测完成！");
		}

		else
		{
			ui.lineEdit->setText("没有执行过参数测量！");
		}
	}

	else
	{
		ui.lineEdit->setText("没有点云文件加载！");
	}
}

void Collection::FileOutput()
{
	QString stringfile = QFileDialog::getExistingDirectory(this, "请选择模板保存路径...", "./");
	stringfile = stringfile + "/" + str + ".csv";

	if (canshuflag == true && detectflag == true)
	{
		if (stringfile.isEmpty())
		{
			return;
		}

		else
		{
			QFile file(stringfile);
			bool flag = file.open(QIODevice::Truncate | QIODevice::WriteOnly);

			if (flag)
			{
				QTextStream stream(&file);
				QString conTents;

				QHeaderView *header = ui.tableWidget->horizontalHeader();
				if (header)
				{
					for (int i = 0; i < header->count(); i++)
					{
						QTableWidgetItem *item = ui.tableWidget->horizontalHeaderItem(i);

						if (!item)
						{
							continue;
						}

						conTents += item->text() + ",";
					}
					conTents += "\n";
				}

				for (int i = 0; i < ui.tableWidget->rowCount(); i++)
				{
					for (int j = 0; j < ui.tableWidget->columnCount(); j++)
					{

						QTableWidgetItem* item = ui.tableWidget->item(i, j);

						if (!item)
						{
							continue;
						}

						QString str = item->text();
						str.replace(",", " ");
						conTents += str + ",";
					}
					conTents += "\n";
				}

				QHeaderView *header2 = ui.tableWidget_3->horizontalHeader();
				if (header2)
				{
					for (int i = 0; i < header2->count(); i++)
					{
						QTableWidgetItem *item = ui.tableWidget_3->horizontalHeaderItem(i);

						if (!item)
						{
							continue;
						}

						conTents += item->text() + ",";
					}
					conTents += "\n";
				}

				for (int i = 0; i < ui.tableWidget_3->rowCount(); i++)
				{
					for (int j = 0; j < ui.tableWidget_3->columnCount(); j++)
					{

						QTableWidgetItem* item = ui.tableWidget_3->item(i, j);

						if (!item)
						{
							continue;
						}

						QString str = item->text();
						str.replace(",", " ");
						conTents += str + ",";
					}
					conTents += "\n";
				}

				stream << conTents;
				file.close();
				ui.lineEdit->setText("报告导出完成！");
			}

			else
			{
				return;
			}
		}
	}

	else
	{
		if (cableflag == true)
		{
			if (canshuflag == true)
			{
				if (detectflag == false)
				{
					ui.lineEdit->setText("没有执行过缺陷检测！");
				}
			}

			else
			{
				ui.lineEdit->setText("没有执行过参数测量！");
			}
		}

		else
		{
			ui.lineEdit->setText("没有加载点云！");
		}

		return;
	}
}