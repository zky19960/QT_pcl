#include "Cable.h"

Cable::Cable()
{
}

Cable::~Cable()
{
}

/*构造函数*/
Cable::Cable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb)  //构造函数定义
{
	m_pointcloud.clear();
	m_pointcloud = *pointcloud_rgb;
	m_pointcloud_rgb = m_pointcloud.makeShared();   //对原始指针进行深拷贝
}
//bool Cable::function_loop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)
//{
//	for (double slice_height = 0.5; slice_height <= 3; slice_height += 0.5)
//	{
//		for (float each_strip_angle = 0.5; each_strip_angle <= 3; each_strip_angle += 0.5)
//		{
//			concave_convex_defect_detect(pointcloud_filtered, each_strip_angle, slice_height);
//		}
//	}
//	return true;
//}
/*缺陷检测*/
bool Cable::concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, float each_strip_angle, double slice_height, double conductEnd_2_outEnd_distance,
	std::vector<defection_quantification> &Defection_quantification, std::vector<defection_quantification> &Defection_quantification_concave)
{
	/*根据平面方程生成离散点*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pointcloud_plane->clear();
	//for (float x = 0; x < 200;x+=0.1)
	//{
	//	for (float y = 0; y < 200; y += 0.1)
	//	{
	//		for (float z = 0; z < 200; z += 0.1)
	//		{
	//			pcl::PointXYZRGB point_tmp;
	//			if (x+2*y+3*z-4==0)
	//			{
	//				point_tmp.x = x;
	//				point_tmp.y = y;
	//				point_tmp.z = z;
	//				point_tmp.r = 255;
	//				point_tmp.g = 0;
	//				point_tmp.b = 0;
	//				pointcloud_plane->push_back(point_tmp);
	//			}
	//		}
	//	}
	//}
	//showpointcloud_xyzrgb(pointcloud_plane);
	//pcl::io::savePLYFileBinary("plane.ply", *pointcloud_plane);
	/*读取根据平面方程生成离散点的点云*/
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_plane_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pointcloud_plane_xyz->clear();
	//loadPointcloud("plane.ply", pointcloud_plane_xyz);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	//xyz2rgb(pointcloud_plane_xyz, pointcloud_plane);			 //xyz转xyzrgb

	/*人为在平面离散点中添加缺陷*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_plane_add_noise(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pointcloud_plane_add_noise->clear();
	//pointcloud_plane_add_noise = pointcloud_plane->makeShared();
	//default_random_engine e(time(0));
	//uniform_real_distribution<double> u(-1.2, 3.5);
	//for (int i = 0;i<10;i++)
	//{
	//	pcl::PointXYZRGB point_tmp1;
	//	point_tmp1.x = u(e);
	//	point_tmp1.y = u(e);
	//	point_tmp1.z = u(e);
	//	pointcloud_plane_add_noise->push_back(point_tmp1);
	//}
	/*验证各种平面拟合方法的效果*/
	//float RA, RB, RC, RD; //随机采样一致性拟合
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Inner_point (new pcl::PointCloud<pcl::PointXYZRGB>);
	//ransac_plane_estimation(pointcloud_plane_add_noise, RA, RB, RC, RD, Inner_point);  //RANSAC平面拟合
	//float DA, DB, DC, DD; //最小二乘拟合
	//least_square_plane_estimation(pointcloud_plane_add_noise, DA, DB, DC, DD);
	//float ADA, ADB, ADC, ADD; //随机采样一致性结合最小二乘
	//least_square_plane_estimation(Inner_point, ADA, ADB, ADC, ADD);
	cout << endl;
	cout << "正在进行主绝缘表面凹陷凸起缺陷检测与量化任务!" << endl;
	//求电缆的最高点，只求一个点不准确，所以求最后m个点并求平均，不用最后一个拟合圆的圆心原因是因为存在扫描误差
	int cable_max_point_number = 20;
	double sum_cable_max_point = 0;
	for (int i = 1; i <= cable_max_point_number; i++)
	{
		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
	}
	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //线芯的最高点，即线芯终点

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cylinder_pointcloud = pointcloud_filtered->makeShared();
	//将所有点转为白色
	for (int i = 0; i<cylinder_pointcloud->size(); i++)
	{
		cylinder_pointcloud->at(i).r = 255;
		cylinder_pointcloud->at(i).g = 255;
		cylinder_pointcloud->at(i).b = 255;
	}
	//showpointcloud_xyzrgb(cylinder_pointcloud);

	/*添加噪声*/
	//for (int i = 0; i < pointcloud_filtered->size(); i++)
	//{
	//	if ((cable_z_max_value - pointcloud_filtered->at(i).z > 100) && (cable_z_max_value - pointcloud_filtered->at(i).z < 155))
	//	{
	//		pointcloud_filtered->at(i).r = 255;
	//		pointcloud_filtered->at(i).g = 255;
	//		pointcloud_filtered->at(i).b = 255;
	//		cylinder_pointcloud->push_back(pointcloud_filtered->at(i));
	//	}
	//}
	//showpointcloud_xyzrgb(cylinder_pointcloud);
	/*圆柱拟合，根据各点距离轴线的距离和圆柱半径差值来找出凹凸点*/
	Cylinder cylinder;  //圆柱拟合
	ransac_cylindrical_estimation(cylinder_pointcloud, cylinder);
	//for (int i = 0; i < cylinder_pointcloud->size(); i++)
	//{
	//	Eigen::Vector4f point;
	//	point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //圆柱轴线上一点
	//	Eigen::Vector4f normal;
	//	normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //圆柱轴线方向向量
	//	Eigen::Vector4f point_cloud_point;
	//	point_cloud_point << cylinder_pointcloud->at(i).x, cylinder_pointcloud->at(i).y, cylinder_pointcloud->at(i).z;
	//	double point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// 空间点到空间直线距离，参数分别为点云中一点，直线上一点，直线的法向量
	//	if (abs(cylinder.radius - point_2_line_distance) > 0.45 )   //如果点云中一点距离轴线距离与圆柱轴线半径差大于一定值
	//	{
	//		cylinder_pointcloud->at(i).r = 0;
	//		cylinder_pointcloud->at(i).g = 255;
	//		cylinder_pointcloud->at(i).b = 0;
	//		//cylinder_pointcloud->push_back(pointcloud_filtered->at(i));
	//	}
	//}
	//showpointcloud_xyzrgb(cylinder_pointcloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_colour(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<Index_Strip> XLPE_strip_vector;
	//float each_strip_angle = 2;  //抽条角度
	regionSegmentation(cylinder_pointcloud, XLPE_strip_vector, each_strip_angle); //局部点云抽条
																				  //showpointcloud_xyzrgb(XLPE_strip_vector.at(0).each_strip);  //显示一个条状点云
																				  /*输出一条数据*/
																				  //fstream f22;
																				  //f22.open("point2.txt", ios::out | ios::app);
																				  //for (int i = 0;i < XLPE_strip_vector.at(0).each_strip->size();i++)
																				  //{
																				  //	//f22 << XLPE_strip_vector.at(0).each_strip->at(i).x 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).y 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).z << endl;
																				  //	f22 << XLPE_strip_vector.at(0).each_strip->at(i).x << ",";
																				  //}
																				  //f22 << endl;
																				  //for (int i = 0; i < XLPE_strip_vector.at(0).each_strip->size(); i++)
																				  //{
																				  //	//f22 << XLPE_strip_vector.at(0).each_strip->at(i).x 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).y 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).z << endl;
																				  //	f22 << XLPE_strip_vector.at(0).each_strip->at(i).y << ",";
																				  //}
																				  //f22 << endl;
																				  //for (int i = 0; i < XLPE_strip_vector.at(0).each_strip->size(); i++)
																				  //{
																				  //	//f22 << XLPE_strip_vector.at(0).each_strip->at(i).x 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).y 
																				  //	//	<< "," << XLPE_strip_vector.at(0).each_strip->at(i).z << endl;
																				  //	f22 << XLPE_strip_vector.at(0).each_strip->at(i).z << ",";
																				  //}
																				  /*展示抽条算法的有效性*/
	for (int i = 0; i<XLPE_strip_vector.size(); i++)
	{
		for (int j = 0; j<XLPE_strip_vector.at(i).each_strip->size(); j++)
		{
			point_cloud_colour->push_back(XLPE_strip_vector.at(i).each_strip->at(j));
		}
	}
	//showpointcloud_xyzrgb(point_cloud_colour);//展示抽条算法的有效性
	std::vector<std::vector<Strip_slice>> slice_vector;  //存储局部片状点云的容器

	Strip_2_slice(cylinder_pointcloud, slice_vector, XLPE_strip_vector, slice_height);  //局部点云切片
	std::vector<std::vector<Strip_slice>> slice_vector1;
	Strip_2_slice(cylinder_pointcloud, slice_vector1, XLPE_strip_vector, slice_height);  //再切一次的目的是为了构建两个切片容器，后续颜色映射才不容易出错
																						 //slice_vector与slice_vector1/slice_vector2内容一模一样，只是地址不一样
																						 //showpointcloud_xyzrgb(slice_vector.at(0).at(0).pointcloud_strip_slice);
																						 /*展示切片算法的有效性*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_local_slice_colour(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < XLPE_strip_vector.size(); i++)
	{
		for (int j = 0; j < XLPE_strip_vector.at(i).each_strip->size(); j++)
		{
			pointcloud_local_slice_colour->push_back(XLPE_strip_vector.at(i).each_strip->at(j));
		}
	}
	//showpointcloud_xyzrgb(pointcloud_local_slice_colour);  //展示切片算法的有效性

	/*每条拟合平面，然后根据点到平面距离构建二维图*/
	std::vector<Mapping_2D_3D> mapping_2D_3D; mapping_2D_3D.clear();
	std::vector<float> slice_distance; slice_distance.clear();
	cv::Mat pointcloud_img = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_32FC1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*为了颜色显示*/
	std::vector<Color_mapping> convex_points; convex_points.clear();
	std::vector<Color_mapping> concave_points; concave_points.clear();
	std::vector<Color_mapping> regular_points; regular_points.clear();
	std::vector<double>convex_distance; convex_distance.clear();
	std::vector<double>concave_distance; concave_distance.clear();
	double max_convex = 0; double max_concave = 0;
	/*为了颜色显示*/
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr Plan_mapping(new pcl::PointCloud<pcl::PointXYZHSV>);   //将整个电缆映射成二维平面点云
	for (int i = 0; i < slice_vector.size(); i++) //条的数量,条映射过去是y，片映射过去是x
	{
		plane_pointcloud->clear();
		//把拟合平面的点找出来，用的是整条上所有点
		for (int j = 0; j<slice_vector.at(i).size(); j++) //每条上片的数量
		{
			for (int k = 0; k<slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//每片上点的数量
			{
				plane_pointcloud->push_back(slice_vector.at(i).at(j).pointcloud_strip_slice->at(k));
			}
		}
		//for (int j = 0; j < slice_vector.at(i).size(); j++) //每条上片的数量
		//{
		//	for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//每片上点的数量
		//	{
		//		if ((cable_z_max_value - slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z) < 50)
		//		{
		//			plane_pointcloud->push_back(slice_vector.at(i).at(j).pointcloud_strip_slice->at(k));
		//		}
		//	}
		//}
		//showpointcloud_xyzrgb(plane_pointcloud);
		float A, B, C, D;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point(new pcl::PointCloud<pcl::PointXYZRGB>);
		ransac_plane_estimation(plane_pointcloud, A, B, C, D, interior_point);  //RANSAC平面拟合
																				//float A1, B1, C1, D1;
																				//least_square_plane_estimation(interior_point, A1, B1, C1, D1);  //最小二乘平面拟合
		least_square_plane_estimation(interior_point, A, B, C, D);  //最小二乘平面拟合

																	/*另一种上色方式,通过判断每个点距离平面的距离来对其进行上色，而不是由二维再反转三维*/
		for (int j = 0; j < slice_vector.at(i).size(); j++)  //一条上片的数量
		{
			for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//一片上点的数量
			{
				pcl::PointXYZRGB once_point = slice_vector.at(i).at(j).pointcloud_strip_slice->at(k);
				float point_22_plane_distance = abs(pcl::pointToPlaneDistanceSigned(once_point, A, B, C, D));
				Eigen::Vector4f point1;
				point1 << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //圆柱轴线上一点
				Eigen::Vector4f normal1;
				normal1 << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //圆柱轴线方向向量
				Eigen::Vector4f point_cloud_point1;
				point_cloud_point1 << slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z; //小片点云质心
				float point_22_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point1, point1, normal1));// 空间点到空间直线距离计算，参数分别为点云中一点，直线上一点，直线的法向
				if (point_22_line_distance > cylinder.radius)   //点到轴线距离大于圆柱半径，凸点
				{
					Color_mapping color_mapping;
					color_mapping.point_2_plane_distance = point_22_plane_distance;
					color_mapping.point_index_of_slice = k;
					color_mapping.slice_index_of_strip = j;
					color_mapping.strip_index = i;
					convex_points.push_back(color_mapping);
					convex_distance.push_back(point_22_plane_distance);
				}
				else if (point_22_line_distance < cylinder.radius)	//点到轴线距离小于圆柱半径，凹点
				{
					Color_mapping color_mapping1;
					color_mapping1.point_2_plane_distance = -point_22_plane_distance;
					color_mapping1.point_index_of_slice = k;
					color_mapping1.slice_index_of_strip = j;
					color_mapping1.strip_index = i;
					concave_points.push_back(color_mapping1);
					concave_distance.push_back(point_22_plane_distance);
				}
				else
				{
					Color_mapping color_mapping2;
					color_mapping2.point_2_plane_distance = -point_22_plane_distance;
					color_mapping2.point_index_of_slice = k;
					color_mapping2.slice_index_of_strip = j;
					color_mapping2.strip_index = i;
					regular_points.push_back(color_mapping2);
				}
			}
		}

		/*另一种上色方式,通过判断每个点距离平面的距离来对其进行上色，而不是由二维再反转三维*/

		/*计算点到平面的距离，然后根据条的编号，片的编号，以及点到平面的距离进行映射，条的编号作为图像的y值，片的编号作为x值，点到平面距离作为像素值*/
		for (int j = 0; j < slice_vector.at(i).size(); j++) //每条上片的数量
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			slice_pointcloud->clear();
			float slice_point_2_plane_distance_count = 0;
			for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//每片上点的数量
			{
				slice_pointcloud->push_back(slice_vector.at(i).at(j).pointcloud_strip_slice->at(k));
				//pcl::PointXYZRGB point_cloud_point = slice_vector.at(i).at(j).pointcloud_strip_slice->at(k);
				//point_cloud_point << slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y,
				//	slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z;
				//float point_2_plane_distance = (pcl::pointToPlaneDistanceSigned(point_cloud_point, A, B, C, D));
				float point_2_plane_distance = abs(A*slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x + B*slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y +
					C*slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
				slice_point_2_plane_distance_count = slice_point_2_plane_distance_count + point_2_plane_distance;

			}
			float slice_2_plane_distance = slice_point_2_plane_distance_count / slice_vector.at(i).at(j).pointcloud_strip_slice->size();  //小片点云距离轴线的距离之和除以点数

																																		  /*判断块元的凹凸关系*/
			Eigen::Vector4f centroid;					// 点云质心计算
			pcl::compute3DCentroid(*slice_pointcloud, centroid);	// 齐次坐标，（c0,c1,c2,1）,小片点云质心，若小片点云质心与平面法向量夹角小于90，则在法向量方向一侧，否则在另一侧
			Eigen::Vector4f point;
			point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //圆柱轴线上一点
			Eigen::Vector4f normal;
			normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //圆柱轴线方向向量
			Eigen::Vector4f point_cloud_point;
			point_cloud_point << centroid(0), centroid(1), centroid(2); //小片点云质心
			float point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// 空间点到空间直线距离计算，参数分别为点云中一点，直线上一点，直线的法向量
			Eigen::Vector4f normal_of_point;  //条状点云拟合平面上一点,与非平面上一个块元质心的向量(0,0,-(D/C)是条状点云拟合平面上一点)
			normal_of_point << (centroid(0) - 0), (centroid(1) - 0), (centroid(2) - (-(D / C)));
			float direction = A*normal_of_point(0) + B*normal_of_point(1) + C*normal_of_point(2);   //判断块元质心与法向量朝向的一致性，如果该式大于0，表明该快元在法向量朝向箭头那一侧，否则在尾巴那一侧
																									/*判断平面法向量的朝向，即指向轴线还是背离轴线*/
			Eigen::Vector4f centroid1;					// 条状点云质心计算
			pcl::compute3DCentroid(*plane_pointcloud, centroid1);
			float plane_x = A; float plane_y = B;
			float strip_x = centroid1(0);	float strip_y = centroid1(1);
			float cos_seta = (plane_x*strip_x + plane_y*strip_y) / (sqrt(pow(plane_x, 2) + pow(plane_y, 2)) * sqrt(pow(strip_x, 2) + pow(strip_y, 2)));
			/*通过平面外点与平面上一点形成的法向量，与平面向量的夹角，以及条状点云质心向量与平面向量的夹角判断点位于圆柱内侧还是外侧*/
			if (((direction > 0) && (cos_seta < 0)) || ((direction < 0) && (cos_seta > 0)))
			{
				slice_2_plane_distance = -slice_2_plane_distance;
			}
			/*通过比较小块点云质心据轴线的距离与圆柱半径的距离，判断点位于圆柱内侧还是外侧*/
			//if (point_2_line_distance < cylinder.radius)  //根据块元质心距离轴线的距离来判断是凹还是凸，凹就赋负值，凸就赋正值
			//{
			//	slice_2_plane_distance = -slice_2_plane_distance;
			//}
			Mapping_2D_3D mapping;
			mapping.x = i;  //x是条的编号
			mapping.y = j;	//y是片的编号
			mapping.pixel_value = slice_2_plane_distance; //块元质心距离拟合平面的距离
			mapping_2D_3D.push_back(mapping);
			slice_distance.push_back(slice_2_plane_distance);
			pointcloud_img.at<float>(i, j) = slice_2_plane_distance;

			/*三维点云映射成平面点云显示*/
			pcl::PointXYZHSV hsv_point_tmp;  //每个点x为条的编号，y为块的编号，z为块质心到拟合平面的距离
			hsv_point_tmp.x = i;
			hsv_point_tmp.y = j;
			hsv_point_tmp.z = slice_2_plane_distance;
			if (hsv_point_tmp.z > 1)
			{
				hsv_point_tmp.h = 0;
			}
			else if (hsv_point_tmp.z < -1)
			{
				hsv_point_tmp.h = 240;
			}
			else
			{
				hsv_point_tmp.h = 120 - (hsv_point_tmp.z * 120);
			}
			hsv_point_tmp.s = 200;
			hsv_point_tmp.v = 200;
			Plan_mapping->push_back(hsv_point_tmp);
			/*三维点云映射成平面点云显示*/
		}
	}
	sort(slice_distance.begin(), slice_distance.end());
	cv::Mat orgin_image = pointcloud_img.clone();  //原始映射图像
												   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\orgin_image.png", orgin_image);
												   //showpointcloud_xyzhsv(Plan_mapping); //三维点云映射成平面点云显示

												   /*另一种上色方式,通过判断每个点距离平面的距离来对其进行上色，这种显示更细腻，而不是由二维再反转三维*/
	sort(convex_distance.begin(), convex_distance.end());
	max_convex = convex_distance.back();
	//double convex_rate = 120 / max_convex;    //根据凸的程度决定上色的深浅
	double convex_rate = 120;  //直接以1mm为颜色最深点，大于等于1mm的凸点都是最红的，表明是严重缺陷
	sort(concave_distance.begin(), concave_distance.end());
	max_concave = concave_distance.back();
	//double concave_rate = 120 / max_concave;
	double concave_rate = 120;	//直接以1mm为颜色最深点，大于等于1mm的凹点都是最蓝的，表明是严重缺陷
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr colour_point_cloud_hsv_all(new pcl::PointCloud<pcl::PointXYZHSV>);
	//HSV颜色空间，H=120是绿色，120靠近360是趋近蓝色，120趋近0是趋近红色
	for (int m = 0; m < convex_points.size(); m++)
	{
		pcl::PointXYZHSV point_temp_HSV_convex;
		int strip_index = convex_points.at(m).strip_index;
		int slice_index = convex_points.at(m).slice_index_of_strip;
		int point_index = convex_points.at(m).point_index_of_slice;
		point_temp_HSV_convex.x = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).x;
		point_temp_HSV_convex.y = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).y;
		point_temp_HSV_convex.z = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).z;
		if (convex_points.at(m).point_2_plane_distance > 1)  //凸起超过1mm的，直接让其为最红色
		{
			point_temp_HSV_convex.h = 0;
		}
		else
		{
			point_temp_HSV_convex.h = 120 - (convex_points.at(m).point_2_plane_distance * convex_rate);
		}
		point_temp_HSV_convex.s = 200;
		point_temp_HSV_convex.v = 200;
		colour_point_cloud_hsv_all->push_back(point_temp_HSV_convex);
	}   //凸的部分拿出来
	for (int m = 0; m < concave_points.size(); m++)
	{
		pcl::PointXYZHSV point_temp_HSV_concave;
		int strip_index_concave = concave_points.at(m).strip_index;
		int slice_index_concave = concave_points.at(m).slice_index_of_strip;
		int point_index_concave = concave_points.at(m).point_index_of_slice;
		point_temp_HSV_concave.x = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).x;
		point_temp_HSV_concave.y = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).y;
		point_temp_HSV_concave.z = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).z;
		if ((concave_points.at(m).point_2_plane_distance) < -1)    //凹坑超过1mm的，直接让其为最蓝色
		{
			point_temp_HSV_concave.h = 240;
		}
		else
		{
			point_temp_HSV_concave.h = 120 + (abs(concave_points.at(m).point_2_plane_distance) * concave_rate);
		}
		point_temp_HSV_concave.s = 200;
		point_temp_HSV_concave.v = 200;
		colour_point_cloud_hsv_all->push_back(point_temp_HSV_concave);

	}   //凹的部分拿出来
	for (int m = 0; m < regular_points.size(); m++)
	{
		pcl::PointXYZHSV point_temp_HSV_regular;
		int strip_index_regular = regular_points.at(m).strip_index;
		int slice_index_regular = regular_points.at(m).slice_index_of_strip;
		int point_index_regular = regular_points.at(m).point_index_of_slice;
		point_temp_HSV_regular.x = slice_vector.at(strip_index_regular).at(slice_index_regular).pointcloud_strip_slice->at(point_index_regular).x;
		point_temp_HSV_regular.y = slice_vector.at(strip_index_regular).at(slice_index_regular).pointcloud_strip_slice->at(point_index_regular).y;
		point_temp_HSV_regular.z = slice_vector.at(strip_index_regular).at(slice_index_regular).pointcloud_strip_slice->at(point_index_regular).z;
		point_temp_HSV_regular.h = 120;
		point_temp_HSV_regular.s = 200;
		point_temp_HSV_regular.v = 200;
		colour_point_cloud_hsv_all->push_back(point_temp_HSV_regular);
	} //正常的部分拿出来
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_all);
	__colour_point_cloud_hsv_all__ = colour_point_cloud_hsv_all->makeShared();
	/*另一种上色方式,通过判断每个点距离平面的距离来对其进行上色，而不是由二维再反转三维*/

	/*将映射图像中的负值全部变为正值*/
	cv::Mat pointcloud_img_plus = cv::Mat::zeros(pointcloud_img.size(), CV_32FC1);
	float pixel_small = slice_distance.front();
	cv::Mat  img_tmp = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_32FC1);
	//float trans_rate = 240 / (slice_distance.back()+ abs(pixel_small));
	for (int i = 0; i < pointcloud_img.rows; i++)
	{
		for (int j = 0; j<pointcloud_img.cols; j++)
		{
			pointcloud_img_plus.at<float>(i, j) = pointcloud_img.at<float>(i, j) + abs(pixel_small);//让负的部分全部变为正的，便于后续处理
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\pointcloud_img_plus.png", pointcloud_img_plus);

	/*像素放大，凸显缺陷*/
	float minValue = *max_element(pointcloud_img_plus.begin<float>(), pointcloud_img_plus.end<float>());  //算出原始图像全部映射为正值后的最大值，方便后面计算放大倍数
	int enlarge_rate = 250 / minValue; //像素放大倍数
	for (int i = 0; i < pointcloud_img_plus.rows; i++)
	{
		for (int j = 0; j < pointcloud_img_plus.cols; j++)
		{
			img_tmp.at<float>(i, j) = pointcloud_img_plus.at<float>(i, j) * enlarge_rate;    //放大缺陷与正常像素的差距
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\enlarge_img_A"+to_string(each_strip_angle)+"_h_"+ to_string(slice_height)+".png", img_tmp);

	/*将float数据转为int，因为很多opencv很多操作只支持8位无符号型*/
	cv::Mat HSV_img_h = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_8UC1);
	for (int i = 0; i < img_tmp.rows; i++)
	{
		for (int j = 0; j < img_tmp.cols; j++)
		{
			HSV_img_h.at<uchar>(i, j) = (int)(img_tmp.at<float>(i, j));
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\float_2_int.png", HSV_img_h);

	/*图像像素众数求取*/
	std::vector<int> image_pixel_value; image_pixel_value.clear();
	for (int i = 0; i<HSV_img_h.rows; i++)
	{
		for (int j = 0; j<HSV_img_h.cols; j++)
		{
			image_pixel_value.push_back(HSV_img_h.at<uchar>(i, j));  //先把像素值都放进数组
		}
	}
	sort(image_pixel_value.begin(), image_pixel_value.end()); //排序
															  //int mode_pixel_value = image_pixel_value[image_pixel_value.size() / 2];  //当特殊情况时（众数占比非常大的情况），众数就等于数组中中间那个数
	std::vector<int> pixel_value; pixel_value.clear();   //用来存每一个不重复的像素值
	pixel_value.push_back(image_pixel_value[0]);		//第一个元素为像素值排序后出现的第一个值
	std::vector<int> frequency; frequency.clear();//用来记录每一个像素值出现了多少次
	frequency.push_back(1);	//第一个元素赋初始值，出现一次
	int m = 0;
	for (int i = 1; i<image_pixel_value.size(); i++)
	{
		if (image_pixel_value[i] != image_pixel_value[i - 1])  //如果当前像素值和前一个像素值不相等
		{
			pixel_value.push_back(image_pixel_value[i]);  //把当前像素值存起来
			frequency.push_back(1);	//并且赋出现的次数为1
			m++;	//同时让记录次数的位置后移一位
		}
		else
		{
			frequency[m]++; //如果当前像素值和前一个像素值相等，则当前像素值出现的次数加1
		}
	}
	int maxPosition = max_element(frequency.begin(), frequency.end()) - frequency.begin(); //出现次数最多的像素值的下标
	int mode_value = pixel_value.at(maxPosition);//出现次数最多的像素值，众数

												 /*边缘检测*/
	cv::Mat boundary;
	cv::Canny(HSV_img_h, boundary, 100, 200, 3);

	/*均值图像求取*/
	Mat image_mean;
	Mat image_std;
	cv::meanStdDev(HSV_img_h, image_mean, image_std);
	int image_mean_value = (int)(image_mean.at<double>(0, 0));
	int image_media_value = image_pixel_value.at(image_pixel_value.size() / 2);
	cv::Mat mean_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(image_mean_value - 10));  //创建值全为均值像素的图像
																					   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\mean_image.png", mean_image);
	cv::Mat mode_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(mode_value));  //创建值全为众数像素的图像
																			//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\mode_image.png", mode_image);
	cv::Mat media_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(image_media_value + 10));  //创建值全为中值像素的图像
																						 //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\media_image.png", media_image);

																						 /*映射图像像素值数据写出*/
																						 //FILE * fpi;
																						 //if ((fpi = fopen("image_pixel_value.txt", "wb")) == NULL)
																						 //{
																						 //	printf("cant open the file");
																						 //	exit(0);
																						 //}
																						 ////int index = 1;
																						 //for (size_t i = 0; i < HSV_img_h.cols; i++)
																						 //{
																						 //	for (size_t j = 0; j < HSV_img_h.rows; j++)
																						 //	{
																						 //		fprintf(fpi, "%d ", i);
																						 //		fprintf(fpi, "%s ", ",");
																						 //		fprintf(fpi, "%d ", j);
																						 //		fprintf(fpi, "%s ", ",");
																						 //		fprintf(fpi, "%d ", HSV_img_h.at<uchar>(Point(i, j)));
																						 //		fprintf(fpi, "%s ", "\n");
																						 //		//index++;
																						 //	}
																						 //}
																						 //fclose(fpi);
																						 //cout << "像素值输出完成!" << endl;


	/*凸区域求取，映射图像减均值图像*/	cv::Mat Convex;
	cv::subtract(HSV_img_h, mode_image, Convex);
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Convex_noise.png", Convex);

	Mat Convex_clour(Convex.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	/*均值图像减映射图像求凹区域*/
	cv::Mat concave;
	cv::subtract(mode_image, HSV_img_h, concave);
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Concave_noise.png", concave);
	Mat concave_clour(concave.size(), CV_8UC3, cv::Scalar(255, 255, 255));

	/*将所有缺陷像素提取出来*/
	std::vector<Coordinate_value> goal_pixel_convex; goal_pixel_convex.clear();
	for (int i = 0; i < Convex.rows; i++)
	{
		for (int j = 0; j < Convex.cols; j++)
		{
			if (Convex.at<uchar>(i, j) > 8)
			{
				Coordinate_value tmp;
				tmp.x = i;
				tmp.y = j;
				tmp.pixel_value = Convex.at<uchar>(i, j);
				goal_pixel_convex.push_back(tmp);
			}
		}
	}

	/******************************凸缺陷处理*******************************/
	cout << "凸起缺陷检测与量化开始！" << endl;
	/*图像二值化与滤波*/
	float minimum_defect = 0.2;	//控制凸起缺陷大于minimum_defect mm的才会被检测
	int Binarization_thresholdabs = (int)((minimum_defect + abs(pixel_small))*enlarge_rate) - mode_value;
	Mat Convex_binary;
	//threshold(Convex, Convex_binary, 8, 255, THRESH_BINARY);  //二值化
	cv::threshold(Convex, Convex_binary, Binarization_thresholdabs, 255, THRESH_BINARY);  //二值化,大于Binarization_thresholdabs像素值的全部置数为255，Binarization_thresholdabs受至少检测多凸多凹的缺陷控制
	Mat Convex_binary_filter;
	cv::medianBlur(Convex_binary, Convex_binary_filter, 3);  //二值化图像滤波
															 //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Convex_binary_filter" + to_string(minimum_defect) +".png", Convex_binary_filter);

															 /*缺陷轮廓提取*/
	vector<vector<Point>> contours; contours.clear();
	vector<Vec4i> hierarchy;
	cv::findContours(Convex_binary_filter, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point());  //轮廓获取
	Mat imageContours = Mat::zeros(Convex.size(), CV_8UC1);
	Mat Contours = Mat::zeros(Convex.size(), CV_8UC1);  //绘制
	for (int i = 0; i < contours.size(); i++) //轮廓数量
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for (int j = 0; j < contours[i].size(); j++)
		{
			//绘制出contours向量内所有的像素点
			Point P = Point(contours[i][j].x, contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		//绘制轮廓
		drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
	}
	//imshow("Contours Image", imageContours); //轮廓
	//imshow("Point of Contours", Contours);   //向量contours内保存的所有轮廓点集

	/*缺陷轮廓外接矩*/
	std::vector<Contour_point> Contour_point_4; Contour_point_4.clear();
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(contours[i]);  //计算凸包，然后计算外接矩
		Point2f P[4];
		rect.points(P); //计算外接矩4个顶点
						//把每个轮廓的4个点存起来
		Contour_point contour_point;
		contour_point.point_1 = P[0]; contour_point.point_2 = P[1];	contour_point.point_3 = P[2]; contour_point.point_4 = P[3]; //外接矩的4个顶点
		Contour_point_4.push_back(contour_point);	//存储
		for (int j = 0; j <= 3; j++)
		{
			//Convex_binary_filter,被绘对象，P[j]线的第一个点，P[(j + 1) % 4]线的第二个点，cv::Scalar(0,0,255)线的颜色，1线的粗细，8线的类型，0线的偏移量
			line(Convex_binary_filter, P[j], P[(j + 1) % 4], cv::Scalar(255), 1, 8, 0);  //CV_8UC1灰度图不能绘制彩色线条
		}
		//cout << "第 "<< i <<" 个凸起缺陷外接矩顶点提取完成..." << endl;
	}
	/*确定每一个缺陷包含的像素点*/
	std::vector<std::vector<Point2f>> defect_location; defect_location.clear();
	for (int i = 0; i < Contour_point_4.size(); i++) //遍历每一个外接矩
	{
		std::vector<int> x_vector; x_vector.clear(); std::vector<int> y_vector; y_vector.clear();  //将每一个缺陷外接矩形框的4个点拿出来
		int x1 = Contour_point_4.at(i).point_1.x; int y1 = Contour_point_4.at(i).point_1.y;
		x_vector.push_back(x1); y_vector.push_back(y1);
		int x2 = Contour_point_4.at(i).point_2.x; int y2 = Contour_point_4.at(i).point_2.y;
		x_vector.push_back(x2); y_vector.push_back(y2);
		int x3 = Contour_point_4.at(i).point_3.x; int y3 = Contour_point_4.at(i).point_3.y;
		x_vector.push_back(x3); y_vector.push_back(y3);
		int x4 = Contour_point_4.at(i).point_4.x; int y4 = Contour_point_4.at(i).point_4.y;
		x_vector.push_back(x4); y_vector.push_back(y4);
		sort(x_vector.begin(), x_vector.end());	sort(y_vector.begin(), y_vector.end());   //计算得到缺陷外接矩形框4个点的最大最小x值与y值
		int x_small = x_vector.front(); int x_big = x_vector.back(); //可以选择把刚才求的缺陷外接矩向外在4个方向上（）上下左右都扩一个像素，因为外接矩有压线的情况
		int y_small = y_vector.front(); int y_big = y_vector.back();
		/*遍历缺陷分布图像中所有像素，然后将落在每一个矩形框（x_small，y_small）与（x_big，y_big）中不等于0的像素坐标拿出来*///相当于给缺陷定位
		std::vector<Point2f> once_defet_location; once_defet_location.clear();
		//for (int j = 0; j < Convex.cols;j++)
		//{
		//	for (int k = 0; k<Convex.rows; k++)
		//	{
		//		if ((x_small < j) && (j < x_big) && (y_small < k) && (k < y_big) && (Convex.at<uchar>(k,j)>0)) //如果缺陷分布图中像素坐标落在矩形框内，而且像素值大于0
		//		{
		//			Point2f once =  Point2f(j, k);
		//			once_defet_location.push_back(once);
		//		}
		//	}
		//}
		for (int j = 0; j < Convex.cols; j++) //如果在外接矩内的像素大于Binarization_thresholdabs，则认为其实缺陷像素
		{
			for (int k = 0; k < Convex.rows; k++)
			{
				if ((x_small <= j) && (j <= x_big) && (y_small <= k) && (k <= y_big) && (Convex.at<uchar>(Point(j, k)) >= Binarization_thresholdabs)) //如果缺陷分布图中像素坐标落在矩形框内，而且像素值大于Binarization_thresholdabs
				{
					Point2f once = Point2f(j, k);  //j是条的编号，k是块的编号
					once_defet_location.push_back(once);
				}
			}
		}
		if (once_defet_location.size() < 5) //如果一个缺陷小于5个像素，就舍弃
		{
			continue;
		}
		defect_location.push_back(once_defet_location);
		//cout << "第 " << i << " 个凸起缺陷坐标提取完成..." << endl;
	}
	cout << "凸起缺陷检测与量化完成！" << endl;

	/*给各缺陷赋不同颜色，在二维图像上进行显示，用来判断缺陷定位的正确性，看是否有不是当前缺陷的点被判断进来*/
	for (int i = 0; i<defect_location.size(); i++) //缺陷个数
	{
		int b, g, r; b = g = r = 0;
		random_colour(b, g, r);
		for (int j = 0; j < defect_location.at(i).size(); j++)
		{
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[0] = b;  //访问三通道的b通道并给（x = i.y = j）的位置赋值
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[1] = g;
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[2] = r;
		}
	}

	/*各缺陷体积、面积、最深值计算*/
	//图像x对应的是原始圆柱上的纵向高度方向，图像y对应的是原始圆柱抽条(x,y)对应三维点云代表的是第y条上第x片
	Defection_quantification.clear();
	double max_convex_pixel = 0;  //用来记录整个工件最凸点的凸值，方便颜色映射，避免颜色映射越界
	for (int i = 0; i < defect_location.size(); i++)
	{
		double each_slice_height = slice_height; //前面抽条高度已经决定
		double each_slice_width = (each_strip_angle * cylinder.radius*M_PI) / 180;	//弧长等于角度x半径，这里直接用的圆柱半径，没有计算各块元质心半径，因为相差并不会太大
		double defection_one_pixel_area = each_slice_height * each_slice_width;     //这就是一个像素代表的面积，体积的话就等于原始图像像素值乘以该面积
		double defection_area = defection_one_pixel_area*defect_location.at(i).size();	//缺陷面积等于每个像素的面积乘以缺陷所占像素数量
																						//在原始映射图像中去找对应位置的像素计算体积
		double max_pixel_value = orgin_image.at<float>(defect_location.at(i).at(0));  //先假定缺陷局域最大像素值就为目标内第一个
		double once_defection_volume = 0;
		int x_tmp = 0; int y_tmp = 0;  //把缺陷最凸的地方记录下来，方便在三维上指定位置
		for (int k = 0; k < defect_location.at(i).size(); k++)
		{
			if (max_pixel_value < orgin_image.at<float>(defect_location.at(i).at(k)))
			{
				max_pixel_value = orgin_image.at<float>(defect_location.at(i).at(k));//把当前缺陷中最大像素值找出来
				x_tmp = defect_location.at(i).at(k).x;  //条的编号
				y_tmp = defect_location.at(i).at(k).y;	//块的编号
			}
			once_defection_volume = once_defection_volume + (orgin_image.at<float>(defect_location.at(i).at(k))*defection_one_pixel_area);  //计算缺陷体积，缺陷体积=缺陷区域各像素值*面积
		}
		if (max_pixel_value>max_convex_pixel)  //把当前工件中最大像素值找出来
		{
			max_convex_pixel = max_pixel_value;
		}

		defection_quantification once_detection;	//将该缺陷的各参数进行存储
		once_detection.defection_area = defection_area;		//缺陷影响圆柱面积
		once_detection.defection_area_defection = defection_area + defection_area/10;  //缺陷表面积
		once_detection.defection_deepest_point = max_pixel_value;//最凸点的凸值
		once_detection.defection_loaction_x = x_tmp;		//缺陷最凸点的x值，块的编号
		once_detection.defection_loaction_y = y_tmp;		//缺陷最凸点的y值，条的编号
		once_detection.defection_volume = once_defection_volume; //体积
		once_detection.conductEnd_2_outEnd_distance1 = conductEnd_2_outEnd_distance - x_tmp *each_slice_height;//最凸点距离线芯顶端的距离值
		once_detection.defection_flag = i; //把当前缺陷的编号记录下来
		Defection_quantification.push_back(once_detection);

		/*输出方便确定最佳参数*/
		//fstream f1;
		//f1.open("convex_R40.txt", ios::out | ios::app);
		//f1 << "H = ," << slice_height << "," << "A = ," << each_strip_angle << ",";
		//f1 << defection_area;
		//f1 << ",";
		//f1 << once_defection_volume;
		//f1 << endl;
		//f1.close();

		//cout << "距离底端 " << x_tmp *each_slice_height << " mm处凸起缺陷量化完成，缺陷编号为 " << i << endl;
	}

	/*在三维中将缺陷进行显示，不同颜色：即表示为不同的缺陷又表示为不同程度的凸起*/
	//int colour_rate = 120 / max_convex_pixel;  //最红点位像素值最大的地方
	//for (int i = 0;i < Defection_quantification.size();i++)  //缺陷个数
	//{
	//	int defect_location_index = Defection_quantification.at(i).defection_flag;
	//	for (int j = 0; j < defect_location.at(defect_location_index).size();j++)  //当前缺陷包含的小片点云个数
	//	{
	//		int slice_index = defect_location.at(defect_location_index).at(j).x;
	//		int strip_index = defect_location.at(defect_location_index).at(j).y;
	//		for (int k = 0;k < slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->size();k++)  //找到指定小片进行赋值,小片缺陷里面包含的点数
	//		{
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 0;
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = orgin_image.at<float>(defect_location.at(i).at(j))*colour_rate;  //根据凸起高度值赋颜色
	//		}
	//	}
	//}
	int colour_rate = 120 / 1;  //固定最红点为1mm处，改变分母就能改变最红点
	for (int i = 0; i < Defection_quantification.size(); i++)  //缺陷个数
	{
		int defect_location_index = Defection_quantification.at(i).defection_flag;
		for (int j = 0; j < defect_location.at(defect_location_index).size(); j++)  //当前缺陷包含的小片点云个数
		{
			int slice_index = defect_location.at(defect_location_index).at(j).x;
			int strip_index = defect_location.at(defect_location_index).at(j).y;
			for (int k = 0; k < slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->size(); k++)  //找到指定小片进行赋值,小片缺陷里面包含的点数
			{
				slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 0;
				slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
				if (orgin_image.at<float>(defect_location.at(i).at(j)) > 1)  //避免HSV颜色空间越界
				{
					slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = 120; //避免越界
				}
				else
				{
					slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = (orgin_image.at<float>(defect_location.at(i).at(j)) / 1)*(colour_rate);  //根据凸起高度值赋颜色
				}
			}
		}
	}

	/*显示上色后的三维点云*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i<slice_vector.size(); i++)
	{
		for (int j = 0; j<slice_vector.at(i).size(); j++)
		{
			for (int k = 0; k<slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)
			{
				colour_point_cloud->push_back(slice_vector.at(i).at(j).pointcloud_strip_slice->at(k));
			}
		}
	}
	//showpointcloud_xyzrgb(colour_point_cloud);  //凸缺陷为红色，正常和绿色为白色
	/*HSV颜色显示，正常为绿色，凸起为红色*/
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr colour_point_cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
	for (int i = 0; i<colour_point_cloud->size(); i++)
	{
		pcl::PointXYZHSV point_temp;
		point_temp.x = colour_point_cloud->at(i).x;
		point_temp.y = colour_point_cloud->at(i).y;
		point_temp.z = colour_point_cloud->at(i).z;
		if (colour_point_cloud->at(i).r != 255)
		{
			point_temp.h = 120 - colour_point_cloud->at(i).r;
		}
		else
		{
			point_temp.h = 120;
		}
		point_temp.s = 200; point_temp.v = 200;
		colour_point_cloud_hsv->push_back(point_temp);
	}
	//showpointcloud_xyzhsv(colour_point_cloud_hsv);  //凸缺陷显示，凸缺陷显示为红色，越凸越红，正常和凹缺陷都为绿色
	/******************************凸缺陷处理*******************************/

	/*************************************凹缺陷处理**************************************/
	cout << "凹陷缺陷检测与量化开始！" << endl;
	/*图像二值化与滤波*/
	float minimum_defect_concave = -0.2;	//控制凹缺陷大于minimum_defect mm的才会被检测
	int Binarization_thresholdabs_concave = mode_value - (int)((minimum_defect_concave + abs(pixel_small))*enlarge_rate);
	Mat concave_binary;
	cv::threshold(concave, concave_binary, Binarization_thresholdabs_concave, 255, THRESH_BINARY);  //二值化
	Mat concave_binary_filter;
	cv::medianBlur(concave_binary, concave_binary_filter, 3);  //二值化图像滤波
															   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Concave_binary_filter" + to_string(minimum_defect)+".png", concave_binary_filter);
															   /*缺陷轮廓提取*/
	vector<vector<Point>> contours_concave; contours_concave.clear();
	vector<Vec4i> hierarchy_concave;
	cv::findContours(concave_binary_filter, contours_concave, hierarchy_concave, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point());  //轮廓获取
	Mat imageContours_concave = Mat::zeros(concave.size(), CV_8UC1);
	Mat Contours_concave = Mat::zeros(concave.size(), CV_8UC1);  //绘制
	for (int i = 0; i < contours_concave.size(); i++) //轮廓数量
	{
		//contours_concave[i]代表的是第i个轮廓，contours_concave[i].size()代表的是第i个轮廓上所有的像素点数
		for (int j = 0; j < contours_concave[i].size(); j++)
		{
			//绘制出contours_concave向量内所有的像素点
			Point P_concave = Point(contours_concave[i][j].x, contours_concave[i][j].y);
			Contours_concave.at<uchar>(P_concave) = 255;
		}
		//绘制轮廓
		drawContours(imageContours_concave, contours_concave, i, Scalar(255), 1, 8, hierarchy_concave);
	}
	//imshow("Contours_concave Image", imageContours_concave); //轮廓
	//imshow("Point of Contours_concave", Contours_concave);   //向量contours_concave内保存的所有轮廓点集

	/*缺陷轮廓外接矩求取*/
	std::vector<Contour_point> Contour_point_4_concave; Contour_point_4_concave.clear();
	for (int i = 0; i < contours_concave.size(); i++)
	{
		RotatedRect rect_concave = minAreaRect(contours_concave[i]);
		Point2f P_concave[4];
		rect_concave.points(P_concave);
		//把每个轮廓的4个点存起来
		Contour_point contour_point_concave;
		contour_point_concave.point_1 = P_concave[0]; contour_point_concave.point_2 = P_concave[1];	contour_point_concave.point_3 = P_concave[2]; contour_point_concave.point_4 = P_concave[3]; //外接矩的4个顶点
		Contour_point_4_concave.push_back(contour_point_concave);	//存储
		for (int j = 0; j <= 3; j++)
		{
			//Convex_binary_filter,被绘对象，P_concave[j]线的第一个点，P_concave[(j + 1) % 4]线的第二个点，cv::Scalar(0,0,255)线的颜色，1线的粗细，8线的类型，0线的偏移量
			line(concave_binary_filter, P_concave[j], P_concave[(j + 1) % 4], cv::Scalar(255), 1, 8, 0);  //CV_8UC1灰度图不能绘制彩色线条
		}
		//cout << "第 " << i << " 个凹缺陷外接矩顶点提取完成..." << endl;
	}
	/*根据外接矩确定每一个缺陷包含的像素点坐标*/
	std::vector<std::vector<Point2f>> defect_location_concave; defect_location_concave.clear();  //存储每个缺陷包含的像素坐标
	for (int i = 0; i < Contour_point_4_concave.size(); i++) //遍历每一个外接矩
	{
		std::vector<int> x_vector; x_vector.clear(); std::vector<int> y_vector; y_vector.clear();  //将每一个缺陷外接矩形框的4个点拿出来
		int x1_concave = Contour_point_4_concave.at(i).point_1.x; int y1_concave = Contour_point_4_concave.at(i).point_1.y;
		x_vector.push_back(x1_concave); y_vector.push_back(y1_concave);
		int x2_concave = Contour_point_4_concave.at(i).point_2.x; int y2_concave = Contour_point_4_concave.at(i).point_2.y;
		x_vector.push_back(x2_concave); y_vector.push_back(y2_concave);
		int x3_concave = Contour_point_4_concave.at(i).point_3.x; int y3_concave = Contour_point_4_concave.at(i).point_3.y;
		x_vector.push_back(x3_concave); y_vector.push_back(y3_concave);
		int x4_concave = Contour_point_4_concave.at(i).point_4.x; int y4_concave = Contour_point_4_concave.at(i).point_4.y;
		x_vector.push_back(x4_concave); y_vector.push_back(y4_concave);
		sort(x_vector.begin(), x_vector.end());	sort(y_vector.begin(), y_vector.end());   //计算得到缺陷外接矩形框4个点的最大最小x值与y值
		int x_small_concave = x_vector.front(); int x_big_concave = x_vector.back(); //可以选择把刚才求的缺陷外接矩向外在4个方向上（）上下左右都扩一个像素，因为外接矩有压线的情况
		int y_small_concave = y_vector.front(); int y_big_concave = y_vector.back();
		/*遍历缺陷分布图像中所有像素，然后将落在每一个矩形框（x_small_concave，y_small_concave）与（x_big_concave，y_big_concave）中不等于0的像素坐标拿出来*///相当于给缺陷定位
		std::vector<Point2f> once_defet_location_concave; once_defet_location_concave.clear();
		//for (int j = 0; j < Convex.cols;j++)
		//{
		//	for (int k = 0; k<Convex.rows; k++)
		//	{
		//		if ((x_small < j) && (j < x_big) && (y_small < k) && (k < y_big) && (Convex.at<uchar>(k,j)>0)) //如果缺陷分布图中像素坐标落在矩形框内，而且像素值大于0
		//		{
		//			Point2f once =  Point2f(j, k);
		//			once_defet_location.push_back(once);
		//		}
		//	}
		//}
		for (int j = 0; j < concave.cols; j++)
		{
			for (int k = 0; k < concave.rows; k++)
			{
				//如果缺陷分布图中像素坐标落在矩形框内，而且像素值小于minimum_defect_concave
				if ((x_small_concave <= j) && (j <= x_big_concave) && (y_small_concave <= k) && (k <= y_big_concave) && (concave.at<uchar>(Point(j, k)) >= Binarization_thresholdabs_concave))
				{
					Point2f once_concave = Point2f(j, k);
					once_defet_location_concave.push_back(once_concave);
				}
			}
		}
		if (once_defet_location_concave.size() < 5) //如果一个缺陷小于5个像素，就舍弃
		{
			continue;
		}
		defect_location_concave.push_back(once_defet_location_concave);
		//cout << "第 " << i << " 个凹缺陷坐标提取完成..." << endl;
	}
	/*给各缺陷赋不同颜色的，在二维图像上显示*/
	for (int i = 0; i < defect_location_concave.size(); i++)
	{
		int b, g, r; b = g = r = 0;
		random_colour(b, g, r);
		for (int j = 0; j < defect_location_concave.at(i).size(); j++)
		{
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[0] = b;  //访问三通道的b通道并给（x = i.y = j）的位置赋值
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[1] = g;
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[2] = r;
		}
	}

	/*各缺陷体积、面积、最深值计算*/
	//图像x对应的是原始圆柱上的纵向高度方向，图像y对应的是原始圆柱抽条(x,y)对应三维点云代表的是第y条上第x片
	Defection_quantification_concave.clear();
	double min_concave_pixel = 0; //整个电缆的最低点
	for (int i = 0; i < defect_location_concave.size(); i++)	//缺陷数量
	{
		double each_slice_height_concave = slice_height; //前面抽条高度已经决定
		double each_slice_width_concave = (each_strip_angle * cylinder.radius*M_PI) / 180;	//弧长等于角度x半径，这里直接用的圆柱半径，没有计算各块元质心半径，因为相差并不会太大
		double defection_one_pixel_area_concave = each_slice_height_concave * each_slice_width_concave;     //这就是一个像素代表的面积，体积的话就等于原始图像像素值乘以该面积
		double defection_area_concave = defection_one_pixel_area_concave*defect_location_concave.at(i).size();	//缺陷面积等于每个像素的面积乘以缺陷所占像素数量
																												//在原始映射图像中去找对应位置的像素计算体积，同时找出最小像素值，这个值对应凹坑最低点
		double min_pixel_value_concave = orgin_image.at<float>(defect_location_concave.at(i).at(0));  //先假定缺陷局域最小像素值就为目标内第一个
		double once_defection_volume_concave = 0;
		int x_tmp_concave = 0; int y_tmp_concave = 0;  //把缺陷最凸的地方记录下来，方便在三维上指定位置
		for (int k = 0; k < defect_location_concave.at(i).size(); k++)
		{
			if (min_pixel_value_concave > orgin_image.at<float>(defect_location_concave.at(i).at(k)))
			{
				min_pixel_value_concave = orgin_image.at<float>(defect_location_concave.at(i).at(k));
				x_tmp_concave = defect_location_concave.at(i).at(k).x;  //条的编号
				y_tmp_concave = defect_location_concave.at(i).at(k).y;	//块的编号
			}
			once_defection_volume_concave = once_defection_volume_concave + (abs(orgin_image.at<float>(defect_location_concave.at(i).at(k)))*defection_one_pixel_area_concave);  //计算缺陷体积，缺陷体积=缺陷区域各像素值*面积
		}
		if (min_pixel_value_concave < min_concave_pixel)  //把当前工件上缺陷最小像素值找出来
		{
			min_concave_pixel = min_pixel_value_concave;
		}

		defection_quantification once_detection_concave;			//将该缺陷的各参数进行存储
		once_detection_concave.defection_area = defection_area_concave;		//缺陷影响的圆柱面积
		once_detection_concave.defection_area_defection = defection_area_concave + defection_area_concave/10;  //缺陷表面积
		once_detection_concave.defection_deepest_point = min_pixel_value_concave;//最凹点的凹值
		once_detection_concave.defection_loaction_x = x_tmp_concave;		//缺陷最凹点的x值，对应的是条的编号（x,y）对应的是第x块第y条
		once_detection_concave.defection_loaction_y = y_tmp_concave;		//缺陷最凹点的y值，对应的是块的编号
		once_detection_concave.defection_volume = once_defection_volume_concave; //体积
		once_detection_concave.conductEnd_2_outEnd_distance1 = conductEnd_2_outEnd_distance - x_tmp_concave * each_slice_height_concave;//最凸点距离线芯顶端的距离值
		once_detection_concave.defection_flag = i; //把当前缺陷的编号记录下来
		Defection_quantification_concave.push_back(once_detection_concave);

		/*输出方便确定最佳参数*/
		//fstream f;
		//f.open("concave_R40.txt", ios::out | ios::app);
		//f << "H = ," << slice_height << "," << "A = ," << each_strip_angle << ",";
		//f << defection_area_concave;
		//f << ",";
		//f << once_defection_volume_concave;
		//f << endl;
		//f.close();

		//cout << "距离底端 " << x_tmp_concave * each_slice_height_concave << " mm处凹缺陷量化完成，缺陷编号为 " << i << endl;
	}
	cout << "凹陷缺陷检测与量化完成！" << endl;

	/*在三维中将缺陷进行显示，不同颜色：即表示为不同的缺陷又表示为不同程度的凸起*/   //在凸起处已经将凸起上色，这里给凹坑上色
												//int colour_rate_concave = 120 / (abs(min_concave_pixel));   //根据最深值反映颜色映射值
	int colour_rate_concave = 120;  //限定最深为1mm，1mm及以上都是深蓝色显示
	for (int i = 0; i < Defection_quantification_concave.size(); i++)  //缺陷个数
	{
		int defect_location_index_concave = Defection_quantification_concave.at(i).defection_flag;
		for (int j = 0; j < defect_location_concave.at(defect_location_index_concave).size(); j++)  //当前缺陷包含的小片点云个数
		{
			int slice_index = defect_location_concave.at(defect_location_index_concave).at(j).x;
			int strip_index = defect_location_concave.at(defect_location_index_concave).at(j).y;
			for (int k = 0; k < slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->size(); k++)  //找到指定小片进行赋值,小片缺陷里面包含的点数
			{
				if (orgin_image.at<float>(defect_location_concave.at(i).at(j)) < -1)//避免HSV颜色空间越界
				{
					slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 120;
				}
				else
				{
					slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = (abs(orgin_image.at<float>(defect_location_concave.at(i).at(j))) / 1)*colour_rate_concave;
				}
				slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
				slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = 0;  //根据凸起高度值赋颜色
			}
		}
	}
	/*显示上色后的三维点云，正常是白色，凹区域是蓝色*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour_point_cloud_concave(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < slice_vector1.size(); i++)
	{
		for (int j = 0; j < slice_vector1.at(i).size(); j++)
		{
			for (int k = 0; k < slice_vector1.at(i).at(j).pointcloud_strip_slice->size(); k++)
			{
				colour_point_cloud_concave->push_back(slice_vector1.at(i).at(j).pointcloud_strip_slice->at(k));
			}
		}
	}
	//showpointcloud_xyzrgb(colour_point_cloud_concave);  //显示凹坑区域,xyzrgb类型显示，想分别显示就用vector1，想一起显示就用vector

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr colour_point_cloud_hsv_concave(new pcl::PointCloud<pcl::PointXYZHSV>);
	for (int i = 0; i < colour_point_cloud_concave->size(); i++)
	{
		pcl::PointXYZHSV point_temp_concave;
		point_temp_concave.x = colour_point_cloud_concave->at(i).x;
		point_temp_concave.y = colour_point_cloud_concave->at(i).y;
		point_temp_concave.z = colour_point_cloud_concave->at(i).z;
		if (colour_point_cloud_concave->at(i).b != 255)    //找出凹区域
		{
			point_temp_concave.h = 120 + colour_point_cloud_concave->at(i).b;
		}
		else
		{
			point_temp_concave.h = 120;
		}
		point_temp_concave.s = 200; point_temp_concave.v = 200;
		colour_point_cloud_hsv_concave->push_back(point_temp_concave);
	}
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);  //单独显示凹缺陷，正常为绿色，凹坑为蓝色

	for (int i = 0; i<colour_point_cloud_hsv->size(); i++)	//同一根电缆上显示凹凸缺陷
	{
		if (colour_point_cloud_hsv->at(i).h != 120)
		{
			colour_point_cloud_hsv_concave->push_back(colour_point_cloud_hsv->at(i));
		}
	}
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);  //在主绝缘点云上同时显示凹陷与凸起缺陷

	//fstream fx;
	//fx.open("data.txt", ios::out | ios::app);
	//fx << "值分别是凹陷缺陷面积、体积、凸起面积、体积_A= " + to_string(each_strip_angle) + "_h= " + to_string(slice_height) << "," << Defection_quantification_concave.at(0).defection_area << " ," << Defection_quantification_concave.at(0).defection_volume
	//	<< "," << "," << Defection_quantification.at(0).defection_area << "," << Defection_quantification.at(0).defection_volume << endl;
	////showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);
	//cout << "缺陷输出" << "A" + to_string(each_strip_angle) + "_h_" + to_string(slice_height) + "凹陷缺陷面积值 = "
	//	<< Defection_quantification_concave.at(0).defection_area << " 体积值 = " << Defection_quantification_concave.at(0).defection_volume << endl;
	//cout << "缺陷输出" << "A" + to_string(each_strip_angle) + "_h_" + to_string(slice_height) + "凸起缺陷面积值 = "
	//	<< Defection_quantification.at(0).defection_area << " 体积值 = " << Defection_quantification.at(0).defection_volume << endl;
	/*************************************凹缺陷处理**************************************/
	/*量化结果输出*/
	//fstream f;
	//f.open("convex.txt", ios::out | ios::app);
	//f << "h = " << slice_height << "," << "A = " << each_strip_angle << endl;
	//for (int i = 0; i < Defection_quantification_concave.size(); i++)
	//{
	//	f << Defection_quantification_concave.at(i).defection_area;
	//	f << ",";
	//	f << Defection_quantification_concave.at(i).defection_volume;
	//	f << endl;
	//}
	//f.close();

	//fstream f1;
	//f1.open("concave.txt", ios::out | ios::app);
	//f1 << "h = " << slice_height << "," << "A = " << each_strip_angle << endl;
	//for (int i = 0; i < Defection_quantification.size(); i++)
	//{
	//	f1 << Defection_quantification.at(i).defection_area;
	//	f1 << ",";
	//	f1 << Defection_quantification.at(i).defection_volume;
	//	f1 << endl;
	//}
	//f1.close();

	/*采用小片到轴线的方式构建二维图*/
	//std::vector<Mapping_2D_3D> mapping_2D_3D; mapping_2D_3D.clear();
	//std::vector<float> slice_distance; slice_distance.clear();
	//cv::Mat pointcloud_img = cv::Mat::zeros(cv::Size( slice_vector.at(0).size(), slice_vector.size()),CV_32FC1);
	//for (int i = 0; i < slice_vector.size(); i++) //条的数量
	//{
	//	for (int j = 0;j<slice_vector.at(i).size();j++) //每条上片的数量
	//	{
	//		float slice_point_2_line_distance_count = 0;
	//		for (int k = 0;k<slice_vector.at(i).at(j).pointcloud_strip_slice->size();k++)
	//		{
	//			Eigen::Vector4f point;
	//			point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //圆柱轴线上一点
	//			Eigen::Vector4f normal;
	//			normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //圆柱轴线方向向量
	//			Eigen::Vector4f point_cloud_point;
	//			point_cloud_point << slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y,
	//				slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z;
	//			float point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// 空间点到空间直线距离，参数分别为点云中一点，直线上一点，直线的法向量
	//			slice_point_2_line_distance_count = slice_point_2_line_distance_count + point_2_line_distance;
	//		}
	//		float slice_2_line_distance = slice_point_2_line_distance_count / slice_vector.at(i).at(j).pointcloud_strip_slice->size();
	//		Mapping_2D_3D mapping;
	//		mapping.x = i;
	//		mapping.y = j;
	//		mapping.pixel_value = slice_2_line_distance;
	//		mapping_2D_3D.push_back(mapping);	
	//		float sitance_average = slice_2_line_distance - cylinder.radius;  //块元到圆柱轴线距离减去圆柱半径
	//		slice_distance.push_back(sitance_average);
	//		pointcloud_img.at<float>(i, j) = sitance_average;
	//	}
	//}
	//sort(slice_distance.begin(), slice_distance.end());

	/*计算图像中灰度分布直方图*/
	/*直方图*/
	//设置提取直方图的相关变量
	//Mat hist;  //用于存放直方图计算结果
	//const int channels[1] = { 0 };  //通道索引
	//float inRanges[2] = { 0,255 };
	//const float* ranges[1] = { inRanges };  //像素灰度值范围
	//const int bins[1] = { 256 };  //直方图的维度，其实就是像素灰度值的最大值
	//calcHist(&HSV_img_h, 1, channels, Mat(), hist, 1, bins, ranges);  //计算图像直方图
	////准备绘制直方图
	//int hist_w = 512;
	//int hist_h = 400;
	//int width = 2;
	//Mat histImage = Mat::zeros(hist_h, hist_w, CV_8UC3);
	//for (int i = 1; i <= hist.rows; i++)
	//{
	//	rectangle(histImage, Point(width*(i - 1), hist_h - 1),
	//		Point(width*i - 1, hist_h - cvRound(hist.at<float>(i - 1) / 15)),
	//		Scalar(255, 255, 255), -1);
	//}

	cout << "主绝缘表面凹陷凸起缺陷检测与量化任务完成！" << endl;


	return true;
}
/*点云添加高斯噪声*/
//bool add_gauss_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)
//{
//	cout << "******添加高斯噪声！******" << endl;
//	cout << "添加结束点云尺寸：" << pointcloud_filtered->size() << endl;
//	showpointcloud_xyzrgb(pointcloud_filtered);
//	// 设置XYZ各纬度的均值和标准差
//	float xmean = 0, ymean = 0, zmean = 0;
//	float xstddev = 0.1, ystddev = 0.1, zstddev = 0.1;
//	// ---------------------------生成高斯分布的点云数据---------------------------------------
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::NormalGenerator<float> > generator;
//	uint32_t seed = static_cast<uint32_t> (time(NULL));
//	pcl::common::NormalGenerator<float>::Parameters x_params(xmean, xstddev, seed++);
//	generator.setParametersForX(x_params);
//	pcl::common::NormalGenerator<float>::Parameters y_params(ymean, ystddev, seed++);
//	generator.setParametersForY(y_params);
//	pcl::common::NormalGenerator<float>::Parameters z_params(zmean, zstddev, seed++);
//	generator.setParametersForZ(z_params);
//	generator.fill((*pointcloud_filtered).width, (*pointcloud_filtered).height, *gauss_cloud);
//	// ---------------------------添加高斯分布的随机噪声--------------------------------------
//	for (size_t i = 0; i < pointcloud_filtered->points.size(); ++i)
//	{
//		gauss_cloud->points[i].x += pointcloud_filtered->points[i].x;
//		gauss_cloud->points[i].y += pointcloud_filtered->points[i].y;
//		gauss_cloud->points[i].z += pointcloud_filtered->points[i].z;
//	}
//
//
//	printf("高斯噪声添加完毕！！！");
//	cout << "添加结束点云尺寸：" << pointcloud_filtered->size() <<endl;
//	showpointcloud_xyzrgb(gauss_cloud);
//	return true;
//}
/*最小二乘平面拟合*/
bool least_square_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pointCloud, float &A, float &B, float &C, float &D)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Vector4d centroid;                    // 质心
	Eigen::Matrix3d covariance_matrix;           // 协方差矩阵
												 // 计算归一化协方差矩阵和质心
	pcl::computeMeanAndCovarianceMatrix(*plane_pointCloud, covariance_matrix, centroid);
	// 计算协方差矩阵的特征值与特征向量
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);
	// 查找最小特征值的位置
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);
	// 获取平面方程：AX+BY+CZ+D = 0的系数
	Eigen::Vector3d normal = eigenVectors.col(minCol);
	D = -normal.dot(centroid.head<3>());
	A = normal[0];
	B = normal[1];
	C = normal[2];

	return true;
}
/*RANSAC平面拟合*/
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D)
{
	A = B = C = D = 0;
	//对每一个小片进行平面拟合
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//创建一个模型参数对象，用于记录结果			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers表示误差能容忍的点 记录的是点云的序号			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// 创建一个分割器			
	seg.setOptimizeCoefficients(true);	// Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。			
	seg.setModelType(pcl::SACMODEL_PLANE);	// Mandatory-设置目标几何形状，需要拟合的形状		
	seg.setMethodType(pcl::SAC_RANSAC);	//分割方法：随机采样法			
	seg.setDistanceThreshold(0.1);	//设置误差容忍范围，也就是阈值  同拟合平面的距离超过阈值的点，就被判定为无效数据。
	seg.setInputCloud(strip_one_slice);	//输入点云			
	seg.segment(*inliers, *coefficients);	//分割点云，获得平面和法向量 coefficients中存储的是平面模型的系数A/B/C/D，inliers存储拟合出平面的点,可以描述模型的数据

	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];
	return 0;
}
/*RANSAC平面拟合函数重载,基于分割*/
//bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice,float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point)
//{
//	A = B = C = D = 0;
//	//对每一个小片进行平面拟合
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//创建一个模型参数对象，用于记录结果			
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers表示误差能容忍的点 记录的是点云的序号			
//	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// 创建一个分割器			
//	seg.setOptimizeCoefficients(true);	// Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。			
//	seg.setModelType(pcl::SACMODEL_PLANE);	// Mandatory-设置目标几何形状，需要拟合的形状		
//	seg.setMethodType(pcl::SAC_RANSAC);	//分割方法：随机采样法			
//	seg.setDistanceThreshold(0.1);	//设置误差容忍范围，也就是阈值  同拟合平面的距离超过阈值的点，就被判定为无效数据。
//	seg.setInputCloud(strip_one_slice);	//输入点云			
//	seg.segment(*inliers, *coefficients);	//分割点云，获得平面和法向量 coefficients中存储的是平面模型的系数A/B/C/D，inliers存储拟合出平面的点,可以描述模型的数据
//
//	A = coefficients->values[0];
//	B = coefficients->values[1];
//	C = coefficients->values[2];
//	D = coefficients->values[3];
//
//	//取出内点
//	pcl::copyPointCloud(*strip_one_slice, inliers->indices, *interior_point);
//
//	return 0;
//}
/*RANSAC平面拟合函数重载，基于拟合*/
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point)
{
	interior_point->clear();
	A = B = C = D = 0;
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(strip_one_slice));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);//定义RANSAC算法模型
	ransac.setDistanceThreshold(0.01);//设定距离阈值
	ransac.setMaxIterations(1000);     //设置最大迭代次数
	ransac.setProbability(0.99);      //设置从离群值中选择至少一个样本的期望概率
	ransac.computeModel();            //拟合平面
	vector<int> inliers;              //用于存放内点索引的vector
	ransac.getInliers(inliers);       //获取内点索引

	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);  //获取拟合平面参数，coeff分别按顺序保存a,b,c,d

	A = coeff[0];
	B = coeff[1];
	C = coeff[2];
	D = coeff[3];

	//取出内点
	pcl::copyPointCloud(*strip_one_slice, inliers, *interior_point);
	return true;
}
/*法线估计*/
bool show_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice, pcl::PointCloud <pcl::Normal>::Ptr normals)
{

	cout << "法线显示函数被调用!" << endl;
	pcl::visualization::PCLVisualizer viewer("show_normals");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(slice, "slice");
	//显示当前小片的法线，其中：slice表示原始点云模型，normals表示法向信息，1表示需要显示法线的点云间隔（这里表示每个点都显示），0.5表示法向长度
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(slice, 255, 0, 0);  //设置点云颜色
	viewer.addPointCloud<pcl::PointXYZRGB>(slice, single_color, "sample cloud");  //添加坐标系
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(slice, normals, 5, 2.5, "normals");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");  //修改点云大小
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return true;
}
/*加载点云*/
bool loadPointcloud(std::string strFilename, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src)
{
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(strFilename, *pointcloud_src) == -1)
	{
		std::cout << "Loading .ply file failed!" << std::endl;
		return false;
	}
	else
	{
		std::cout << endl;
		std::cout << "接头点云加载成功!" << std::endl;
	}
	return true;
}
/*zyz 2 xyzrgb*/
bool xyz2rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb)
{
	pcl::PointXYZRGB pointtemp;
	for (int i = 0; i < pointcloud_src->size(); i++)
	{
		pointtemp.x = pointcloud_src->at(i).x;
		pointtemp.y = pointcloud_src->at(i).y;
		pointtemp.z = pointcloud_src->at(i).z;
		pointtemp.r = 255;
		pointtemp.g = 255;
		pointtemp.b = 255;
		pointcloud_rgb->push_back(pointtemp);
	}
	return true;
}
/*显示xyzrgbnormal类型的点云*/
bool showpointcloud_xyzrgbnormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud_in)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(pointcloud_in, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
	return true;
}
/*显示xyzrgbnor类型的点云*/
bool showpointcloud_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud_in, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
	return true;
}
/*显示xyz类型的点云*/
bool showpointcloud_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(pointcloud_in, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
	return true;
}
/*显示xyzhsv类型的点云*/
bool showpointcloud_xyzhsv(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pointcloud_in)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	//viewer->addPointCloud<pcl::PointXYZHSV>(pointcloud_in, "sample cloud");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	//}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerHSVField<pcl::PointXYZHSV> hsv(pointcloud_in);
	viewer->addPointCloud<pcl::PointXYZHSV>(pointcloud_in, hsv, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
	return true;
}
/*ransic拟合圆执行函数*/
bool ransice_circular_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Circle &circle)
{
	//cout << "->正在估计空间圆..." << endl;
	circle.m_center_x = circle.m_center_y = circle.m_center_z = circle.m_radius = 0;
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>(cloud_ptr));	//选择拟合点云与几何模型
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3D);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(0.1);	//设置距离阈值，与模型距离小于0.1的点作为内点
	ransac.setMaxIterations(15000);		//设置最大迭代次数
	ransac.computeModel();				//执行模型估计
	std::vector<int> inliers;			//存储内点索引的向量
	ransac.getInliers(inliers);			//提取内点对应的索引

										// 根据索引提取内点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_ptr, inliers, *cloud_circle);
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	circle.m_center_x = coefficient[0];  //为了返回给调用函数处的圆心坐标与半径，引用传参
	circle.m_center_y = coefficient[1];
	circle.m_center_z = coefficient[2];
	circle.m_radius = coefficient[3];

	/*对比展示参与拟合的范围，与拟合结果*/

	//cout << "空间圆心："
	//	<< "(" << coefficient[0] << ","
	//	<< coefficient[1] << ","
	//	<< coefficient[2] << ")"
	//	<< endl;
	//cout << "半径：" << coefficient[3] << endl;
	//cout << "空间圆法向量："
	//	<< "(" << coefficient[4] << ","
	//	<< coefficient[5] << ","
	//	<< coefficient[6] << ")"
	//	<< endl;

	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, "cloud");											//添加原始点云
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	//颜色
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");	//点的大小

	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_circle, "circle");										//添加模型点云,拟合结果
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "circle");	//颜色
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "circle");	//点的大小
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	return true;
}
/*对输入点云进行滤波及按z轴排序*/
bool Cable::filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_filtered)		//引用传参
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;									//创建半径滤波器;
	outrem.setInputCloud(m_pointcloud_rgb);												//设置输入点云;
	std::cout << "输入接头点云点数：" << m_pointcloud_rgb->size() << std::endl;			//滤波前点云数量
	outrem.setRadiusSearch(1);															//设置半径为1的范围内找临近点;
	outrem.setMinNeighborsInRadius(3);													//设置查询点的邻域点集数小于3的删除;
	outrem.filter(*pointcloud_filtered);												//执行条件滤波，结果存储在pointcloud_filtered中
																						//cout << "排序前当前电缆z最大值 = " << pointcloud_filtered->at(pointcloud_filtered->size() - 1).z << "当前电缆z最小值 = " << pointcloud_filtered->at(0).z << endl;
	rankpointcloud_z(pointcloud_filtered);		   //对电缆按z值排序
	cout << "输入接头点云z轴最大值 = " << pointcloud_filtered->at(pointcloud_filtered->size() - 1).z << ",  " << "输入接头点云z轴最小值 = " << pointcloud_filtered->at(0).z << endl;
	//std::cout << "滤波后点云数量：" << pointcloud_filtered->size() << std::endl;		//滤波后点云数量输出
	//showpointcloud_xyzrgb(pointcloud_filtered);
	return true;
}
/*按z值排序算法*/
bool Cable::rankpointcloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank)
{
	int i;
	for (i = pointcloud_rank->size() / 2 - 1; i >= 0; --i)
	{
		heapify_z(pointcloud_rank, i, pointcloud_rank->size() - 1);  //排序
	}
	for (i = pointcloud_rank->size() - 1; i > 0; --i)
	{
		pcl::PointXYZRGB pointtemp = pointcloud_rank->at(i);
		pointcloud_rank->at(i) = pointcloud_rank->at(0);
		pointcloud_rank->at(0) = pointtemp;

		heapify_z(pointcloud_rank, 0, i);
	}
	return true;
}
/*z值排序算法的子函数*/
bool Cable::heapify_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank, int n_first, int n_end)
{
	int n_father = n_first;
	int n_son = n_father * 2 + 1;
	while (n_son < n_end)
	{
		if (n_son + 1 < n_end&&pointcloud_rank->at(n_son).z < pointcloud_rank->at(n_son + 1).z)
			++n_son;
		if (pointcloud_rank->at(n_father).z > pointcloud_rank->at(n_son).z)
			break;
		else
		{
			pcl::PointXYZRGB pointtemp = pointcloud_rank->at(n_father);
			pointcloud_rank->at(n_father) = pointcloud_rank->at(n_son);
			pointcloud_rank->at(n_son) = pointtemp;
			n_father = n_son;
			n_son = n_father * 2 + 1;
		}
	}
	return true;
}
/*对点云按固定z值进行圆拟合*/
bool Cable::z_value_circle_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double distance)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr once_circle_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	rankpointcloud_z(pointcloud_filtered);		//对滤波后的点云按z轴进行排序,确保现在点云文件里面点云一定是按z值大小进行索取的
	if (m_indexandcircles.empty())				//判断m_indexandcircles是否为空，存储圆索引的全局变量，如果为空就执行圆拟合的操作
	{
		if (m_pointcloud_rgb->empty())			//判断输入的点云是否为空
		{
			std::cout << "point cloud is empty!" << std::endl;
			return false;
		}

		pcl::PointXYZRGB point;
		pcl::PointXYZRGB point2;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_fit(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int> index;								//储存截面圆点的索引

		point = pointcloud_filtered->at(0);					//点云文件排序后的第一个点
		double z_min = pointcloud_filtered->at(0).z;		//点云文件排序后的的z轴最低点的z值
		double z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;		//排序后点云的最高点的z值
																						//cout <<"圆拟合开始： "<< "cable_size_min: " << z_min << "," << " cable_size_max：" << z_max << endl;
		cout << endl;
		cout << "正在进行接头点云位姿校正！ " << endl;


		m_indexandcircles.clear();
		//先加(distance / 2)是为了避免圆的厚度只有其它地方圆的一半
		for (double k = z_min + (distance / 2); k < z_max; k = k + distance)    //拟合圆，先找出点，然后拟合圆、存储拟合圆的索引、存储拟合圆上所有点云点的索引，每个圆是1mm
		{
			//直接指定xy，变换z值，从而找出不同z高度的点云
			point2.x = point.x;
			point2.y = point.y;
			point2.z = k;

			index.clear();         //每次循环初始化
			pointcloud_fit->clear();
			once_circle_points->clear();
			for (int t = 0; t < pointcloud_filtered->size(); t++)  //一个for循环会找出一定范围的点进行存储，后面用来拟合圆
			{
				//point2.z是种子点通过临近搜索而找出来的临近点，计算滤波后点云文件中与找出来的临近点z值小于一定范围的点
				double theta = pointcloud_filtered->at(t).z - point2.z;
				if (theta < distance / 2 && theta > -distance / 2)
				{
					pointcloud_fit->push_back(pointcloud_filtered->at(t));  //pointcloud_fit存储种子点高度一定范围内的点的三维信息
					index.push_back(t);			//index存储种子点高度一定范围内点的索引（处于容器（滤波后的点云文件）哪个位置，上面是存储的点的内容（三维空间位置），这是存储的点处于容器的位置）
				}
			}
			if (pointcloud_fit->size())  //用找出来的点拟合圆
			{
				once_circle_points = pointcloud_fit->makeShared();

				Circle circle;   //结构体圆，包括圆心（x，y，z）和半径
				ransice_circular_fitting(pointcloud_fit, circle);		//临近点拟合圆
				IndexofCircle indexofcircle;		//结构体，包含一个结构体圆心，整型容器（存储拟合圆上所有点云点的索引）
				indexofcircle.once_circle = circle;	//circleleastfit拟合的圆赋值给结构体indexofcircle里面的圆
				indexofcircle.each_circle_index = index;		//每一个圆里面点云点的索引（索引是针对滤波后的整个点云文件来说的，范围是整个滤波后的点云）
				indexofcircle.each_circle_points = once_circle_points->makeShared();
				//m_indexandcircles全局变量存储的是每个拟合圆的结构体信息（圆（圆心、半径），每个拟合圆上点云构成的vector容器）储存，每一个循环存储一个拟合圆的结构体信息
				m_indexandcircles.push_back(indexofcircle);		//m_indexandcircles存储的是每个拟合圆的结构体信息！！！
			}
			//std::cout << "圆拟合完成：" << k / z_max * 100 << "%";    //进度条
			printf("\r");
		}


	}
	/*显示一个圆，便于查看圆的情况*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pointcloud_temp->clear();
	//for (int i = 0; i < m_indexandcircles.at(0).each_circle_index.size();i++)
	//{
	//	pointcloud_temp->push_back(pointcloud_filtered->at(i));
	//}
	//showpointcloud_xyzrgb(pointcloud_temp);
	//cout << endl;
	//cout << "拟合圆的个数 = " << m_indexandcircles.size() << endl;
	cout << "接头点云位姿校正完成！ " << endl;
	return true;

}
/*临近圆半径标准差计算*/
bool Cable::calculation_standard_deviation(int index, int range, double &standard_fore, double &standard_back)
{
	double fore_sum, back_Sum;
	double average_value_fore, average_value_back;
	double variance_temp_fore, variance_temp_back;
	double variance_fore, variance_back;

	fore_sum = back_Sum = average_value_fore = average_value_back = standard_fore = standard_back
		= variance_temp_fore = variance_temp_back = variance_fore = variance_back = 0;

	for (int i = 1; i <= range; i++)    //计算均值
	{
		fore_sum = fore_sum + m_indexandcircles.at(index - i).once_circle.m_radius;
		back_Sum = back_Sum + m_indexandcircles.at(index + i).once_circle.m_radius;
	}
	average_value_fore = fore_sum / range;
	average_value_back = back_Sum / range;

	for (int j = 1; j <= range; j++)  //计算方差的分子
	{
		variance_temp_fore = variance_temp_fore + pow(((m_indexandcircles.at(index - j).once_circle.m_radius) - average_value_fore), 2);
		variance_temp_back = variance_temp_back + pow(((m_indexandcircles.at(index + j).once_circle.m_radius) - average_value_back), 2);
	}
	variance_fore = variance_temp_fore / range;    //方差
	variance_back = variance_temp_back / range;

	standard_fore = sqrt(variance_fore);
	standard_back = sqrt(variance_back);
	//standard_fore = variance_fore;
	//standard_back = variance_back;
	return true;
}
/*标准差区域分割，6_21之前版本代码*/
/*bool Cable::rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end)*/
//{
//	/*给拟合圆上不同的颜色，方便观察*/
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_circle_colour(new pcl::PointCloud<pcl::PointXYZRGB>);    //rgb滤波后的点云
//	//for (int i = 0;i < m_indexandcircles.size(); i++)
//	//{
//	//	int b, g, r; b = g = r = 0;
//	//	random_colour(b,g,r);
//	//	for (int j = 0; j<m_indexandcircles.at(i).each_circle_points->size();j++ )
//	//	{
//	//		m_indexandcircles.at(i).each_circle_points->at(j).b = b;
//	//		m_indexandcircles.at(i).each_circle_points->at(j).g = g;
//	//		m_indexandcircles.at(i).each_circle_points->at(j).r = r;
//	//		pointcloud_circle_colour->push_back(m_indexandcircles.at(i).each_circle_points->at(j));
//	//	}
//	//}
//	//showpointcloud_xyzrgb(pointcloud_circle_colour);
//	
//	/*半径数据写出*/
//	//FILE * fp5;
//	//if ((fp5 = fopen("circle_radius_0.5_standard.txt", "wb")) == NULL)
//	//{
//	//	printf("cant open the file");
//	//	exit(0);
//	//}
//	//for (size_t i = 0; i < m_indexandcircles.size(); i++)
//	//{
//	//	fprintf(fp5, "%f ", m_indexandcircles.at(i).once_circle.m_radius);
//	//	fprintf(fp5, "%s ", "\n");
//	//}
//	//fclose(fp5);
//	//cout << "半径输出完成!" << endl;
//	
//	std::cout << "正在进行区域分割!" << endl;
//	region_start.clear();
//	region_end.clear();
//	//求电缆的最低点，只求一个点不准确，所以求最后m个点并求平均，不用单独一个点来确定的原因是因为存在扫描误差
//	int cable_min_point_number = 20;
//	double sum_cable_min_point = 0;
//	for (int i = 1; i <= cable_min_point_number; i++)
//	{
//		sum_cable_min_point = sum_cable_min_point + pointcloud_filtered->at(i).z;
//	}
//	double cable_z_mini_value = sum_cable_min_point / cable_min_point_number;  //线芯的最低点，即线芯起点
//
//	//求电缆的最高点，只求一个点不准确，所以求最后m个点并求平均，不用最后一个拟合圆的圆心原因是因为存在扫描误差
//	int cable_max_point_number = 20;
//	double sum_cable_max_point = 0;
//	for (int i = 1; i <= cable_max_point_number; i++)
//	{
//		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
//	}
//	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //线芯的最高点，即线芯终点
//
//
//	double standard_fore, standard_back;    //圆半径的标准差
//	double standard_back_compare_fore, standard_fore_compare_back;
//	std::vector<double> back_compare_fore_all;
//	std::vector<double> fore_compare_back_all;
//	back_compare_fore_all.clear();
//	fore_compare_back_all.clear();
//
//	/*标准差计算，具体需要方差还是标准差看自己需求*/
//	int k = 15; //每个点取前后20个圆计算标准差，然后计算标准差之比
//	for (int i = 0; i < k; i++)  //为了与拟合圆数量相等，前面填k个0
//	{
//		back_compare_fore_all.push_back(0);
//		fore_compare_back_all.push_back(0);
//	}
//
//	for (int i = k; i < m_indexandcircles.size() - k; i++)
//	{
//		standard_fore = standard_back = standard_back_compare_fore = standard_fore_compare_back = 0;
//		calculation_standard_deviation(i, k, standard_fore, standard_back);    //参数i代表计算第i个拟合圆
//
//		standard_back_compare_fore = standard_back / standard_fore;
//		standard_fore_compare_back = standard_fore / standard_back;
//		back_compare_fore_all.push_back(standard_back_compare_fore);
//		fore_compare_back_all.push_back(standard_fore_compare_back);
//	}
//	for (int i = 0; i < k; i++)	//为了与拟合圆数量相等，后面填k个0
//	{
//		back_compare_fore_all.push_back(0);
//		fore_compare_back_all.push_back(0);
//	}
//
//	/*半径标准差数据写出*/
//	//FILE * fp6;
//	//if ((fp6 = fopen("back_compare_fore_15.txt", "wb")) == NULL)
//	//{
//	//	printf("cant open the file");
//	//	exit(0);
//	//}
//	//for (size_t i = 0; i < back_compare_fore_all.size(); i++)
//	//{
//	//	fprintf(fp6, "%f ", back_compare_fore_all.at(i));
//	//	fprintf(fp6, "%s ", "\n");
//	//}
//	//fclose(fp6);
//	//cout << "标准差比值后比前输出完成!" << endl;
//
//	/*半径标准差数据写出*/
//	//FILE * fp7;
//	//if ((fp7 = fopen("fore_compare_back_15.txt", "wb")) == NULL)
//	//{
//	//	printf("cant open the file");
//	//	exit(0);
//	//}
//	//for (size_t i = 0; i < fore_compare_back_all.size(); i++)
//	//{
//	//	fprintf(fp7, "%f ", fore_compare_back_all.at(i));
//	//	fprintf(fp7, "%s ", "\n");
//	//}
//	//fclose(fp7);
//	//cout << "标准差比值后比前输出完成!" << endl;
//
//
//	std::vector<Circle> region_circles; //用来对每个起点或终点进行精分割迭代返回新拟合圆的，每次调用前记得初始化
//	/**************************---找区域起点---****************************/
//	/**************************---找区域起点---****************************/
//	/**************************---找区域起点---****************************/
//	
//	/*^^^^^^^^^^^^^^^^^^^^^^^外半导电过渡带起点^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	//std::vector<double> back_compare_fore_1;
//	//back_compare_fore_1.clear();
//	//std::vector<int> start1_index_all;  start1_index_all.clear();
//
//	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
//	//{
//	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 370) &&
//	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 310))
//	//	{
//	//		start1_index_all.push_back(i);
//	//	}
//	//}
//	//if (!(start1_index_all.size()))
//	//{
//	//	cout << "外半导电过渡带起点区间提取异常!" << endl;
//	//}
//
//	///***********************利用600-680mm最大圆半径的方式找外半导电过渡带起点***********************************/
//	//std::vector<int> cable_circle_index_size;   //所有拟合圆索引
//	//cable_circle_index_size.clear();
//	//for (int i = 0; i < m_indexandcircles.size(); i++)
//	//{
//	//	cable_circle_index_size.push_back(i);
//	//}
//	//std::vector<int> Outer_semiconducting_circle_index;
//	//Outer_semiconducting_circle_index.clear();
//	//std::vector<int>::const_iterator region_index_start = cable_circle_index_size.begin() + start1_index_all.at(0);
//	//std::vector<int>::const_iterator region_index_end = cable_circle_index_size.begin() + start1_index_all.at(start1_index_all.size() - 1);
//	//Outer_semiconducting_circle_index.assign(region_index_start, region_index_end);   //把处于距离顶端600-680mm圆的索引拿出来存储
//
//	///*标准差法找出外半导电过渡带起点*/
//	////sort(start_1_start.begin(), start_1_start.end());
//	//std::vector<double>::const_iterator first_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(0);
//	//std::vector<double>::const_iterator second_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(start1_index_all.size() - 1);
//	//back_compare_fore_1.assign(first_back_2_fore_1, second_back_2_fore_1);    //把处于该范围内的标准差比值截取出来
//	//double goal_1 = back_compare_fore_1.at(0);
//	//int flag = 0;
//	//for (int i = 0; i < back_compare_fore_1.size(); i++)					  //找出该范围内标准差比值最大的标号
//	//{
//	//	if (goal_1 < back_compare_fore_1.at(i))
//	//	{
//	//		goal_1 = back_compare_fore_1.at(i);
//	//		flag = i;
//	//	}
//	//}
//
//	//int first_start_index = start1_index_all.at(flag + 1);
//	//printf("%s", "\n");
//	//double first_start_z_value = m_indexandcircles.at(first_start_index).once_circle.m_center_z;    //外半导电过渡带起点
//
//	/*直接根据距离指定外半导电过渡带的起点*/
//	
//	double first_start_z_value = cable_z_max_value - 340 + ( 2  / (rand() % 6));    //外半导电过渡带起点
//
//	/*外半导电过渡带起点标准差法迭代*/
//	//region_circles.clear();
//	//fusion_iteration_detail_segmentation(first_start, 20/circle_distance, 1, region_circles);  //取出该起点的前1cm重新以更小的z值进行圆拟合，重新进行前后标准差之比计算，重新找到极大值			
//	//double first_start_updatez_z_value = 0;
//	//update_segmentation_start(region_circles, k/2, cable_z_mini_value, first_start_updatez_z_value);
//	//double  first_start_update = first_start_updatez_z_value;
//	//cout << "update外半导电过渡带起点z值距离线缆底端距离 = " << first_start_updatez_z_value << " mm" << endl;
//	//double first_start_z_value = first_start_updatez_z_value + cable_z_mini_value;
//	//printf("%s", "\n");
//
//
//	/*外半导电过渡带贴胶带，最大半径差法找起点*/
//	//double out_semiconduct_max_radiues_index = 0;
//	//double out_semiconduct_max_radius = m_indexandcircles.at(start1_index_all.at(0)).once_circle.m_radius - m_indexandcircles.at(start1_index_all.at(1)).once_circle.m_radius;
//	//for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
//	//{
//	//	if (m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 1).once_circle.m_radius	> out_semiconduct_max_radius)
//	//	{
//	//		out_semiconduct_max_radius = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 1).once_circle.m_radius;
//	//		out_semiconduct_max_radiues_index = i;
//	//	}
//	//}
//	//int first_start_radius = out_semiconduct_max_radiues_index + 1;
//	//double first_start_z_value = m_indexandcircles.at(first_start_radius).once_circle.m_center_z;
//
//
//	/*利用物理厚度来找外半导电过渡带起点*/
//	//double core_radius_parameter = 21;
//	//std::vector<int> outer_semi_conduct_start; outer_semi_conduct_start.clear(); //根据物理厚度找出外半导电终点
//	//for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
//	//{
//	//	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius_parameter > 18.9)
//	//	{
//	//		outer_semi_conduct_start.push_back(i);
//	//	}
//	//}
//	//if (!outer_semi_conduct_start.size())
//	//{
//	//	cout << "利用物理厚度来找外半导电过渡带起点异常" << endl;
//	//}
//	//sort(outer_semi_conduct_start.begin(), outer_semi_conduct_start.end());
//	//int outer_semi_conduct_start_max_index = outer_semi_conduct_start.at(outer_semi_conduct_start.size() - 1) + 1;
//	//double first_start_z_value = m_indexandcircles.at(outer_semi_conduct_start_max_index).once_circle.m_center_z;
//
//
//	/*^^^^^^^^^^^^^^^^^^^^^^^反应力锥起点^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	std::vector<double> back_compare_fore_3;
//	back_compare_fore_3.clear();
//	std::vector<int>start3_index_all;
//	start3_index_all.clear();
//	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
//	{
//		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 105 &&
//			cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 85) //把距离线芯85-105mm的圆的索引找出来
//		{
//			start3_index_all.push_back(i);
//		}
//	}
//	if (!(start3_index_all.size()))
//	{
//		cout << "反应力锥起点区间提取异常；" << endl;
//	}
//	std::vector<double>::const_iterator first_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(0);
//	std::vector<double>::const_iterator second_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(start3_index_all.size() - 1);
//	back_compare_fore_3.assign(first_back_2_fore_3, second_back_2_fore_3);
//
//	double goal_3 = back_compare_fore_3.at(0);  //找出55-135mm里back：fore里面的最大值
//	int flag_3 = 0;
//	for (int i = 0; i < back_compare_fore_3.size(); i++)
//	{
//		if (goal_3 < back_compare_fore_3.at(i))
//		{
//			goal_3 = back_compare_fore_3.at(i);
//			flag_3 = i;
//		}
//	}
//	int second_start_index = start3_index_all.at(flag_3 + 1);
//	double second_start_z_value = m_indexandcircles.at(second_start_index).once_circle.m_center_z ;
//
//	/*优化迭代*/
//	//region_circles.clear();  //每次使用记得初始化，否者后面结果就是有问题的
//	//fusion_iteration_detail_segmentation(third_start, 10 / circle_distance, 2, region_circles);  //取出该起点的前1cm重新进行圆拟合，重新进行前后标准差之比计算，重新找到极大值			
//	//double thrid_start_updatez_z_value = 0;
//	//update_segmentation_start(region_circles, k / 3, cable_z_mini_value, thrid_start_updatez_z_value);
//	//double thrid_start_update = thrid_start_updatez_z_value;
//	//printf("%s", "\n");
//
//
//
//	/**************************---找区域终点---****************************/
//	/**************************---找区域终点---****************************/
//	/**************************---找区域终点---****************************/
//
//	/*^^^^^^^^^^^^^^^^^^^^^^^外半导电过渡带终点^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	//std::vector<double> fore_compare_back_1;
//	//fore_compare_back_1.clear();
//	//std::vector<int> region_circle_index_end_1;		region_circle_index_end_1.clear();
//	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
//	//{
//	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 310) &&
//	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 290))
//	//	{
//	//		region_circle_index_end_1.push_back(i);
//	//	}
//	//}
//	//if (!(region_circle_index_end_1.size()))
//	//{
//	//	cout << "外半导电过渡带终点区间提取异常；" << endl;
//	//}
//	//std::vector<double>::const_iterator first_fore_2_back_1 = fore_compare_back_all.begin() + region_circle_index_end_1.at(0);
//	//std::vector<double>::const_iterator second_fore_2_back_1 = fore_compare_back_all.begin() + region_circle_index_end_1.at(region_circle_index_end_1.size() - 1);
//	//fore_compare_back_1.assign(first_fore_2_back_1, second_fore_2_back_1);
//
//	//double goal_back = fore_compare_back_1.at(0);
//	//int flag_back = 0;
//	//for (int i = 0; i < fore_compare_back_1.size(); i++)
//	//{
//	//	if (goal_back < fore_compare_back_1.at(i))
//	//	{
//	//		goal_back = fore_compare_back_1.at(i);
//	//		flag_back = i;
//	//	}
//	//}
//	////int first_end = flag_back + m_indexandcircles.size() - end_1_start.size();
//	//int first_end_index = region_circle_index_end_1.at(flag_back + 1);
//	//double first_end_z_value = m_indexandcircles.at(first_end_index).once_circle.m_center_z;
//
//	double first_end_z_value = cable_z_max_value - 300 + (2 / (rand() % 6));
//
//
//	//double first_end_update = first_end_distance - 1;
//
//	/*根据外半导电距离线芯的厚度设置参数对外办导电过渡带终点进行求取*/
//	//double core_radius = 21; //该电缆的线芯半径
//	//std::vector<int> outer_semi_conduct_end; outer_semi_conduct_end.clear(); //根据物理厚度找出外半导电终点
//	//for (int i = region_circle_index_end_1.at(0); i < region_circle_index_end_1.at(region_circle_index_end_1.size() - 1); i++)
//	//{
//	//	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius >= 17.58
//	//		&& m_indexandcircles.at(i).once_circle.m_radius - core_radius < 18)
//	//	{
//	//		outer_semi_conduct_end.push_back(i);
//	//	}
//	//}
//	//double first_end_update = 0;
//	//if (outer_semi_conduct_end.size())
//	//{
//	//	sort(outer_semi_conduct_end.begin(), outer_semi_conduct_end.end());
//	//	int outer_semi_conduct_max_index = outer_semi_conduct_end.at(outer_semi_conduct_end.size() - 1) - 1;
//	//	first_end_update = m_indexandcircles.at(outer_semi_conduct_max_index).once_circle.m_center_z;
//	//}
//	//else
//	//{
//	//	cout << "外半导电过渡带终点出现问题！" << endl;
//	//	return 0;
//	//}
//
//
//		/*^^^^^^^^^^^^^^^^^^^^^^^内半导电与线芯交界^^^^^^^^^^^^^^^^^^^^^^^^^*/
//		 //在该物理距离内，这两个相邻圆的半径差是最大的
//		std::vector<IndexofCircle> fore_compare_back_3;
//		fore_compare_back_3.clear();
//
//		std::vector<int> end_3_all_index;	end_3_all_index.clear();//20-60
//
//		for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
//		{
//
//			if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 25) &&
//				(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 40))
//			{
//				end_3_all_index.push_back(i);  //把满足条件的圆的索引存储起来
//			}
//
//		}
//
//		if (!(end_3_all_index.size()))
//		{
//			cout << "内半导电与线芯交界点提取异常；" << endl;
//		}
//		std::vector<IndexofCircle>::const_iterator first_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(0);
//		std::vector<IndexofCircle>::const_iterator second_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(end_3_all_index.size() - 1);
//		fore_compare_back_3.assign(first_fore_2_back_3, second_fore_2_back_3);
//
//		//double adjacent_radius_distance = fore_compare_back_3.at(0).once_circle.m_radius - fore_compare_back_3.at(1).once_circle.m_radius; 
//		//int flag_back_3 = 0;
//		//for (int i = 0; i < fore_compare_back_3.size() - 1; i++)  //找出该物理范围内半径数值相差最大的地方
//		//{
//		//	//cout << fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius << " " << i <<endl;
//		//	if (fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius > adjacent_radius_distance)
//		//	{
//		//		adjacent_radius_distance = fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius;
//		//		flag_back_3 = i;
//		//	}
//		//}
//		//int thrid_end = flag_back_3 + m_indexandcircles.size() - end_3.size();  //这里求的是内半导电与线芯交界
//		//double thrid_end_disance = m_indexandcircles.at(thrid_end).once_circle.m_center_z - cable_z_mini_value; //距离线缆底端距离
//		//cout << "--内半导电与线芯交界点距离线缆底端距离 = " << thrid_end_disance << " mm" << endl;
//
//		double adjacent_radius_distance = 0;
//		int flag_back_3 = 0;
//		for (int i = end_3_all_index.at(0) ; i < end_3_all_index.at(end_3_all_index.size() - 1) - 2; i++)  //找出该物理范围内半径数值相差最大的地方
//		{
//
//			if ((m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius) > adjacent_radius_distance)
//			{
//				adjacent_radius_distance = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius;
//				flag_back_3 = i + 1;
//			}
//		}
//		int inner_semiconducting_end_index = flag_back_3 ;  //这里求的是内半导电与线芯交界点的索引，偏线芯处
//		double inner_semiconducting_end_z_value = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z ;   //内半导电与线芯的交界
//		
//
//		/*根据厚度找出内半导电起点*/
//		double cable_core_radius_average = 21;   //设定线芯区域半径为21mm
//		std::vector<int> inner_semiconducting_index;    //存储根据厚度找出来的内半导电圆的索引
//		inner_semiconducting_index.clear();
//
//		//for (int i = end_3_all_index.at(0); i<end_3_all_index.at(end_3_all_index.size() - 1); i++)
//		//{
//		//	if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
//		//		&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 1.95)    //大阈值控制内半导电起点的位置
//		//		&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //避免线芯松散导致出错
//		//	{
//		//		inner_semiconducting_index.push_back(i);
//		//	}
//		//}
//		for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1); i++)
//		{
//			if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
//				&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 2.2)    //大阈值控制内半导电起点的位置
//				&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //避免线芯松散导致出错
//			{
//				inner_semiconducting_index.push_back(i);
//			}
//		}
//
//		double inner_semiconducting_start_z_value_finally = 0;
//		if (inner_semiconducting_index.size())   //如果有打磨内半导电，则意味着该容器不为空
//		{
//			sort(inner_semiconducting_index.begin(), inner_semiconducting_index.end());   //对找出来的内半导电圆的索引进行排序，起点是i较小的那个地方
//			int inner_semiconducting_start_i = inner_semiconducting_index.at(0);  //内半导电起点圆的索引,找出内半导电与线芯的交点后，利用内半导电厚度找出来的
//			double inner_semiconducting_start_z_value = m_indexandcircles.at(inner_semiconducting_index.at(0)).once_circle.m_center_z;
//			inner_semiconducting_start_z_value_finally = inner_semiconducting_start_z_value;
//		}
//		else   //如果没有打磨内半导电，则内半导电的起点就是线芯与内板导电的交界点，也就是内半导电宽度为0
//		{
//			inner_semiconducting_start_z_value_finally = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;
//			std::cout << "内半导电起点与线芯起点重合！" << std::endl;
//		}
//
//		/*电缆分割起点存储*/
//		region_start.push_back(first_start_z_value);  //外半导电过渡带起点的z值
//		region_start.push_back(second_start_z_value);  //反应力锥起点的z值
//		region_start.push_back(inner_semiconducting_start_z_value_finally);  //内半导电的起点，也是第二个铅笔头终点的z值
//		region_start.push_back(inner_semiconducting_end_z_value);  //线芯起点的z值，也是内半导电终点z值
//		/*电缆分割终点存储*/
//		region_end.push_back(first_end_z_value);  //外半导电过渡带终点的z值,迭代一次
//		region_end.push_back(inner_semiconducting_start_z_value_finally);  //反应力锥终点的z值
//		region_end.push_back(inner_semiconducting_end_z_value);  //内半导电终点的z值
//		region_end.push_back(cable_z_max_value);  //线芯终点的z值	
//
//		/*对分割结果分段上色显示*/
//		for (int n = 0; n < pointcloud_filtered->size(); n++)
//		{
//			if (pointcloud_filtered->at(n).z < region_start.at(0))  //外半导电
//			{
//				pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(0) && pointcloud_filtered->at(n).z < region_end.at(0))  //外半导电过渡带
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 255;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_end.at(0) && pointcloud_filtered->at(n).z < region_start.at(1))  //第一个平滑区域
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(1) && pointcloud_filtered->at(n).z < region_end.at(1))  //铅笔头
//			{
//				pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
//			}
//			else if (pointcloud_filtered->at(n).z > region_end.at(1) && pointcloud_filtered->at(n).z < region_start.at(2))  //内半导电
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(3) && pointcloud_filtered->at(n).z < region_end.at(3))  //线芯
//			{
//				pointcloud_filtered->at(n).r = 198;			pointcloud_filtered->at(n).g = 145;			pointcloud_filtered->at(n).b = 69;
//			}
//		}
//
//		showpointcloud_xyzrgb(pointcloud_filtered);		//分段显示
//
//		std::vector<int> inner_semiconducting_circle_index; inner_semiconducting_circle_index.clear();
//		std::vector<int> XLPE_insulation_circle_index; XLPE_insulation_circle_index.clear();
//
//		for (int i = 0; i<m_indexandcircles.size();i++)
//		{
//			if (region_start.at(2)< m_indexandcircles.at(i).once_circle.m_center_z && 
//				m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(3))  //内半导电圆索引拿出来
//			{
//				inner_semiconducting_circle_index.push_back(i);
//			}
//
//			if (region_end.at(0) < m_indexandcircles.at(i).once_circle.m_center_z &&
//				m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(1))  //XLPE绝缘段圆索引拿出来
//			{
//				XLPE_insulation_circle_index.push_back(i);
//			}
//		}
//
//		std::vector<double> inner_semiconducting_circle_radius; inner_semiconducting_circle_radius.clear();
//		std::vector<double> XLPE_insulation_circle_radius; XLPE_insulation_circle_radius.clear();
//		for (int i = 2; i<inner_semiconducting_circle_index.size(); i++)
//		{
//			inner_semiconducting_circle_radius.push_back(m_indexandcircles.at(inner_semiconducting_circle_index.at(i)).once_circle.m_radius);  ////内半导电圆半径拿出来
//		}
//		sort(inner_semiconducting_circle_radius.begin(), inner_semiconducting_circle_radius.end());
//
//		for (int i = 0; i < XLPE_insulation_circle_index.size(); i++)
//		{
//			XLPE_insulation_circle_radius.push_back(m_indexandcircles.at(XLPE_insulation_circle_index.at(i)).once_circle.m_radius);		//XLPE绝缘段圆圆半径拿出来
//		}
//		sort(XLPE_insulation_circle_radius.begin(), XLPE_insulation_circle_radius.end());
//
//		/*计算反应力锥斜坡长度*/
//		double xiepo = pow((2 * (XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))) - (2 * (inner_semiconducting_circle_radius.at(0))), 2) +
//			pow((region_end.at(1) - region_start.at(1)), 2);
//		double Length_S = sqrt(xiepo);
//
//		cout << "\n" << endl;
//		cout << "导体长度：" << region_end.at(3) - region_start.at(3) << " mm" << endl;
//		cout << "内半导电层长度：" << region_start.at(3) - region_start.at(2) << endl;
//		cout << "内半导电层外径：" << 2 * inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
//		cout << "XLPE绝缘长度：" << region_start.at(1) - region_end.at(0) << " mm" << endl;
//		cout << "XLPE外径：" << 2 * XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2)) << " mm" << endl;
//		cout << "外半导电过渡带长度：" << region_end.at(0) - region_start.at(0) << " mm" << endl;
//		//cout << "反应力锥区域终点外径:Di:" <<2*(inner_semiconducting_circle_radius.at(0)) << endl;
//		//cout << "反应力锥区域起点绝缘外径:Do:" << 2*(XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))<< endl;
//		//cout << "反应力锥斜坡长度：S:" << Length_S << endl;
//		cout << "反应力锥轴向高度：H:" << region_end.at(1) - region_start.at(1) << " mm" << endl;
//		cout << "接头开线高度：" << cable_z_max_value - region_end.at(0) << " mm" << endl;
//
//		//return 0;
//
//		/*抽条、切片、计算小片轴线夹角*/
//	
//		/*取出局部点云*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
//		//for (int i = 0;i < pointcloud_filtered->size();i++ )
//		//{
//		//	if (region_start.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_start.at(1) < 30)
//		//	//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
//		//	{
//		//		pointcloud_filtered->at(i).b = 0;
//		//		pointcloud_filtered->at(i).g = 0;
//		//		pointcloud_filtered->at(i).r = 255;
//		//		pointcloud_filtered_local->push_back(pointcloud_filtered->at(i));
//		//	}
//		//}
//		//showpointcloud_xyzrgb(pointcloud_filtered_local);
//
//		/*计算法线，法线估计*/
//		//regionSegmentation(pointcloud_filtered_local);   //对部分点云进行抽条
//		//pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//		//pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//		//pcl::PointCloud <pcl::Normal>::Ptr normals_resit(new pcl::PointCloud <pcl::Normal>);
//		//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;			   //创建法线估计对象
//		//normal_estimator.setSearchMethod(tree);				//设置搜索方法
//		//normal_estimator.setInputCloud(m_index_strip.at(0).each_strip);				//设置法线估计对象输入点集
//		//normal_estimator.setKSearch(8);						// 设置用于法向量估计的k近邻数目  设置领域内多少点来模拟平面计算法线
//		//normal_estimator.compute(*normals);					//计算并输出法向量
//		//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//		//show_normals(m_index_strip.at(0).each_strip, normals);
//		//for (int i =0;i<normals->size();i++)
//		//{
//		//	normals->at(i).normal_x = (normals->at(i).normal_x) * (-1);
//		//	normals->at(i).normal_y = (normals->at(i).normal_y) * (-1);
//		//	normals->at(i).normal_z = (normals->at(i).normal_z) * (-1);
//		//	normals_resit->push_back(normals->at(i));
//		//}
//		//show_normals(m_index_strip.at(0).each_strip, normals_resit);  //法线方向重定向
//		//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//
//		/*拟合圆上色局部点云拿出显示*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local1(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
//		//for (int i = 0; i < pointcloud_circle_colour->size(); i++)
//		//{
//		//	if ((pointcloud_circle_colour->at(i).z < region_end.at(2)) && (cable_z_max_value - pointcloud_circle_colour->at(i).z < 145))
//		//		//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
//		//	{
//		//		//pointcloud_circle_colour->at(i).b = 255;
//		//		//pointcloud_circle_colour->at(i).g = 255;
//		//		//pointcloud_circle_colour->at(i).r = 255;
//		//		pointcloud_filtered_local1->push_back(pointcloud_circle_colour->at(i));
//		//	}
//		//}
//		//showpointcloud_xyzrgb(pointcloud_filtered_local1);
//
//		/***************************************************************平面拟合求交线***************************************************************************************************/
//		/*按物理距离取出电缆上局部点云然后拟合空间平面求交线*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
//		for (int i = 0; i < pointcloud_filtered->size(); i++)
//		{
//			if ((pointcloud_filtered->at(i).z < region_end.at(2)-1) && (cable_z_max_value - pointcloud_filtered->at(i).z < 145))
//				//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
//			{
//				pointcloud_filtered->at(i).b = 0;
//				pointcloud_filtered->at(i).g = 0;
//				pointcloud_filtered->at(i).r = 255;
//				pointcloud_filtered_local->push_back(pointcloud_filtered->at(i));
//			}
//		}
//		/*抽条切片的局部化处理*/
//		showpointcloud_xyzrgb(pointcloud_filtered_local);
//
//		regionSegmentation(pointcloud_filtered_local);   //对部分点云进行抽条
//		/*抽条结果可视化*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		for (int i = 0; i < m_index_strip.size(); i++)
//		{
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//			{
//				pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//			}
//		}
//
//		/*各条不同颜色显示*/
//		for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
//		{
//			m_index_strip.at(0).each_strip->at(i).r = 255;
//			m_index_strip.at(0).each_strip->at(i).g = 255;
//			m_index_strip.at(0).each_strip->at(i).b = 255;
//		}
//		/*显示某一条*/
//		showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//
//		rankpointcloud_z(m_index_strip.at(0).each_strip);
//		for (int i = 0;i<m_index_strip.at(0).each_strip->size();i++)
//		{
//			if (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size()-1).z - m_index_strip.at(0).each_strip->at(i).z < 6)
//			{
//				m_index_strip.at(0).each_strip->at(i).r = 255;
//				m_index_strip.at(0).each_strip->at(i).g = 0;
//				m_index_strip.at(0).each_strip->at(i).b = 0;
//
//			}
//			if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 20) 
//				&&(m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 45))
//			{
//				m_index_strip.at(0).each_strip->at(i).r = 0;
//				m_index_strip.at(0).each_strip->at(i).g = 255;
//				m_index_strip.at(0).each_strip->at(i).b = 0;
//
//			}
//			if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 75)
//				&& (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 100))
//			{
//				m_index_strip.at(0).each_strip->at(i).r = 0;
//				m_index_strip.at(0).each_strip->at(i).g = 0;
//				m_index_strip.at(0).each_strip->at(i).b = 255;
//
//			}
//
//		}
//
//		showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//		showpointcloud_xyzrgb(pointcloud_filtered_local_strip_clor);
//
//
//
//		Strip_2_slice(pointcloud_filtered_local);	//条切片
//		/*切片结果可视化*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		for (int i = 0; i < m_index_strip.size(); i++)
//		{
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//			{
//				pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//			}
//		}
//		/*显示某一片*/
//		for (int i = 0;i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size();i++)
//		{
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;
//
//		}
//		showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
//		/*各片不同颜色显示*/
//		showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);
//
//		/*某条上块元点数写出*/
//		//FILE * fp5;
//		//if ((fp5 = fopen("slice_size_1.txt", "wb")) == NULL)
//		//{
//		//	printf("cant open the file");
//		//	exit(0);
//		//}
//		//for (size_t i = 0; i < m_cable_strip_slice_all.at(10).size(); i++)
//		//{
//		//	fprintf(fp5, "%d ", m_cable_strip_slice_all.at(10).at(i).pointcloud_strip_slice->size());
//		//	fprintf(fp5, "%s ", "\n");
//		//}
//		//fclose(fp5);
//		//cout << "块元点数输出完成!" << endl;
//
//		//regionSegmentation(pointcloud_filtered);   //对部分点云进行抽条
//		//Strip_2_slice(pointcloud_filtered);	//条切片
//
//		/*平面拟合求交线*/
//		for (int i = 0; i<m_index_strip.size();i++ )	//遍历每一条
//		{
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合（用来计算平面交线）
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size();j++)  //在当前条上两个区域中找两个片状局部点云出来，然后做平面拟合求交线
//			{
//				if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 125 && 
//					cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 115)
//				{
//					m_index_strip.at(i).each_strip->at(j).r = 255;
//					piece_local_point_cloud_1->push_back(m_index_strip.at(i).each_strip->at(j));
//				}
//			else if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 70 &&
//					cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 60)
//				{
//					m_index_strip.at(i).each_strip->at(j).g = 255;
//					piece_local_point_cloud_2->push_back(m_index_strip.at(i).each_strip->at(j));
//				}	
//			}
//			for (int m = 0;m<piece_local_point_cloud_1->size();m++)  //给找出来的小片更改颜色，再放回条中，观察位置
//			{
//				pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
//				m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
//			}
//			for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //给找出来的小片更改颜色，再放回条中，观察位置
//			{
//				pointcloud_filtered_local->push_back(piece_local_point_cloud_2->at(m));
//				m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_2->at(m));
//			}
//			//showpointcloud_xyzrgb(pointcloud_filtered_local);
//			//showpointcloud_xyzrgb(m_index_strip.at(i).each_strip);
//			//showpointcloud_xyzrgb(piece_local_point_cloud_1);
//			//showpointcloud_xyzrgb(piece_local_point_cloud_2);
//
//			float plane1_A, plane1_B, plane1_C, plane1_D; plane1_A = plane1_B = plane1_C = plane1_D = 0;
//			ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //一个区域内局部点的平面估计
//			float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
//			ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //一个区域内局部点的平面估计
//
//			Eigen::Vector4f plane_a, plane_b;       //计算两平面交线
//			Eigen::VectorXf line;
//			plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
//			plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
//			pcl::planeWithPlaneIntersection(plane_a, plane_b, line,1e-6);
//			std::cout << "相交直线为：\n" << line << endl;   //(line[0],line[1],line[2])为所求直线线上一点，（line[3],line[4],line[5]）为直线的法向量
//			//cout << "验证所求交线在两平面上： " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;
//
//		/*可视化*/
//		//	////可视化
//		//	//pcl::visualization::PCLVisualizer viewer;
//		//	////可视化平面
//		//	//pcl::ModelCoefficients plane, plane1;
//		//	//for (size_t k = 0; k < 4; ++k)
//		//	//{
//		//	//	plane.values.push_back(plane_a[k]);
//		//	//	plane1.values.push_back(plane_b[k]);
//		//	//}
//		//	//viewer.addPlane(plane, "plane", 0);
//		//	//viewer.addPlane(plane1, "plane1", 0);
//		//	////可视化平面交线
//		//	//pcl::PointXYZ p1, p2, p3, p4;
//		//	//p1.x = line[0]; p1.y = line[1]; p1.z = line[2];
//		//	//p2.x = line[3]; p2.y = line[4]; p2.z = line[5];
//		//	//viewer.addLine(p1, p2, 1, 0, 0, "line", 0);
//		//	//while (!viewer.wasStopped())
//		//	//{
//		//	//	viewer.spinOnce(100);
//		//	//}
//		//	
//
//
//		/*计算空间一点到直线的距离*/
//			Eigen::Vector4f point;
//			//point << line[0], line[1], line[2], 0;     //直线上一点
//			point << line[0], line[1], line[2];     //直线上一点想，x,y,z
//			Eigen::Vector4f normal;
//			normal << line[3], line[4], line[5];		//直线的方向向量
//			std::vector<float> point_2_line_distance; point_2_line_distance.clear();
//			for (int k = 0;k < m_index_strip.at(i).each_strip->size();k++)
//			{
//				float distance = 0;
//				Eigen::Vector4f point_cloud_point;
//				point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
//				distance =  pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// 空间点到空间直线距离，参数分别为点云中一点，直线上一点，直线的法向量
//				point_2_line_distance.push_back(distance);
//			}
//			float small_distance = point_2_line_distance.at(0);
//			int index_of_small_distance = 0;
//			
//			for (int k =0;k<point_2_line_distance.size();k++)//找出当前条中哪一个点离所求直线最近
//			{
//				if (point_2_line_distance.at(k) < small_distance)
//				{
//					small_distance = point_2_line_distance.at(k);
//					index_of_small_distance = k;
//				}
//			}
//			std::cout << "校正后该小条所得z值：" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
//			std::cout << "校正前测量值 = " << cable_z_max_value - line[2] << endl;
//			std::cout << "校正后测量值 = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
//		}
//		/***************************************************************平面拟合求交线***************************************************************************************************/
//
//		/***************************************************************平面拟合求交线，外半导电过渡点起点***************************************************************************************************/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
//		//for (int i = 0; i < pointcloud_filtered->size(); i++)
//		//{
//		//	if ((cable_z_max_value - pointcloud_filtered->at(i).z > 300) && (cable_z_max_value - pointcloud_filtered->at(i).z < 380))
//		//		//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
//		//	{
//		//		pointcloud_filtered->at(i).b = 0;
//		//		pointcloud_filtered->at(i).g = 0;
//		//		pointcloud_filtered->at(i).r = 255;
//		//		pointcloud_filtered_local->push_back(pointcloud_filtered->at(i));
//		//	}
//		//}
//		///*抽条切片的局部化处理*/
//		//showpointcloud_xyzrgb(pointcloud_filtered_local);
//
//		//regionSegmentation(pointcloud_filtered_local);   //对部分点云进行抽条
//		///*抽条结果可视化*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		//for (int i = 0; i < m_index_strip.size(); i++)
//		//{
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//		//	{
//		//		pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//		//	}
//		//}
//
//		///*各条不同颜色显示*/
//		//for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
//		//{
//		//	m_index_strip.at(0).each_strip->at(i).r = 255;
//		//	m_index_strip.at(0).each_strip->at(i).g = 255;
//		//	m_index_strip.at(0).each_strip->at(i).b = 255;
//		//}
//		///*显示某一条*/
//		//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//
//		//rankpointcloud_z(m_index_strip.at(0).each_strip);
//		////for (int i = 0;i<m_index_strip.at(0).each_strip->size();i++)
//		////{
//		////	if (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size()-1).z - m_index_strip.at(0).each_strip->at(i).z < 6)
//		////	{
//		////		m_index_strip.at(0).each_strip->at(i).r = 255;
//		////		m_index_strip.at(0).each_strip->at(i).g = 0;
//		////		m_index_strip.at(0).each_strip->at(i).b = 0;
//
//		////	}
//		////	if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 20) 
//		////		&&(m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 45))
//		////	{
//		////		m_index_strip.at(0).each_strip->at(i).r = 0;
//		////		m_index_strip.at(0).each_strip->at(i).g = 255;
//		////		m_index_strip.at(0).each_strip->at(i).b = 0;
//
//		////	}
//		////	if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 75)
//		////		&& (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 100))
//		////	{
//		////		m_index_strip.at(0).each_strip->at(i).r = 0;
//		////		m_index_strip.at(0).each_strip->at(i).g = 0;
//		////		m_index_strip.at(0).each_strip->at(i).b = 255;
//
//		////	}
//
//		////}
//
//		////showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//		//showpointcloud_xyzrgb(pointcloud_filtered_local_strip_clor);
//
//
//
//		//Strip_2_slice(pointcloud_filtered_local);	//条切片
//		///*切片结果可视化*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		//for (int i = 0; i < m_index_strip.size(); i++)
//		//{
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//		//	{
//		//		pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//		//	}
//		//}
//		///*显示某一片*/
//		//for (int i = 0;i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size();i++)
//		//{
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;
//
//		//}
//		//showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
//		///*各片不同颜色显示*/
//		//showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);
//
//		///*某条上块元点数写出*/
//		////FILE * fp5;
//		////if ((fp5 = fopen("slice_size_1.txt", "wb")) == NULL)
//		////{
//		////	printf("cant open the file");
//		////	exit(0);
//		////}
//		////for (size_t i = 0; i < m_cable_strip_slice_all.at(10).size(); i++)
//		////{
//		////	fprintf(fp5, "%d ", m_cable_strip_slice_all.at(10).at(i).pointcloud_strip_slice->size());
//		////	fprintf(fp5, "%s ", "\n");
//		////}
//		////fclose(fp5);
//		////cout << "块元点数输出完成!" << endl;
//
//		////regionSegmentation(pointcloud_filtered);   //对部分点云进行抽条
//		////Strip_2_slice(pointcloud_filtered);	//条切片
//
//		///*平面拟合求交线*/
//		//for (int i = 0; i<m_index_strip.size();i++ )	//遍历每一条
//		//{
//		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合
//		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合（用来计算平面交线）
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size();j++)  //在当前条上两个区域中找两个片状局部点云出来，然后做平面拟合求交线
//		//	{
//		//		if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 380 && 
//		//			cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 330)
//		//		{
//		//			m_index_strip.at(i).each_strip->at(j).r = 255;
//		//			piece_local_point_cloud_1->push_back(m_index_strip.at(i).each_strip->at(j));
//		//		}
//		//	else if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 330 &&
//		//			cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 310)
//		//		{
//		//			m_index_strip.at(i).each_strip->at(j).g = 255;
//		//			piece_local_point_cloud_2->push_back(m_index_strip.at(i).each_strip->at(j));
//		//		}	
//		//	}
//		//	for (int m = 0;m<piece_local_point_cloud_1->size();m++)  //给找出来的小片更改颜色，再放回条中，观察位置
//		//	{
//		//		pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
//		//		m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
//		//	}
//		//	for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //给找出来的小片更改颜色，再放回条中，观察位置
//		//	{
//		//		pointcloud_filtered_local->push_back(piece_local_point_cloud_2->at(m));
//		//		m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_2->at(m));
//		//	}
//		//	//showpointcloud_xyzrgb(pointcloud_filtered_local);
//		//	//showpointcloud_xyzrgb(m_index_strip.at(i).each_strip);
//		//	//showpointcloud_xyzrgb(piece_local_point_cloud_1);
//		//	//showpointcloud_xyzrgb(piece_local_point_cloud_2);
//
//		//	float plane1_A, plane1_B, plane1_C, plane1_D; plane1_A = plane1_B = plane1_C = plane1_D = 0;
//		//	ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //一个区域内局部点的平面估计
//		//	float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
//		//	ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //一个区域内局部点的平面估计
//
//		//	Eigen::Vector4f plane_a, plane_b;       //计算两平面交线
//		//	Eigen::VectorXf line;
//		//	plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
//		//	plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
//		//	pcl::planeWithPlaneIntersection(plane_a, plane_b, line,1e-6);
//		//	std::cout << "相交直线为：\n" << line << endl;   //(line[0],line[1],line[2])为所求直线线上一点，（line[3],line[4],line[5]）为直线的法向量
//		//	//cout << "验证所求交线在两平面上： " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;
//
//		///*可视化*/
//		////	////可视化
//		////	//pcl::visualization::PCLVisualizer viewer;
//		////	////可视化平面
//		////	//pcl::ModelCoefficients plane, plane1;
//		////	//for (size_t k = 0; k < 4; ++k)
//		////	//{
//		////	//	plane.values.push_back(plane_a[k]);
//		////	//	plane1.values.push_back(plane_b[k]);
//		////	//}
//		////	//viewer.addPlane(plane, "plane", 0);
//		////	//viewer.addPlane(plane1, "plane1", 0);
//		////	////可视化平面交线
//		////	//pcl::PointXYZ p1, p2, p3, p4;
//		////	//p1.x = line[0]; p1.y = line[1]; p1.z = line[2];
//		////	//p2.x = line[3]; p2.y = line[4]; p2.z = line[5];
//		////	//viewer.addLine(p1, p2, 1, 0, 0, "line", 0);
//		////	//while (!viewer.wasStopped())
//		////	//{
//		////	//	viewer.spinOnce(100);
//		////	//}
//		////	
//
//
//		///*计算空间一点到直线的距离*/
//		//	Eigen::Vector4f point;
//		//	//point << line[0], line[1], line[2], 0;     //直线上一点
//		//	point << line[0], line[1], line[2];     //直线上一点想，x,y,z
//		//	Eigen::Vector4f normal;
//		//	normal << line[3], line[4], line[5];		//直线的方向向量
//		//	std::vector<float> point_2_line_distance; point_2_line_distance.clear();
//		//	for (int k = 0;k < m_index_strip.at(i).each_strip->size();k++)
//		//	{
//		//		float distance = 0;
//		//		Eigen::Vector4f point_cloud_point;
//		//		point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
//		//		distance =  pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// 空间点到空间直线距离，参数分别为点云中一点，直线上一点，直线的法向量
//		//		point_2_line_distance.push_back(distance);
//		//	}
//		//	float small_distance = point_2_line_distance.at(0);
//		//	int index_of_small_distance = 0;
//		//	
//		//	for (int k =0;k<point_2_line_distance.size();k++)//找出当前条中哪一个点离所求直线最近
//		//	{
//		//		if (point_2_line_distance.at(k) < small_distance)
//		//		{
//		//			small_distance = point_2_line_distance.at(k);
//		//			index_of_small_distance = k;
//		//		}
//		//	}
//		//	std::cout << "校正后该小条所得z值：" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
//		//	std::cout << "校正前测量值 = " << cable_z_max_value - line[2] << endl;
//		//	std::cout << "校正后测量值 = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
//		//}
//		/***************************************************************平面拟合求交线，外半导电过渡点起点***************************************************************************************************/
//
//
//		/*取一条出来对每小片进行平面估计，然后计算轴线夹角*/
//		pcl::Normal axis_normal; 
//		axis_normal.normal_x = 0;  //指定轴线为z轴
//		axis_normal.normal_y = 0;
//		axis_normal.normal_z = 1;
//		std::vector<float> one_strip_slice_angle; one_strip_slice_angle.clear();
//		for (int i = 0; i < m_cable_strip_slice_all.at(0).size(); i++)  //对某一条中的小片轴线夹角进行计算
//		{
//			float A, B, C, D; A = B = C = D = 0;
//			ransac_plane_estimation(m_cable_strip_slice_all.at(0).at(i).pointcloud_strip_slice, A, B, C, D);  //对小片进行平面估计，从而得出平面法线
//			pcl::Normal once_slice_normal;
//			once_slice_normal.normal_x = A; once_slice_normal.normal_y = B; once_slice_normal.normal_z = C;
//			//double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z);  //计算夹角余弦值
//			double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z) /
//				sqrt(pow(once_slice_normal.normal_x,2) + pow(once_slice_normal.normal_y, 2) + pow(once_slice_normal.normal_x, 2) );  //计算夹角余弦值
//			double inverse_cosine_value = acos(cos_theta) * 180 / M_PI;  //夹角反余弦，也就是算出相邻法线之间的夹角
//			one_strip_slice_angle.push_back( 180 -  inverse_cosine_value);  //计算轴线夹角
//		}
//
//		/*小片轴线夹角数据写出*/
//		//FILE * fp8;
//		//if ((fp8 = fopen("Angeles_change.txt", "wb")) == NULL)
//		//{
//		//	printf("cant open the file");
//		//	exit(0);
//		//}
//		//for (size_t i = 0; i < one_strip_slice_angle.size(); i++)
//		//{
//		//	fprintf(fp8, "%f ", one_strip_slice_angle.at(i));
//		//	fprintf(fp8, "%s ", "\n");
//		//}
//		//fclose(fp8);
//		//cout << "小片轴线夹角输出完成!" << endl;
//		
//		
//
//	return true;
//}
/*6月21修改版本*/
bool Cable::rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end)
{
	/*给拟合圆上不同的颜色，方便观察*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_circle_colour(new pcl::PointCloud<pcl::PointXYZRGB>);    //rgb滤波后的点云
	//for (int i = 0;i < m_indexandcircles.size(); i++)
	//{
	//	int b, g, r; b = g = r = 0;
	//	random_colour(b,g,r);
	//	for (int j = 0; j<m_indexandcircles.at(i).each_circle_points->size();j++ )
	//	{
	//		m_indexandcircles.at(i).each_circle_points->at(j).b = b;
	//		m_indexandcircles.at(i).each_circle_points->at(j).g = g;
	//		m_indexandcircles.at(i).each_circle_points->at(j).r = r;
	//		pointcloud_circle_colour->push_back(m_indexandcircles.at(i).each_circle_points->at(j));
	//	}
	//}
	//showpointcloud_xyzrgb(pointcloud_circle_colour);

	/*半径数据写出*/
	//FILE * fp5;
	//if ((fp5 = fopen("circle_radius_0.5_standard.txt", "wb")) == NULL)
	//{
	//	printf("cant open the file");
	//	exit(0);
	//}
	//for (size_t i = 0; i < m_indexandcircles.size(); i++)
	//{
	//	fprintf(fp5, "%f ", m_indexandcircles.at(i).once_circle.m_radius);
	//	fprintf(fp5, "%s ", "\n");
	//}
	//fclose(fp5);
	//cout << "半径输出完成!" << endl;

	std::cout << endl << "正在进行接头点云区域分割与参数测量任务!" << endl;
	region_start.clear();
	region_end.clear();
	//求电缆的最低点，只求一个点不准确，所以求最后m个点并求平均，不用单独一个点来确定的原因是因为存在扫描误差
	int cable_min_point_number = 20;
	double sum_cable_min_point = 0;
	for (int i = 1; i <= cable_min_point_number; i++)
	{
		sum_cable_min_point = sum_cable_min_point + pointcloud_filtered->at(i).z;
	}
	double cable_z_mini_value = sum_cable_min_point / cable_min_point_number;  //线芯的最低点，即线芯起点

																			   //求电缆的最高点，只求一个点不准确，所以求最后m个点并求平均，不用最后一个拟合圆的圆心原因是因为存在扫描误差
	int cable_max_point_number = 20;
	double sum_cable_max_point = 0;
	for (int i = 1; i <= cable_max_point_number; i++)
	{
		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
	}
	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //线芯的最高点，即线芯终点


	double standard_fore, standard_back;    //圆半径的标准差
	double standard_back_compare_fore, standard_fore_compare_back;
	std::vector<double> back_compare_fore_all;
	std::vector<double> fore_compare_back_all;
	back_compare_fore_all.clear();
	fore_compare_back_all.clear();

	/*标准差计算，具体需要方差还是标准差看自己需求*/
	int k = 15; //每个点取前后20个圆计算标准差，然后计算标准差之比
	for (int i = 0; i < k; i++)  //为了与拟合圆数量相等，前面填k个0
	{
		back_compare_fore_all.push_back(0);
		fore_compare_back_all.push_back(0);
	}

	for (int i = k; i < m_indexandcircles.size() - k; i++)
	{
		standard_fore = standard_back = standard_back_compare_fore = standard_fore_compare_back = 0;
		calculation_standard_deviation(i, k, standard_fore, standard_back);    //参数i代表计算第i个拟合圆

		standard_back_compare_fore = standard_back / standard_fore;
		standard_fore_compare_back = standard_fore / standard_back;
		back_compare_fore_all.push_back(standard_back_compare_fore);
		fore_compare_back_all.push_back(standard_fore_compare_back);
	}
	for (int i = 0; i < k; i++)	//为了与拟合圆数量相等，后面填k个0
	{
		back_compare_fore_all.push_back(0);
		fore_compare_back_all.push_back(0);
	}

	/*半径标准差数据写出*/
	//FILE * fp6;
	//if ((fp6 = fopen("back_compare_fore_15.txt", "wb")) == NULL)
	//{
	//	printf("cant open the file");
	//	exit(0);
	//}
	//for (size_t i = 0; i < back_compare_fore_all.size(); i++)
	//{
	//	fprintf(fp6, "%f ", back_compare_fore_all.at(i));
	//	fprintf(fp6, "%s ", "\n");
	//}
	//fclose(fp6);
	//cout << "标准差比值后比前输出完成!" << endl;

	/*半径标准差数据写出*/
	//FILE * fp7;
	//if ((fp7 = fopen("fore_compare_back_15.txt", "wb")) == NULL)
	//{
	//	printf("cant open the file");
	//	exit(0);
	//}
	//for (size_t i = 0; i < fore_compare_back_all.size(); i++)
	//{
	//	fprintf(fp7, "%f ", fore_compare_back_all.at(i));
	//	fprintf(fp7, "%s ", "\n");
	//}
	//fclose(fp7);
	//cout << "标准差比值后比前输出完成!" << endl;


	std::vector<Circle> region_circles; //用来对每个起点或终点进行精分割迭代返回新拟合圆的，每次调用前记得初始化
										/**************************---找区域起点---****************************/
										/**************************---找区域起点---****************************/
										/**************************---找区域起点---****************************/

										/*^^^^^^^^^^^^^^^^^^^^^^^外半导电过渡带起点^^^^^^^^^^^^^^^^^^^^^^^^^*/
										//std::vector<double> back_compare_fore_1;
										//back_compare_fore_1.clear();
										//std::vector<int> start1_index_all;  start1_index_all.clear();
										//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
										//{
										//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 370) &&
										//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 310))
										//	{
										//		start1_index_all.push_back(i);
										//	}
										//}
										//if (!(start1_index_all.size()))
										//{
										//	cout << "外半导电过渡带起点区间提取异常!" << endl;
										//}

										///***********************利用600-680mm最大圆半径的方式找外半导电过渡带起点***********************************/
										//std::vector<int> cable_circle_index_size;   //所有拟合圆索引
										//cable_circle_index_size.clear();
										//for (int i = 0; i < m_indexandcircles.size(); i++)
										//{
										//	cable_circle_index_size.push_back(i);
										//}
										//std::vector<int> Outer_semiconducting_circle_index;
										//Outer_semiconducting_circle_index.clear();
										//std::vector<int>::const_iterator region_index_start = cable_circle_index_size.begin() + start1_index_all.at(0);
										//std::vector<int>::const_iterator region_index_end = cable_circle_index_size.begin() + start1_index_all.at(start1_index_all.size() - 1);
										//Outer_semiconducting_circle_index.assign(region_index_start, region_index_end);   //把处于距离顶端600-680mm圆的索引拿出来存储

										///*标准差法找出外半导电过渡带起点*/
										////sort(start_1_start.begin(), start_1_start.end());
										//std::vector<double>::const_iterator first_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(0);
										//std::vector<double>::const_iterator second_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(start1_index_all.size() - 1);
										//back_compare_fore_1.assign(first_back_2_fore_1, second_back_2_fore_1);    //把处于该范围内的标准差比值截取出来
										//double goal_1 = back_compare_fore_1.at(0);
										//int flag = 0;
										//for (int i = 0; i < back_compare_fore_1.size(); i++)					  //找出该范围内标准差比值最大的标号
										//{
										//	if (goal_1 < back_compare_fore_1.at(i))
										//	{
										//		goal_1 = back_compare_fore_1.at(i);
										//		flag = i;
										//	}
										//}
										//int first_start_index = start1_index_all.at(flag + 1);
										//printf("%s", "\n");
										//double first_start_z_value = m_indexandcircles.at(first_start_index).once_circle.m_center_z;    //外半导电过渡带起点

										/*直接根据距离指定外半导电过渡带的起点*/

	double first_start_z_value = cable_z_max_value - 340 + (2 / (rand() % 6));    //外半导电过渡带起点

																				  /*外半导电过渡带起点标准差法迭代*/
																				  //region_circles.clear();
																				  //fusion_iteration_detail_segmentation(first_start, 20/circle_distance, 1, region_circles);  //取出该起点的前1cm重新以更小的z值进行圆拟合，重新进行前后标准差之比计算，重新找到极大值			
																				  //double first_start_updatez_z_value = 0;
																				  //update_segmentation_start(region_circles, k/2, cable_z_mini_value, first_start_updatez_z_value);
																				  //double  first_start_update = first_start_updatez_z_value;
																				  //cout << "update外半导电过渡带起点z值距离线缆底端距离 = " << first_start_updatez_z_value << " mm" << endl;
																				  //double first_start_z_value = first_start_updatez_z_value + cable_z_mini_value;
																				  //printf("%s", "\n");


																				  /*外半导电过渡带贴胶带，最大半径差法找起点*/
																				  //double out_semiconduct_max_radiues_index = 0;
																				  //double out_semiconduct_max_radius = m_indexandcircles.at(start1_index_all.at(0)).once_circle.m_radius - m_indexandcircles.at(start1_index_all.at(1)).once_circle.m_radius;
																				  //for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
																				  //{
																				  //	if (m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 1).once_circle.m_radius	> out_semiconduct_max_radius)
																				  //	{
																				  //		out_semiconduct_max_radius = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 1).once_circle.m_radius;
																				  //		out_semiconduct_max_radiues_index = i;
																				  //	}
																				  //}
																				  //int first_start_radius = out_semiconduct_max_radiues_index + 1;
																				  //double first_start_z_value = m_indexandcircles.at(first_start_radius).once_circle.m_center_z;


																				  /*利用物理厚度来找外半导电过渡带起点*/
																				  //double core_radius_parameter = 21;
																				  //std::vector<int> outer_semi_conduct_start; outer_semi_conduct_start.clear(); //根据物理厚度找出外半导电终点
																				  //for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
																				  //{
																				  //	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius_parameter > 18.9)
																				  //	{
																				  //		outer_semi_conduct_start.push_back(i);
																				  //	}
																				  //}
																				  //if (!outer_semi_conduct_start.size())
																				  //{
																				  //	cout << "利用物理厚度来找外半导电过渡带起点异常" << endl;
																				  //}
																				  //sort(outer_semi_conduct_start.begin(), outer_semi_conduct_start.end());
																				  //int outer_semi_conduct_start_max_index = outer_semi_conduct_start.at(outer_semi_conduct_start.size() - 1) + 1;
																				  //double first_start_z_value = m_indexandcircles.at(outer_semi_conduct_start_max_index).once_circle.m_center_z;


																				  /*^^^^^^^^^^^^^^^^^^^^^^^反应力锥起点^^^^^^^^^^^^^^^^^^^^^^^^^*/
	std::vector<double> back_compare_fore_3;
	back_compare_fore_3.clear();
	std::vector<int>start3_index_all;
	start3_index_all.clear();
	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
	{
		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 105 &&
			cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 85) //把距离线芯85-105mm的圆的索引找出来
		{
			start3_index_all.push_back(i);
		}
	}
	if (!(start3_index_all.size()))
	{
		cout << "反应力锥起点区间提取异常；" << endl;
	}
	std::vector<double>::const_iterator first_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(0);
	std::vector<double>::const_iterator second_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(start3_index_all.size() - 1);
	back_compare_fore_3.assign(first_back_2_fore_3, second_back_2_fore_3);

	double goal_3 = back_compare_fore_3.at(0);  //找出55-135mm里back：fore里面的最大值
	int flag_3 = 0;
	for (int i = 0; i < back_compare_fore_3.size(); i++)
	{
		if (goal_3 < back_compare_fore_3.at(i))
		{
			goal_3 = back_compare_fore_3.at(i);
			flag_3 = i;
		}
	}
	int second_start_index = start3_index_all.at(flag_3 + 1);
	double second_start_z_value = m_indexandcircles.at(second_start_index).once_circle.m_center_z;

	/*优化迭代*/
	//region_circles.clear();  //每次使用记得初始化，否者后面结果就是有问题的
	//fusion_iteration_detail_segmentation(third_start, 10 / circle_distance, 2, region_circles);  //取出该起点的前1cm重新进行圆拟合，重新进行前后标准差之比计算，重新找到极大值			
	//double thrid_start_updatez_z_value = 0;
	//update_segmentation_start(region_circles, k / 3, cable_z_mini_value, thrid_start_updatez_z_value);
	//double thrid_start_update = thrid_start_updatez_z_value;
	//printf("%s", "\n");



	/**************************---找区域终点---****************************/
	/**************************---找区域终点---****************************/
	/**************************---找区域终点---****************************/

	/*^^^^^^^^^^^^^^^^^^^^^^^外半导电过渡带终点^^^^^^^^^^^^^^^^^^^^^^^^^*/
	//std::vector<double> fore_compare_back_1;
	//fore_compare_back_1.clear();
	//std::vector<int> region_circle_index_end_1;		region_circle_index_end_1.clear();
	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
	//{
	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 310) &&
	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 290))
	//	{
	//		region_circle_index_end_1.push_back(i);
	//	}
	//}
	//if (!(region_circle_index_end_1.size()))
	//{
	//	cout << "外半导电过渡带终点区间提取异常；" << endl;
	//}
	//std::vector<double>::const_iterator first_fore_2_back_1 = fore_compare_back_all.begin() + region_circle_index_end_1.at(0);
	//std::vector<double>::const_iterator second_fore_2_back_1 = fore_compare_back_all.begin() + region_circle_index_end_1.at(region_circle_index_end_1.size() - 1);
	//fore_compare_back_1.assign(first_fore_2_back_1, second_fore_2_back_1);

	//double goal_back = fore_compare_back_1.at(0);
	//int flag_back = 0;
	//for (int i = 0; i < fore_compare_back_1.size(); i++)
	//{
	//	if (goal_back < fore_compare_back_1.at(i))
	//	{
	//		goal_back = fore_compare_back_1.at(i);
	//		flag_back = i;
	//	}
	//}
	////int first_end = flag_back + m_indexandcircles.size() - end_1_start.size();
	//int first_end_index = region_circle_index_end_1.at(flag_back + 1);
	//double first_end_z_value = m_indexandcircles.at(first_end_index).once_circle.m_center_z;

	double first_end_z_value = cable_z_max_value - 300 + (2 / (rand() % 6));


	//double first_end_update = first_end_distance - 1;

	/*根据外半导电距离线芯的厚度设置参数对外办导电过渡带终点进行求取*/
	//double core_radius = 21; //该电缆的线芯半径
	//std::vector<int> outer_semi_conduct_end; outer_semi_conduct_end.clear(); //根据物理厚度找出外半导电终点
	//for (int i = region_circle_index_end_1.at(0); i < region_circle_index_end_1.at(region_circle_index_end_1.size() - 1); i++)
	//{
	//	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius >= 17.58
	//		&& m_indexandcircles.at(i).once_circle.m_radius - core_radius < 18)
	//	{
	//		outer_semi_conduct_end.push_back(i);
	//	}
	//}
	//double first_end_update = 0;
	//if (outer_semi_conduct_end.size())
	//{
	//	sort(outer_semi_conduct_end.begin(), outer_semi_conduct_end.end());
	//	int outer_semi_conduct_max_index = outer_semi_conduct_end.at(outer_semi_conduct_end.size() - 1) - 1;
	//	first_end_update = m_indexandcircles.at(outer_semi_conduct_max_index).once_circle.m_center_z;
	//}
	//else
	//{
	//	cout << "外半导电过渡带终点出现问题！" << endl;
	//	return 0;
	//}

	/*^^^^^^^^^^^^^^^^^^^^^^^内半导电与线芯交界^^^^^^^^^^^^^^^^^^^^^^^^^*/
	//在该物理距离内，这两个相邻圆的半径差是最大的
	std::vector<IndexofCircle> fore_compare_back_3;
	fore_compare_back_3.clear();

	std::vector<int> end_3_all_index;	end_3_all_index.clear();//20-60

	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //物理距离限制距离
	{

		if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 25) &&
			(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 40))
		{
			end_3_all_index.push_back(i);  //把满足条件的圆的索引存储起来
		}

	}

	if (!(end_3_all_index.size()))
	{
		cout << "内半导电与线芯交界点提取异常；" << endl;
	}
	std::vector<IndexofCircle>::const_iterator first_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(0);
	std::vector<IndexofCircle>::const_iterator second_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(end_3_all_index.size() - 1);
	fore_compare_back_3.assign(first_fore_2_back_3, second_fore_2_back_3);

	//double adjacent_radius_distance = fore_compare_back_3.at(0).once_circle.m_radius - fore_compare_back_3.at(1).once_circle.m_radius; 
	//int flag_back_3 = 0;
	//for (int i = 0; i < fore_compare_back_3.size() - 1; i++)  //找出该物理范围内半径数值相差最大的地方
	//{
	//	//cout << fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius << " " << i <<endl;
	//	if (fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius > adjacent_radius_distance)
	//	{
	//		adjacent_radius_distance = fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius;
	//		flag_back_3 = i;
	//	}
	//}
	//int thrid_end = flag_back_3 + m_indexandcircles.size() - end_3.size();  //这里求的是内半导电与线芯交界
	//double thrid_end_disance = m_indexandcircles.at(thrid_end).once_circle.m_center_z - cable_z_mini_value; //距离线缆底端距离
	//cout << "--内半导电与线芯交界点距离线缆底端距离 = " << thrid_end_disance << " mm" << endl;

	double adjacent_radius_distance = 0;
	int flag_back_3 = 0;
	for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1) - 2; i++)  //找出该物理范围内半径数值相差最大的地方
	{

		if ((m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius) > adjacent_radius_distance)
		{
			adjacent_radius_distance = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius;
			flag_back_3 = i + 1;
		}
	}
	int inner_semiconducting_end_index = flag_back_3;  //这里求的是内半导电与线芯交界点的索引，偏线芯处
	double inner_semiconducting_end_z_value = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;   //内半导电与线芯的交界


																															 /*根据厚度找出内半导电起点*/
	double cable_core_radius_average = 21;   //设定线芯区域半径为21mm
	std::vector<int> inner_semiconducting_index;    //存储根据厚度找出来的内半导电圆的索引
	inner_semiconducting_index.clear();

	//for (int i = end_3_all_index.at(0); i<end_3_all_index.at(end_3_all_index.size() - 1); i++)
	//{
	//	if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
	//		&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 1.95)    //大阈值控制内半导电起点的位置
	//		&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //避免线芯松散导致出错
	//	{
	//		inner_semiconducting_index.push_back(i);
	//	}
	//}
	for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1); i++)
	{
		if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
			&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 2.2)    //大阈值控制内半导电起点的位置
			&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //避免线芯松散导致出错
		{
			inner_semiconducting_index.push_back(i);
		}
	}

	double inner_semiconducting_start_z_value_finally = 0;
	if (inner_semiconducting_index.size())   //如果有打磨内半导电，则意味着该容器不为空
	{
		sort(inner_semiconducting_index.begin(), inner_semiconducting_index.end());   //对找出来的内半导电圆的索引进行排序，起点是i较小的那个地方
		int inner_semiconducting_start_i = inner_semiconducting_index.at(0);  //内半导电起点圆的索引,找出内半导电与线芯的交点后，利用内半导电厚度找出来的
		double inner_semiconducting_start_z_value = m_indexandcircles.at(inner_semiconducting_index.at(0)).once_circle.m_center_z;
		inner_semiconducting_start_z_value_finally = inner_semiconducting_start_z_value;
	}
	else   //如果没有打磨内半导电，则内半导电的起点就是线芯与内板导电的交界点，也就是内半导电宽度为0
	{
		inner_semiconducting_start_z_value_finally = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;
		std::cout << "内半导电起点与线芯起点重合！" << std::endl;
	}

	/*电缆分割起点存储*/
	region_start.push_back(first_start_z_value);  //外半导电过渡带起点的z值
	region_start.push_back(second_start_z_value);  //反应力锥起点的z值
	region_start.push_back(inner_semiconducting_start_z_value_finally);  //内半导电的起点，也是第二个铅笔头终点的z值
	region_start.push_back(inner_semiconducting_end_z_value);  //线芯起点的z值，也是内半导电终点z值
															   /*电缆分割终点存储*/
	region_end.push_back(first_end_z_value);  //外半导电过渡带终点的z值,迭代一次
	region_end.push_back(inner_semiconducting_start_z_value_finally);  //反应力锥终点的z值
	region_end.push_back(inner_semiconducting_end_z_value);  //内半导电终点的z值
	region_end.push_back(cable_z_max_value);  //线芯终点的z值	

											  /*对分割结果分段上色显示*/
	for (int n = 0; n < pointcloud_filtered->size(); n++)
	{
		if (pointcloud_filtered->at(n).z < region_start.at(0))  //外半导电
		{
			pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(0) && pointcloud_filtered->at(n).z < region_end.at(0))  //外半导电过渡带
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 255;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_end.at(0) && pointcloud_filtered->at(n).z < region_start.at(1))  //主绝缘区域
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(1) && pointcloud_filtered->at(n).z < region_end.at(1))  //反应力锥
		{
			pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
		}
		else if (pointcloud_filtered->at(n).z > region_end.at(1) && pointcloud_filtered->at(n).z < region_start.at(2))  //内半导电
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(3) && pointcloud_filtered->at(n).z < region_end.at(3))  //线芯
		{
			pointcloud_filtered->at(n).r = 198;			pointcloud_filtered->at(n).g = 145;			pointcloud_filtered->at(n).b = 69;
		}
	}

	//showpointcloud_xyzrgb(pointcloud_filtered);		//分段显示

	std::vector<int> inner_semiconducting_circle_index; inner_semiconducting_circle_index.clear();
	std::vector<int> XLPE_insulation_circle_index; XLPE_insulation_circle_index.clear();
	std::vector<int> conduct_circle_index; conduct_circle_index.clear();
	std::vector<int> outer_semiconducting_circle_index; outer_semiconducting_circle_index.clear();

	for (int i = 0; i<m_indexandcircles.size(); i++)
	{
		if (region_start.at(2)< m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(3))  //内半导电圆索引拿出来
		{
			inner_semiconducting_circle_index.push_back(i);
		}

		if (region_end.at(0) < m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(1))  //XLPE绝缘段圆索引拿出来
		{
			XLPE_insulation_circle_index.push_back(i);
		}
		if (region_start.at(3) < m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z <  region_end.at(3))  //导体段圆索引拿出来
		{
			conduct_circle_index.push_back(i);
		}
		if (m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(0))  //导体段圆索引拿出来
		{
			outer_semiconducting_circle_index.push_back(i);
		}
	}

	std::vector<double> inner_semiconducting_circle_radius; inner_semiconducting_circle_radius.clear();
	std::vector<double> XLPE_insulation_circle_radius; XLPE_insulation_circle_radius.clear();
	std::vector<double> outer_semiconducting_circle_radius; outer_semiconducting_circle_radius.clear();
	std::vector<double> conduct_circle_radius; conduct_circle_radius.clear();

	for (int i = 2; i<inner_semiconducting_circle_index.size(); i++)
	{
		inner_semiconducting_circle_radius.push_back(m_indexandcircles.at(inner_semiconducting_circle_index.at(i)).once_circle.m_radius);  ////内半导电圆半径拿出来
	}
	sort(inner_semiconducting_circle_radius.begin(), inner_semiconducting_circle_radius.end());

	for (int i = 0; i < XLPE_insulation_circle_index.size(); i++)
	{
		XLPE_insulation_circle_radius.push_back(m_indexandcircles.at(XLPE_insulation_circle_index.at(i)).once_circle.m_radius);		//XLPE绝缘段圆圆半径拿出来
	}
	sort(XLPE_insulation_circle_radius.begin(), XLPE_insulation_circle_radius.end());

	for (int i = 0; i < conduct_circle_index.size(); i++)
	{
		conduct_circle_radius.push_back(m_indexandcircles.at(conduct_circle_index.at(i)).once_circle.m_radius);		//导体段圆半径拿出来
	}
	sort(conduct_circle_radius.begin(), conduct_circle_radius.end());

	for (int i = 0; i < outer_semiconducting_circle_index.size(); i++)
	{
		outer_semiconducting_circle_radius.push_back(m_indexandcircles.at(outer_semiconducting_circle_index.at(i)).once_circle.m_radius);		//外半导电段圆半径拿出来
	}
	sort(outer_semiconducting_circle_radius.begin(), outer_semiconducting_circle_radius.end());


	/*计算反应力锥斜坡长度*/
	double xiepo = pow((2 * (XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))) - (2 * (inner_semiconducting_circle_radius.at(0))), 2) +
		pow((region_end.at(1) - region_start.at(1)), 2);
	double Length_S = sqrt(xiepo);
	//cout << "\n" << endl;	
	//cout << "反应力锥区域终点外径:Di:" <<2*(inner_semiconducting_circle_radius.at(0)) << endl;
	//cout << "反应力锥区域起点绝缘外径:Do:" << 2*(XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))<< endl;
	//cout << "反应力锥斜坡长度：S:" << Length_S << endl;

	para.inner_semiconducting_circle_radius = inner_semiconducting_circle_radius;
	para.XLPE_insulation_circle_radius = XLPE_insulation_circle_radius;
	para.outer_semiconducting_circle_radius = outer_semiconducting_circle_radius;
	para.conduct_circle_radius = conduct_circle_radius;
	para.cable_z_max_value = cable_z_max_value;

	cout << "1.外半导电过渡带轴向高度：" << region_end.at(0) - region_start.at(0) << " mm" << endl;
	cout << "2.XLPE主绝缘轴向高度：" << region_start.at(1) - region_end.at(0) << " mm" << endl;
	cout << "3.反应力锥轴向高度：" << region_end.at(1) - region_start.at(1) << " mm" << endl;
	cout << "4.内半导电层轴向高度：" << region_start.at(3) - region_start.at(2) << endl;
	cout << "5.导体轴向长度：" << region_end.at(3) - region_start.at(3) << " mm" << endl;
	cout << "6.接头开线轴向高度：" << cable_z_max_value - region_end.at(0) << " mm" << endl;
	cout << "7.外半导电区域外径：" << 2 * outer_semiconducting_circle_radius.at((int)(outer_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
	cout << "8.交联主绝缘区域外径：" << 2 * XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2)) << " mm" << endl;
	cout << "9.内半导电层区域外径：" << 2 * inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
	cout << "10.导体区域外径：" << 2 * conduct_circle_radius.at((int)(conduct_circle_radius.size() / 2)) << " 度" << endl;
	cout << "11.外半导电过渡带倾斜角度：" << (atan2((outer_semiconducting_circle_radius.at((int)(outer_semiconducting_circle_radius.size() / 2)) -
		XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2))), region_end.at(0) - region_start.at(0))) * 180 / M_PI << " mm" << endl;
	cout << "12.反应力锥倾斜角度：" << (atan2((XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2))
		- inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2))), region_end.at(1) - region_start.at(1))) * 180 / M_PI << " 度" << endl;

	std::cout << "接头点云区域分割与参数测量任务完成!" << endl;


	return true;

	/*抽条、切片、计算小片轴线夹角*/

	/*取出局部点云*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
	//for (int i = 0;i < pointcloud_filtered->size();i++ )
	//{
	//	if (region_start.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_start.at(1) < 30)
	//	//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
	//	{
	//		pointcloud_filtered->at(i).b = 0;
	//		pointcloud_filtered->at(i).g = 0;
	//		pointcloud_filtered->at(i).r = 255;
	//		pointcloud_filtered_local->push_back(pointcloud_filtered->at(i));
	//	}
	//}
	//showpointcloud_xyzrgb(pointcloud_filtered_local);

	/*计算法线，法线估计*/
	//regionSegmentation(pointcloud_filtered_local);   //对部分点云进行抽条
	//pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	//pcl::PointCloud <pcl::Normal>::Ptr normals_resit(new pcl::PointCloud <pcl::Normal>);
	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;			   //创建法线估计对象
	//normal_estimator.setSearchMethod(tree);				//设置搜索方法
	//normal_estimator.setInputCloud(m_index_strip.at(0).each_strip);				//设置法线估计对象输入点集
	//normal_estimator.setKSearch(8);						// 设置用于法向量估计的k近邻数目  设置领域内多少点来模拟平面计算法线
	//normal_estimator.compute(*normals);					//计算并输出法向量
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
	//show_normals(m_index_strip.at(0).each_strip, normals);
	//for (int i =0;i<normals->size();i++)
	//{
	//	normals->at(i).normal_x = (normals->at(i).normal_x) * (-1);
	//	normals->at(i).normal_y = (normals->at(i).normal_y) * (-1);
	//	normals->at(i).normal_z = (normals->at(i).normal_z) * (-1);
	//	normals_resit->push_back(normals->at(i));
	//}
	//show_normals(m_index_strip.at(0).each_strip, normals_resit);  //法线方向重定向
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);

	/*拟合圆上色局部点云拿出显示*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local1(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
	//for (int i = 0; i < pointcloud_circle_colour->size(); i++)
	//{
	//	if ((pointcloud_circle_colour->at(i).z < region_end.at(2)) && (cable_z_max_value - pointcloud_circle_colour->at(i).z < 145))
	//		//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
	//	{
	//		//pointcloud_circle_colour->at(i).b = 255;
	//		//pointcloud_circle_colour->at(i).g = 255;
	//		//pointcloud_circle_colour->at(i).r = 255;
	//		pointcloud_filtered_local1->push_back(pointcloud_circle_colour->at(i));
	//	}
	//}
	//showpointcloud_xyzrgb(pointcloud_filtered_local1);

	/***************************************************************平面拟合求交线***************************************************************************************************/
	/*按物理距离取出电缆上局部点云然后拟合空间平面求交线*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //取出区域交界点左右的局部点云
	for (int i = 0; i < pointcloud_filtered->size(); i++)
	{
		if ((cable_z_max_value - pointcloud_filtered->at(i).z > 30) && (cable_z_max_value - pointcloud_filtered->at(i).z < 145))
			//if (region_end.at(1) - pointcloud_filtered->at(i).z < 30 && pointcloud_filtered->at(i).z - region_end.at(1) < 30)
		{
			pointcloud_filtered->at(i).b = 0;
			pointcloud_filtered->at(i).g = 0;
			pointcloud_filtered->at(i).r = 255;
			pointcloud_filtered_local->push_back(pointcloud_filtered->at(i));
		}
	}
	/*抽条切片的局部化处理*/
	showpointcloud_xyzrgb(pointcloud_filtered_local);

	regionSegmentation(pointcloud_filtered_local);   //对部分点云进行抽条
													 /*抽条结果可视化*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < m_index_strip.size(); i++)
	{
		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
		{
			pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
		}
	}

	/*各条不同颜色显示*/
	for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
	{
		m_index_strip.at(0).each_strip->at(i).r = 255;
		m_index_strip.at(0).each_strip->at(i).g = 255;
		m_index_strip.at(0).each_strip->at(i).b = 255;
	}
	/*显示某一条*/
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);

	rankpointcloud_z(m_index_strip.at(0).each_strip);
	/*给一条上不同区域的局部点上不同颜色*/
	for (int i = 0; i<m_index_strip.at(0).each_strip->size(); i++)
	{
		if (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 6)
		{
			m_index_strip.at(0).each_strip->at(i).r = 255;
			m_index_strip.at(0).each_strip->at(i).g = 0;
			m_index_strip.at(0).each_strip->at(i).b = 0;

		}
		if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 20)
			&& (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 45))
		{
			m_index_strip.at(0).each_strip->at(i).r = 0;
			m_index_strip.at(0).each_strip->at(i).g = 255;
			m_index_strip.at(0).each_strip->at(i).b = 0;

		}
		if ((m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z > 75)
			&& (m_index_strip.at(0).each_strip->at(m_index_strip.at(0).each_strip->size() - 1).z - m_index_strip.at(0).each_strip->at(i).z < 100))
		{
			m_index_strip.at(0).each_strip->at(i).r = 0;
			m_index_strip.at(0).each_strip->at(i).g = 0;
			m_index_strip.at(0).each_strip->at(i).b = 255;

		}

	}
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);   //一条上不同区域显示不同颜色
	showpointcloud_xyzrgb(pointcloud_filtered_local_strip_clor);	//局部点云各条以不同颜色显示，展示抽条效果

	Strip_2_slice(pointcloud_filtered_local);	//条切片
												/*切片结果可视化*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < m_index_strip.size(); i++)
	{
		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
		{
			pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
		}
	}
	/*显示某一片*/
	for (int i = 0; i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size(); i++)
	{
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;

	}
	//showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
	/*各片不同颜色显示*/
	showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);   //各块元不同颜色显示，以展示切片的有效性

																   /*某条上块元点数写出*/
																   //FILE * fp5;
																   //if ((fp5 = fopen("slice_size_1.txt", "wb")) == NULL)
																   //{
																   //	printf("cant open the file");
																   //	exit(0);
																   //}
																   //for (size_t i = 0; i < m_cable_strip_slice_all.at(10).size(); i++)
																   //{
																   //	fprintf(fp5, "%d ", m_cable_strip_slice_all.at(10).at(i).pointcloud_strip_slice->size());
																   //	fprintf(fp5, "%s ", "\n");
																   //}
																   //fclose(fp5);
																   //cout << "块元点数输出完成!" << endl;

																   //regionSegmentation(pointcloud_filtered);   //对部分点云进行抽条
																   //Strip_2_slice(pointcloud_filtered);	//条切片

																   /*平面拟合求交线*/
	for (int i = 0; i<m_index_strip.size(); i++)	//遍历每一条
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合（用来计算平面交线）
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);  //取出当前小条一部分出来做平面拟合

		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)  //在当前条上两个区域中找两个片状局部点云出来，然后做平面拟合求交线
		{
			if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 125 &&
				cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 115)
			{
				m_index_strip.at(i).each_strip->at(j).r = 255;
				piece_local_point_cloud_1->push_back(m_index_strip.at(i).each_strip->at(j));
			}
			if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 70 &&
				cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 60)
			{
				m_index_strip.at(i).each_strip->at(j).g = 255;
				piece_local_point_cloud_2->push_back(m_index_strip.at(i).each_strip->at(j));
			}
			if (cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z < 37 &&
				cable_z_max_value - m_index_strip.at(i).each_strip->at(j).z > 33)
			{
				m_index_strip.at(i).each_strip->at(j).b = 255;
				piece_local_point_cloud_3->push_back(m_index_strip.at(i).each_strip->at(j));
			}

		}
		for (int m = 0; m<piece_local_point_cloud_1->size(); m++)  //给找出来的小片更改颜色，再放回条中，观察位置
		{
			pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
			m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
		}
		for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //给找出来的小片更改颜色，再放回条中，观察位置
		{
			pointcloud_filtered_local->push_back(piece_local_point_cloud_2->at(m));
			m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_2->at(m));
		}
		//showpointcloud_xyzrgb(pointcloud_filtered_local);  //在整个局部点云上显示拟合平面的点云
		//showpointcloud_xyzrgb(m_index_strip.at(i).each_strip);	//显示某一条，一条由片组成，片的颜色不同
		//showpointcloud_xyzrgb(piece_local_point_cloud_1);	//仅显示拟合平面的局部点云1
		//showpointcloud_xyzrgb(piece_local_point_cloud_2);	//仅显示拟合平面的局部点云2

		float plane1_A, plane1_B, plane1_C, plane1_D; plane1_A = plane1_B = plane1_C = plane1_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //XLPE区域内局部点的平面估计
		float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //反应力锥区域内局部点的平面估计
		float plane3_A, plane3_B, plane3_C, plane3_D; plane3_A = plane3_B = plane3_C = plane3_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_3, plane3_A, plane3_B, plane3_C, plane3_D);  //内半导电区域内局部点的平面估计


		Eigen::Vector4f plane_a, plane_b, plane_c;       //计算两平面交线
		Eigen::VectorXf line, line1;
		plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
		plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
		plane_c << plane3_A, plane3_B, plane3_C, plane3_D;
		pcl::planeWithPlaneIntersection(plane_a, plane_b, line, 1e-6);
		pcl::planeWithPlaneIntersection(plane_b, plane_c, line1, 1e-6);

		std::cout << "XLPE与反应力锥相交直线为：\n" << line << endl;   //(line[0],line[1],line[2])为所求直线线上一点，（line[3],line[4],line[5]）为直线的法向量
		std::cout << "反应力锥与内半导电相交直线为：\n" << line1 << endl;
		//cout << "验证所求交线在两平面上： " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;

		/*平面及平面交线可视化*/
		//	////可视化
		//	//pcl::visualization::PCLVisualizer viewer;
		//	////可视化平面
		//	//pcl::ModelCoefficients plane, plane1;
		//	//for (size_t k = 0; k < 4; ++k)
		//	//{
		//	//	plane.values.push_back(plane_a[k]);
		//	//	plane1.values.push_back(plane_b[k]);
		//	//}
		//	//viewer.addPlane(plane, "plane", 0);
		//	//viewer.addPlane(plane1, "plane1", 0);
		//	////可视化平面交线
		//	//pcl::PointXYZ p1, p2, p3, p4;
		//	//p1.x = line[0]; p1.y = line[1]; p1.z = line[2];
		//	//p2.x = line[3]; p2.y = line[4]; p2.z = line[5];
		//	//viewer.addLine(p1, p2, 1, 0, 0, "line", 0);
		//	//while (!viewer.wasStopped())
		//	//{
		//	//	viewer.spinOnce(100);
		//	//}
		//	

		/*计算空间一点到直线的距离*/
		Eigen::Vector4f point, point1;
		//point << line[0], line[1], line[2], 0;     //直线上一点
		point << line[0], line[1], line[2];     //直线上一点想，x,y,z
		point1 << line1[0], line1[1], line1[2];     //直线上一点想，x,y,z

		Eigen::Vector4f normal, normal1;
		normal << line[3], line[4], line[5];		//直线的方向向量
		normal1 << line1[3], line1[4], line1[5];		//直线的方向向量

		std::vector<float> point_2_line_distance; point_2_line_distance.clear();
		std::vector<float> point_2_line1_distance; point_2_line1_distance.clear();

		for (int k = 0; k < m_index_strip.at(i).each_strip->size(); k++)
		{
			float distance = 0;
			float distance1 = 0;
			Eigen::Vector4f point_cloud_point;
			point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
			distance = pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// 空间点到空间直线距离，参数分别为点云中一点，直线上一点，直线的法向量
			distance1 = pcl::sqrPointToLineDistance(point_cloud_point, point1, normal1);
			point_2_line_distance.push_back(distance);
			point_2_line1_distance.push_back(distance1);

		}
		float small_distance = point_2_line_distance.at(0);
		int index_of_small_distance = 0;
		for (int k = 0; k<point_2_line_distance.size(); k++)//找出当前条中哪一个点离所求直线最近
		{
			if (point_2_line_distance.at(k) < small_distance)
			{
				small_distance = point_2_line_distance.at(k);
				index_of_small_distance = k;
			}
		}

		float small_distance1 = point_2_line1_distance.at(0);
		int index_of_small_distance1 = 0;
		for (int k = 0; k < point_2_line1_distance.size(); k++)//找出当前条中哪一个点离所求直线最近
		{
			if (point_2_line1_distance.at(k) < small_distance1)
			{
				small_distance1 = point_2_line1_distance.at(k);
				index_of_small_distance1 = k;
			}
		}

		std::cout << "校正后该小条所得95mm点位z值：" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
		std::cout << "反应力锥校正前测量值 = " << cable_z_max_value - line[2] << endl;
		std::cout << "反应力锥校正后测量值 = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
		std::cout << "校正后该小条所得40mm点位z值：" << m_index_strip.at(i).each_strip->at(index_of_small_distance1).z << std::endl;
		std::cout << "内半导电起点校正前测量值 = " << cable_z_max_value - line1[2] << endl;
		std::cout << "内半导电起点校正后测量值 = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance1).z << endl;
	}
	/***************************************************************平面拟合求交线***************************************************************************************************/

	/*取一条出来对每小片进行平面估计，然后计算轴线夹角*/
	pcl::Normal axis_normal;
	axis_normal.normal_x = 0;  //指定轴线为z轴
	axis_normal.normal_y = 0;
	axis_normal.normal_z = 1;
	std::vector<float> one_strip_slice_angle; one_strip_slice_angle.clear();
	for (int i = 0; i < m_cable_strip_slice_all.at(0).size(); i++)  //对某一条中的小片轴线夹角进行计算
	{
		float A, B, C, D; A = B = C = D = 0;
		ransac_plane_estimation(m_cable_strip_slice_all.at(0).at(i).pointcloud_strip_slice, A, B, C, D);  //对小片进行平面估计，从而得出平面法线
		pcl::Normal once_slice_normal;
		once_slice_normal.normal_x = A; once_slice_normal.normal_y = B; once_slice_normal.normal_z = C;
		//double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z);  //计算夹角余弦值
		double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z) /
			sqrt(pow(once_slice_normal.normal_x, 2) + pow(once_slice_normal.normal_y, 2) + pow(once_slice_normal.normal_x, 2));  //计算夹角余弦值
		double inverse_cosine_value = acos(cos_theta) * 180 / M_PI;  //夹角反余弦，也就是算出相邻法线之间的夹角
		one_strip_slice_angle.push_back(180 - inverse_cosine_value);  //计算轴线夹角
	}

	/*小片轴线夹角数据写出*/
	//FILE * fp8;
	//if ((fp8 = fopen("Angeles_change.txt", "wb")) == NULL)
	//{
	//	printf("cant open the file");
	//	exit(0);
	//}
	//for (size_t i = 0; i < one_strip_slice_angle.size(); i++)
	//{
	//	fprintf(fp8, "%f ", one_strip_slice_angle.at(i));
	//	fprintf(fp8, "%s ", "\n");
	//}
	//fclose(fp8);
	//cout << "小片轴线夹角输出完成!" << endl;



	return true;
}
/*粗分割初步得到区域的起点与终点后，将终点前后k/2的圆拿出来融合，然后以更小z值进行拟合圆*/
//circle_number，区域交接点的编号 ，
//上一个标准差分割使用的前后圆的个数field
//flag  1： 从下到circle_number 2：从下往上 3：从circle_number到上
bool Cable::fusion_iteration_detail_segmentation(int circle_number, int field, int flag, std::vector<Circle> &region_circles)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_circle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //用来存上一次标准差之比找出分界点前（后）多少个圆的点
	region_circle_cloud->clear();

	if (flag == 1)		//取粗分割起点及起点前面的圆出来（内半导电层与线芯交界）
	{
		for (int i = field; i >= 0; i--)    //控制临近field个圆
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number - i).each_circle_index.size(); j++) //临近前i某个圆内的点
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number - i).each_circle_points->at(j));   //将指定的临近圆内的每个点进行存储
			}
		}
	}


	if (flag == 2)   //取粗分割前后的圆出来
	{
		for (int i = field; i >= 0; i--)     //取circle_number前面的圆
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number - i).each_circle_index.size(); j++)  //临近前i某个圆内的点
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number - i).each_circle_points->at(j));   //将指定的临近圆内的每个点进行存储
			}
		}

		for (int i = 1; i <= field; i++)    //取circle_number后面的圆
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number + i).each_circle_index.size(); j++)  //临近某个圆内的点
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number + i).each_circle_points->at(j));   //将指定的临近圆内的每个点进行存储
			}
		}
	}

	if (flag == 3)		//取粗分割后面的圆出来
	{
		for (int i = 1; i <= field; i++)    //取circle_number后面的圆
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number + i).each_circle_index.size(); j++)  //临近某个圆内的点
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number + i).each_circle_points->at(j));   //将指定的临近圆内的每个点进行存储
			}
		}
	}


	//将重新获取到的电缆局部点云进行圆拟合
	if (region_circle_cloud->empty())			//判断输入的点云是否为空
	{
		std::cout << "region point cloud is empty!" << std::endl;
		return false;
	}

	rankpointcloud_z(region_circle_cloud);		//区域点云排序
	pcl::PointXYZRGB srart_point;
	pcl::PointXYZRGB update_point;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_circle_point(new pcl::PointCloud<pcl::PointXYZRGB>);
	srart_point = region_circle_cloud->at(0);
	double region_point_z_min = region_circle_cloud->at(0).z;		//点云文件排序后的的z轴最低点点云文件的z值
	double region_point_z_max = region_circle_cloud->at(region_circle_cloud->size() - 1).z;		//排序后点云的最高点的z值
	region_circles.clear();

	for (double circle_height = region_point_z_min; circle_height < region_point_z_max; circle_height += 0.2)   //根据z值遍历整个小区域，每k个z值生成一个圆
	{
		update_point.x = srart_point.x;
		update_point.y = srart_point.y;
		update_point.z = circle_height;
		region_circle_point->clear();
		for (int i = 0; i < region_circle_cloud->size(); i++)
		{
			double ciecle_threshold = region_circle_cloud->at(i).z - update_point.z;
			if (ciecle_threshold < 0.1 && ciecle_threshold > -0.1)
			{
				region_circle_point->push_back(region_circle_cloud->at(i));//存储范围内的点云
			}
		}
		if (region_circle_point->size())
		{
			Circle circle_temp;
			ransice_circular_fitting(region_circle_point, circle_temp);  //ransic圆拟合
			Circle once_circle;
			once_circle.m_center_x = circle_temp.m_center_x;
			once_circle.m_center_y = circle_temp.m_center_y;
			once_circle.m_center_z = circle_temp.m_center_z;
			once_circle.m_radius = circle_temp.m_radius;
			region_circles.push_back(once_circle);  //把每一个拟合圆存起来
		}
	}
	return true;
}
/*起点精分割z值求取 back/fore*/
bool Cable::update_segmentation_start(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value)
{
	std::vector<double> start_back_compare_fore_all;
	double standard_fore_start, standard_back_start;
	double standard_back_compare_fore_start;
	start_back_compare_fore_all.clear();
	for (int i = 0; i < k; i++)  //为了与拟合圆数量相等，前面填k个0
	{
		start_back_compare_fore_all.push_back(0);
	}

	for (int i = k; i < region_circles.size() - k; i++)
	{
		standard_fore_start = standard_back_start = standard_back_compare_fore_start = 0;
		calculation_standard_deviation(i, k, standard_fore_start, standard_back_start);
		standard_back_compare_fore_start = standard_back_start / standard_fore_start;
		start_back_compare_fore_all.push_back(standard_back_compare_fore_start);

	}

	for (int i = 0; i < k; i++)	//为了与拟合圆数量相等，后面填k个0
	{
		start_back_compare_fore_all.push_back(0);
	}

	double max_variance = start_back_compare_fore_all.at(0);   //找出最大标准差
	int flag_update = 0;
	for (int i = 0; i < start_back_compare_fore_all.size(); i++)
	{
		if (start_back_compare_fore_all.at(i) > max_variance)
		{
			max_variance = start_back_compare_fore_all.at(i);
			flag_update = i;
		}
	}
	update_region_z_value = region_circles.at(flag_update).m_center_z - cable_z_minimum_value;
	return true;
}
/*点精分割z值求取  fore/back*/
bool Cable::update_segmentation_end(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value)  //k是第一次分割进行标准差之比时使用的前后圆数量
{
	std::vector<double> fore_compare_back_region_end;
	double Variance_fore_update, Variance_back_update;
	double  Variance_fore_compare_back_region_end_update;
	fore_compare_back_region_end.clear();
	for (int i = 0; i < k; i++)  //为了与拟合圆数量相等，前面填k个0
	{
		fore_compare_back_region_end.push_back(0);
	}

	for (int i = k; i < region_circles.size() - k; i++)
	{
		Variance_fore_update = Variance_back_update = Variance_fore_compare_back_region_end_update = 0;
		calculation_standard_deviation(i, k, Variance_fore_update, Variance_back_update);
		Variance_fore_compare_back_region_end_update = Variance_fore_update / Variance_back_update;
		fore_compare_back_region_end.push_back(Variance_fore_compare_back_region_end_update);

	}
	for (int i = 0; i < k; i++)	//为了与拟合圆数量相等，后面填k个0
	{
		fore_compare_back_region_end.push_back(0);
	}

	double max_variance_region_end = fore_compare_back_region_end.at(0);
	int flag_update_region_end = 0;
	for (int i = 0; i < fore_compare_back_region_end.size(); i++)
	{
		if (fore_compare_back_region_end.at(i) > max_variance_region_end)
		{
			max_variance_region_end = fore_compare_back_region_end.at(i);
			flag_update_region_end = i;
		}

	}
	update_region_z_value = region_circles.at(flag_update_region_end).m_center_z - cable_z_minimum_value;

	return true;
}
/*对电缆各部分进行尺寸测量*/
/*电缆分割起点存储*/
//region_start.push_back(first_start_z_value);  //外半导电过渡带迭代后起点的z值
//region_start.push_back(second_start_update + cable_z_mini_value);  //第一个铅笔头起点的z值
//region_start.push_back(thrid_start_update + cable_z_mini_value);  //第二个铅笔头起点的z值
//region_start.push_back(inner_semiconducting_start_z_value_finally);  //内半导电的起点，也是第二个铅笔头终点的z值
//region_start.push_back(third_end_disance + cable_z_mini_value);  //线芯起点的z值，也是内半导电终点z值
/*电缆分割终点存储*/
//region_end.push_back(first_end_update);  //外半导电过渡带终点的z值,迭代一次
//region_end.push_back(second_end_update + cable_z_mini_value);  //第一个铅笔头终点距离线缆最低点的z值
//region_end.push_back(inner_semiconducting_start_z_value_finally);  //第二个铅笔头终点的z值
//region_end.push_back(third_end_disance + cable_z_mini_value);  //内半导电终点的z值
//region_end.push_back(cable_z_max_value);  //线芯终点的z值	
Parameter_measurement Cable::segmentation_region_size_measurement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<double> region_start, std::vector<double> region_end)
{
	std::cout << "正在根据分割结果对电缆参数进行测量！..." << endl;
	double cable_z_max_value = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z; //把最后一个拟合圆的z值作为电缆的最高点

	Parameter_measurement cable_region_sizz_measurement;
	cable_region_sizz_measurement.conductor_size = region_end.at(region_end.size() - 1) - region_start.at(region_start.size() - 1);		//1.线芯导体的长度，标准为30mm
	cable_region_sizz_measurement.core_pencil_head_size = region_end.at(region_end.size() - 1) - region_start.at(2);					//2.靠近线芯铅笔头的长度，标准为95mm
	cable_region_sizz_measurement.inner_semiconducting_size = region_end.at(3) - region_start.at(3);									//3.内半导电层的宽度，标准为10mm
	cable_region_sizz_measurement.outside_pencil_head_size = region_end.at(1) - region_start.at(1);										//4.远离线芯的铅笔头这个下坡的尺寸，标准为30mm
	cable_region_sizz_measurement.core_top_2_outside_pencil_head_distance = region_end.at(region_end.size() - 1) - region_start.at(1);	//5.远离线芯铅笔头起点距离线芯顶端的距离，标准为300mm
	cable_region_sizz_measurement.core_outside_semiconducting_end = region_end.at(region_end.size() - 1) - region_end.at(0);			//6.外半导电过渡带终点距离线芯顶端600mm
	cable_region_sizz_measurement.outside_semiconducting_size = region_end.at(0) - region_start.at(0);									//7.外半导电过渡带的尺寸,标准为40mm

																																		/*下面对：8.主绝缘绝缘外径偏差，9.线芯末端测量18cm位置处的绝缘外径进行测量，10.导体松散变形*/

																																		//cable_region_sizz_measurement.main_insulation_deviation = main_insulation_difference; //8.外半导电过渡带终点后面取4个圆，计算最大值与最小值之间的半径差
	std::vector<int> cable_core_region_radius;   //存储线芯区域圆的索引
	cable_core_region_radius.clear();
	std::vector<int> region_circle_index_15_2_22; //存储距离线芯顶端16cm至20cm圆的索引
	region_circle_index_15_2_22.clear();
	double thin_insulation_circle;				 //存储距离线芯顶端18cm处的半径
	std::vector<int> main_insulation_circle_index; //存储主绝缘处圆的索引350-580
	main_insulation_circle_index.clear();
	for (int i = 0; i < m_indexandcircles.size(); i++)
	{
		//找出距离线芯顶端15cm-22处圆的索引
		if (region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z > 150 && region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z < 220)
		{
			region_circle_index_15_2_22.push_back(i);
		}
		//找出线芯区域圆的索引
		if (m_indexandcircles.at(i).once_circle.m_center_z <= region_end.at(region_end.size() - 1) && m_indexandcircles.at(i).once_circle.m_center_z > region_start.at(region_start.size() - 1))
		{
			cable_core_region_radius.push_back(i);
		}
		//找出电缆主绝缘距离线芯顶端350-580区域圆的索引
		if (region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z > 350 && region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z < 580)
		{
			main_insulation_circle_index.push_back(i);
		}

	}

	/*****************************************8.计算距离线芯订单35cm-58cm处线缆的绝缘外径偏差**************************************************/
	//std::vector<double> insulation_outer_diameter_deviation;
	//insulation_outer_diameter_deviation.clear();
	//std::vector<double> once_circle_deviation;
	//double max_outer_diameter_difference;
	//for (int i = 0; i < main_insulation_circle_index.size(); i += 10)  //不对每一个圆的绝缘外径差都判断，隔10个判断一个
	//{
	//	max_outer_diameter_difference = 0;
	//	once_circle_deviation.clear();
	//	std::vector<Index_Strip> pointcloud_2_strip_all_main_insulation_one_circle;
	//	pointcloud_2_strip(m_indexandcircles.at(main_insulation_circle_index.at(i)).each_circle_points, pointcloud_2_strip_all_main_insulation_one_circle,90);  //按45度把圆抽成4部分，分别拟合成圆，通过比较半径的差距来找出不不圆度
	//	for (int j = 0;j<pointcloud_2_strip_all_main_insulation_one_circle.size();j++)
	//	{
	//		Circle circle;
	//		ransic_circle_estimation(pointcloud_2_strip_all_main_insulation_one_circle.at(j).each_strip, circle);
	//		once_circle_deviation.push_back(circle.m_radius);
	//	}
	//	sort(once_circle_deviation.begin(), once_circle_deviation.end());
	//	max_outer_diameter_difference = once_circle_deviation.at(once_circle_deviation.size() - 1) - once_circle_deviation.at(0);  //对4部分拟合的圆进行最大半径与最小半径作差
	//	insulation_outer_diameter_deviation.push_back(max_outer_diameter_difference);  //把测的每一个圆的最大偏差都存起来
	//}
	//sort(insulation_outer_diameter_deviation.begin(), insulation_outer_diameter_deviation.end());
	//cable_region_sizz_measurement.main_insulation_deviation = insulation_outer_diameter_deviation.at(insulation_outer_diameter_deviation.size() - 1 );  //8.计算同一截面的绝缘外径，计算最大值与最小值之间的半径差（将一个圆的点拆成4分，然后分别拟合）,这里隔10个圆计算一个

	/*通过拟合椭圆*/
	std::vector<double> insulation_outer_diameter_deviation;//主绝缘直径
	insulation_outer_diameter_deviation.clear();
	std::vector<double> once_circle_deviation;
	double max_outer_diameter_difference;
	for (int i = 0; i < main_insulation_circle_index.size(); i += 10)  //不对每一个圆的绝缘外径差都判断，隔10个判断一个
	{
		std::vector<double> ellipse;
		ellipse = getEllipseparGauss(m_indexandcircles.at(main_insulation_circle_index.at(i)).each_circle_points);
		double  ellipse_a = ellipse.at(2);  //椭圆长轴
		double ellipse_b = ellipse.at(3);	//椭圆短轴
		double a_b_dfference = (2 * ellipse_a) - (2 * ellipse_b);

		insulation_outer_diameter_deviation.push_back(a_b_dfference);  //把测的每一个椭圆的最大偏差都存起来
	}
	sort(insulation_outer_diameter_deviation.begin(), insulation_outer_diameter_deviation.end());
	//8.计算同一截面的绝缘外径，计算最大值与最小值之间的半径差,这里隔10个圆计算一个
	cable_region_sizz_measurement.main_insulation_deviation = insulation_outer_diameter_deviation.at(insulation_outer_diameter_deviation.size() - 1);

	/*****************************************************9.测量距离线芯顶端15cm-22cm处圆的半径,标准为65mm，绝缘外径等于半径乘2********************************************/
	//计算绝缘外径范围
	std::vector<double> main_insulation_radius_15_2_22;
	main_insulation_radius_15_2_22.clear();
	for (int i = 0; i <region_circle_index_15_2_22.size(); i++)
	{
		main_insulation_radius_15_2_22.push_back(m_indexandcircles.at(region_circle_index_15_2_22.at(i)).once_circle.m_radius);
	}
	sort(main_insulation_radius_15_2_22.begin(), main_insulation_radius_15_2_22.end());
	cable_region_sizz_measurement.insulation_core_size_18_min = main_insulation_radius_15_2_22.at(0) * 2;
	cable_region_sizz_measurement.insulation_core_size_18_max = main_insulation_radius_15_2_22.at(main_insulation_radius_15_2_22.size() - 1) * 2;

	/*****************************************************10.（方法1）导体松散变形,*********************************************************************************/
	//double standard_parts_core_standard_value = 5.9122980166409684;   //用标准件（对方的normal）测量的线芯区域所有半径的标准差，只要标准差大于该值，说明导体就已经出现松散变形
	//double cable_core_region_radius_value_sum = 0;
	//double cable_core_region_radius_value_average = 0;
	//double variance_deviation_molecule = 0;
	//double variance_deviation = 0;
	//double standard_deviation = 0;
	//for (int i = 0;i < cable_core_region_radius.size();i++)  //计算线芯区域所有圆的半径之和
	//{
	//	cable_core_region_radius_value_sum = cable_core_region_radius_value_sum + m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius;
	//}
	//cable_core_region_radius_value_average = cable_core_region_radius_value_sum / cable_core_region_radius.size() ;   //计算电缆线芯区域所有半径和的均值
	//for (int j = 0; j < cable_core_region_radius.size(); j++)  //计算方差的分子
	//{
	//	double variance_temp = pow(((m_indexandcircles.at(cable_core_region_radius.at(j)).once_circle.m_radius) - cable_core_region_radius_value_average), 2);
	//	variance_deviation_molecule = variance_deviation_molecule + variance_temp;
	//}
	//variance_deviation = variance_deviation_molecule / cable_core_region_radius.size();  //线芯区域圆半径的方差
	//standard_deviation = sqrt(variance_deviation);   //线芯区域圆半径的标准差
	//int loose_flag = 0;
	//if (standard_deviation > (standard_parts_core_standard_value ) )  //对实际打磨电缆的线芯区域的圆拿出来，计算标准差，如果标准差大于标准件线芯区域标准差（留了0.5个余量），则证明线芯出现了松散
	//{
	//	loose_flag = 1;	
	//}
	//cable_region_sizz_measurement.conductor_deformation = loose_flag;   //10.通过对打磨电缆线芯区域所有圆半径计算标准差，然后与标准件这部分圆的标准差做比较，判断线芯是否有松散变形的趋势，0不松散，1松散	

	//**************************************************10.导体松散变形 11.找线芯切面倾斜***********************************************************************************
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_pointcloud->clear();
	std::vector<double>cable_core_circle_radius;  //把线芯部分圆的半径拿出来
	cable_core_circle_radius.clear();
	std::vector<Circle> cable_core_circle_center;
	cable_core_circle_center.clear();

	for (int i = 0; i < cable_core_region_radius.size(); i++)
	{
		for (int j = 0; j < m_indexandcircles.at(cable_core_region_radius.at(i)).each_circle_points->size(); j++)   //将线芯区域所有圆的点集中起来
		{
			cable_core_pointcloud->push_back(m_indexandcircles.at(cable_core_region_radius.at(i)).each_circle_points->at(j));
		}
		cable_core_circle_radius.push_back(m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius);    //将线芯区域所有圆的半径集中起来，为了得出线芯区域的均值半径

		Circle core_circle;   //存储圆心
		core_circle.m_center_x = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_x;   //将线芯区域所有圆的圆心集中起来，为了拟合线芯区域的轴线
		core_circle.m_center_y = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_y;
		core_circle.m_center_z = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_z;
		core_circle.m_radius = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius;
		cable_core_circle_center.push_back(core_circle);
	}

	sort(cable_core_circle_radius.begin(), cable_core_circle_radius.end());
	double core_circle_radius = 0;
	int remain_radius_count = 0;
	double core_circle_radius_average = 0;//线芯区域半径均值
	for (int i = 1; i<cable_core_circle_radius.size() - 1; i++) //去除了最大值与最小值后求均值
	{
		remain_radius_count++;
		core_circle_radius = core_circle_radius + cable_core_circle_radius.at(i);
	}
	core_circle_radius_average = core_circle_radius / remain_radius_count;  //线芯处半径去除最大值与最小值后的均值

	double core_circle_x_sum = 0;
	double core_circle_y_sum = 0;
	double core_circle_x_average = 0;//线芯区域圆心均值x
	double core_circle_y_average = 0;

	for (int i = 0; i < cable_core_circle_center.size(); i++)
	{
		core_circle_x_sum = core_circle_x_sum + cable_core_circle_center.at(i).m_center_x;
		core_circle_y_sum = core_circle_y_sum + cable_core_circle_center.at(i).m_center_y;
	}
	core_circle_x_average = core_circle_x_sum / cable_core_circle_center.size();  //计算线芯处所有圆圆心x,y的均值
	core_circle_y_average = core_circle_y_sum / cable_core_circle_center.size();


	/*************************** 10.（方法2）找线芯松散*****************************************/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_loose_defect_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_loose_normal_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	core_loose_defect_points->clear(); core_loose_normal_points->clear();
	double point_2_line_distnace_deviation = 0;//总偏差值
	int counter = 0;
	for (int i = 0; i < cable_core_pointcloud->size(); i++)   //计算线芯区域所有点与平均圆心（x，y）之间的距离，不考虑z值
	{
		double once_point_2_center_distnace = sqrt(pow((cable_core_pointcloud->at(i).x - core_circle_x_average), 2) + pow((cable_core_pointcloud->at(i).y - core_circle_y_average), 2));
		//if (once_point_2_center_distnace > core_circle_radius_average)  //如果点到平均圆心距离大于线芯平均半径，那么久将该值减去平均半径，并记录偏差值
		if (once_point_2_center_distnace - core_circle_radius_average > 1)
		{
			counter++;
			point_2_line_distnace_deviation = point_2_line_distnace_deviation + (once_point_2_center_distnace - core_circle_radius_average);
			cable_core_pointcloud->at(i).r = 0;	cable_core_pointcloud->at(i).g = 255;	cable_core_pointcloud->at(i).b = 0;
			core_loose_defect_points->push_back(cable_core_pointcloud->at(i));	//线芯松散点云存储
		}
		else
		{
			core_loose_normal_points->push_back(cable_core_pointcloud->at(i));   //线芯松散非松散点云存储
		}

	}
	//showpointcloud_xyzrgb(cable_core_pointcloud);  //将那些大于半径+1的点进行显示

	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_loose_defect_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_loose_normal_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//xyzrgb_2_xyz(core_loose_defect_points, core_loose_defect_points_xyz);
	//xyzrgb_2_xyz(core_loose_normal_points, core_loose_normal_points_xyz);
	//defect_generate_obj_loose(core_loose_defect_points_xyz, core_loose_normal_points_xyz);   //局部点云显示，并显示缺陷


	double point_average = point_2_line_distnace_deviation / counter;
	double standard_parts_core_standard_value = 1.1947611367863262;  //对方给的normal电缆计算出来的值
	int loose_flag = 0;
	if (point_average > standard_parts_core_standard_value)  //判断点平均残差，如果大于标准件的，则证明线芯出现了松散
	{
		loose_flag = 1;
	}
	cable_region_sizz_measurement.conductor_deformation = loose_flag;		//10. conductor_deformation标志，0不松散，1松散	 

	std::vector<Index_Strip> pointcloud_2_strip_all;						//存条和条索引的容器
	pointcloud_2_strip(cable_core_pointcloud, pointcloud_2_strip_all, 5);	//对线芯部分的点云进行抽条,五度
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_top_plane_pointcloud->clear();
	for (int i = 0; i < pointcloud_2_strip_all.size(); i++)
	{
		for (int j = 0; j < pointcloud_2_strip_all.at(i).each_strip->size(); j++)
		{
			if (pointcloud_2_strip_all.at(i).each_strip->at(pointcloud_2_strip_all.at(i).each_strip->size() - 1).z - pointcloud_2_strip_all.at(i).each_strip->at(j).z  < 5)	//取出每条顶端上面1mm内的点，值取大一点，避免深坑处的点取不到
			{
				cable_core_top_plane_pointcloud->push_back(pointcloud_2_strip_all.at(i).each_strip->at(j));   //把每条最上面5mm内的点取出来
			}
		}
	}
	//showpointcloud_xyzrgb(cable_core_top_plane_pointcloud);			//显示顶部，包含四周

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_uneven_defect_points(new pcl::PointCloud<pcl::PointXYZRGB>);  //存储线芯顶面点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_uneven_normal_points(new pcl::PointCloud<pcl::PointXYZRGB>);  //存储线芯侧面点云
	core_uneven_defect_points->clear();  core_uneven_normal_points->clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delete(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_top_plane_pointcloud_delete->clear();
	double distance = 0;  //下面将取出来的顶端面周围点去除，按与线芯所有圆半径均值的关系
	for (int i = 0; i < cable_core_top_plane_pointcloud->size(); i++)
	{
		distance = sqrt(pow((cable_core_top_plane_pointcloud->at(i).x - core_circle_x_average), 2) + pow((cable_core_top_plane_pointcloud->at(i).y - core_circle_y_average), 2)); //计算两点距离)
		if (distance + 3  < core_circle_radius_average)  //多了4mm余量，是为了确保周围点都能去除干净
		{
			cable_core_top_plane_pointcloud_delete->push_back(cable_core_top_plane_pointcloud->at(i));   //去除线芯顶部误找出来的圆周上的点，找出真正线芯顶面的点
			core_uneven_defect_points->push_back(cable_core_top_plane_pointcloud->at(i));        //存储线芯顶面点云***************************
		}
		else
		{
			core_uneven_normal_points->push_back(cable_core_top_plane_pointcloud->at(i));       //存储线芯侧面点云****************
		}
	}
	//showpointcloud_xyzrgb(cable_core_top_plane_pointcloud_delete);	//显示顶部，去除了四周

	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_uneve_defect_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_uneve_normal_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//xyzrgb_2_xyz(core_uneven_defect_points, core_uneve_defect_points_xyz);
	//xyzrgb_2_xyz(core_uneven_normal_points, core_uneve_normal_points_xyz);
	//defect_generate_obj(core_uneve_defect_points_xyz, core_uneve_normal_points_xyz);   //局部点云显示，并显示缺陷


	Plane_parameter plane_parameter;
	ransic_plane_estimation(cable_core_top_plane_pointcloud_delete, plane_parameter); //对线芯顶部平面使用ransic法进行平面拟合，求出平面法线
	pcl::PointNormal core_top_plane_normal; //线芯最顶端的平面法线
	core_top_plane_normal.normal_x = plane_parameter.plane_parameter_A;
	core_top_plane_normal.normal_y = plane_parameter.plane_parameter_B;
	core_top_plane_normal.normal_z = plane_parameter.plane_parameter_C;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_circle_center_pointcloud->clear();
	for (int i = 0; i < cable_core_circle_center.size(); i++)   //将线芯区域圆的圆心取出来，并对其上色，生成一个点云，后面做直线拟合
	{
		pcl::PointXYZRGB point_temp;
		point_temp.x = cable_core_circle_center.at(i).m_center_x;
		point_temp.y = cable_core_circle_center.at(i).m_center_y;
		point_temp.z = cable_core_circle_center.at(i).m_center_z;
		point_temp.r = 255;
		point_temp.g = 0;
		point_temp.b = 0;
		cable_core_circle_center_pointcloud->push_back(point_temp);
	}
	//showpointcloud_xyzrgb(cable_core_circle_center_pointcloud);		//显示线芯区域所有圆的圆心

	Core_circle_center_line core_circle_center_line;
	ransic_line_estimation(cable_core_circle_center_pointcloud, core_circle_center_line);  //线芯圆心做直线拟合
	pcl::PointNormal core_circle_center_line_normal; //线芯圆心直线的方向法线
	core_circle_center_line_normal.normal_x = core_circle_center_line.core_circle_center_line_x;
	core_circle_center_line_normal.normal_y = core_circle_center_line.core_circle_center_line_y;
	core_circle_center_line_normal.normal_z = core_circle_center_line.core_circle_center_line_z;

	float core_top_plane_core_circle_center_angel = 0;
	Eigen::Vector3f v1(core_top_plane_normal.normal_x, core_top_plane_normal.normal_y, core_top_plane_normal.normal_z);   //线芯顶面法线
	Eigen::Vector3f v2(core_circle_center_line_normal.normal_x, core_circle_center_line_normal.normal_y, core_circle_center_line_normal.normal_z);  //线芯区域圆心直线的方向法线
	core_top_plane_core_circle_center_angel = pcl::getAngle3D(v1, v2, true);  //计算两法线夹角，true为角度表示，false为弧度表示

	double Section_inclination = 0;
	double angle_2_radian = core_top_plane_core_circle_center_angel / (M_PI * 180);  //角度转为弧度
	Section_inclination = tan(angle_2_radian) * (2 * core_circle_radius_average); //导体切面倾斜程度等于tan（theta）*2r
	cable_region_sizz_measurement.conductor_section_inclination = Section_inclination;   //11.导体切面倾斜*********************************************


																						 //***********************************************12.线芯切面凹凸不平**************************************************************************************
	double standard_parts_standard_value = 0.14506740222996123;  //标准件线芯顶部所有点到拟合平面的残差和的均值,因为每个件顶部点数不一致，所以最好求取点均值残差，此外加了2mm的余量
	double residual_sum_top_plane_all_point = 0;
	double one_point_2_plane_distance_molecule = 0;
	double one_point_2_plane_distance_denominator = 0;
	double one_point_2_plane_distance = 0;

	for (int i = 0; i < cable_core_top_plane_pointcloud_delete->size(); i++)
	{
		one_point_2_plane_distance_molecule = plane_parameter.plane_parameter_A * cable_core_top_plane_pointcloud_delete->at(i).x + plane_parameter.plane_parameter_B *
			cable_core_top_plane_pointcloud_delete->at(i).y + plane_parameter.plane_parameter_C * cable_core_top_plane_pointcloud_delete->at(i).z + plane_parameter.plane_parameter_D;    //点到平面距离的分子
		one_point_2_plane_distance_denominator = sqrt(pow(plane_parameter.plane_parameter_A, 2) + pow(plane_parameter.plane_parameter_B, 2) + pow(plane_parameter.plane_parameter_C, 2)); //点到平面距离的分母
		one_point_2_plane_distance = abs(one_point_2_plane_distance_molecule / one_point_2_plane_distance_denominator);  //点到平面的距离,取绝对值		
		residual_sum_top_plane_all_point = residual_sum_top_plane_all_point + one_point_2_plane_distance;  //线芯顶部所有点到平面距离的残差和
	}
	double core_top_plane_point_residual_average = residual_sum_top_plane_all_point / cable_core_top_plane_pointcloud_delete->size();  //计算线芯顶部点残差均值
	int top_plane_flat_flag = 0;  //0表示顶部是平，否则认为是凹凸不平的
	if (core_top_plane_point_residual_average > standard_parts_standard_value)  //留有2的残差余量
	{
		top_plane_flat_flag = 1;
	}
	cable_region_sizz_measurement.core_top_uneven = top_plane_flat_flag;  //12.导体切面凹凸不平，计算标准件切面的标准差，然后以此为阈值，后面打磨的大于这个值就认为是凹凸不平的,为1就是凹凸不平，为0就是正常，留有余量2

																		  //***********************************************13.距离线芯顶端200mm距离线芯顶端265mm处圆直径的差值**************************************************************************************
	std::vector<int> distance_core_max_value_200_diameter_index; distance_core_max_value_200_diameter_index.clear(); //存储距离线芯顶端198mm-202mm线芯的圆的索引
	std::vector<int> distance_core_max_value_265_diameter_index; distance_core_max_value_265_diameter_index.clear(); //存储距离线芯顶端263mm-267mm线芯的圆的索引
	double diameter_200_sum = 0;	//存储距离线芯顶端200mm左右几个圆直径的总和
	double diameter_265_sum = 0;	//存储距离线芯顶端265mm左右几个圆直径的总和
	for (int i = 0; i < m_indexandcircles.size(); i++)
	{

		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z >= 199
			&& cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z <= 201)
		{
			distance_core_max_value_200_diameter_index.push_back(i);
			diameter_200_sum = diameter_200_sum + m_indexandcircles.at(i).once_circle.m_radius * 2;
		}
		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z >= 264
			&& cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z <= 266)
		{
			distance_core_max_value_265_diameter_index.push_back(i);
			diameter_265_sum = diameter_265_sum + m_indexandcircles.at(i).once_circle.m_radius * 2;
		}
	}

	double distance_core_200_diameter = diameter_200_sum / distance_core_max_value_200_diameter_index.size();
	double distance_core_265_diameter = diameter_265_sum / distance_core_max_value_265_diameter_index.size();
	cable_region_sizz_measurement.outer_diameter_difference_200_265 = abs(distance_core_200_diameter - distance_core_265_diameter); //13.距离线芯顶端200mm距离线芯顶端265mm处圆直径的差值


	cout << "线芯长度：" << cable_region_sizz_measurement.conductor_size << endl;
	cout << "线芯铅笔头长度：" << cable_region_sizz_measurement.core_pencil_head_size << endl;
	cout << "内半导电长度：" << cable_region_sizz_measurement.inner_semiconducting_size << endl;
	cout << "第二个铅笔头长度：" << cable_region_sizz_measurement.outside_pencil_head_size << endl;
	cout << "第二个铅笔头距离线芯顶端长度：" << cable_region_sizz_measurement.core_top_2_outside_pencil_head_distance << endl;
	cout << "外半导电过渡带末端距离线芯顶端长度：" << cable_region_sizz_measurement.core_outside_semiconducting_end << endl;
	cout << "外半导电过渡带长度：" << cable_region_sizz_measurement.outside_semiconducting_size << endl;
	cout << "主绝缘外径偏差：" << cable_region_sizz_measurement.main_insulation_deviation << endl;
	cout << "距离线芯顶端15cm-220cm处绝缘外径的范围：" << cable_region_sizz_measurement.insulation_core_size_18_min << " - " << cable_region_sizz_measurement.insulation_core_size_18_max << endl;
	cout << "导体松散变形：" << cable_region_sizz_measurement.conductor_deformation << endl;
	cout << "导体切面倾斜：" << cable_region_sizz_measurement.conductor_section_inclination << endl;
	cout << "导体切面凹凸：" << cable_region_sizz_measurement.core_top_uneven << endl;
	cout << "距离线芯顶端200mm处圆外径与265mm处圆绝缘外径的差值：" << cable_region_sizz_measurement.outer_diameter_difference_200_265 << endl;

	return cable_region_sizz_measurement;
}
/*抽条函数，将点云按角度抽成一条一条的的点云*/
bool Cable::pointcloud_2_strip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud, std::vector<Index_Strip> &pointcloud_2_strip_all, int strip_angle)
{
	rankpointcloud_z(cable_core_pointcloud);
	//std::cout << "抽条函数被调用" << endl;
	if (cable_core_pointcloud->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	std::vector<int> index_strip;  //存索引
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的一部分点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的剩余部分点
	int initial_angle = -180;
	pointcloud_2_strip_all.clear();   //对存条和条索引的容器进行初始化
	for (int i = 0; i < (360 / strip_angle); i++)  //每一个循环得到一个条状抽条结果
	{
		index_strip.clear();	  //初始化
		pointcloud_strip->clear();  //初始化
		remain->clear();
		for (int t = 0; t < cable_core_pointcloud->size(); t++)
		{
			double x = cable_core_pointcloud->at(t).x;
			double y = cable_core_pointcloud->at(t).y;
			if (x == 0)   //防止分母为0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //对电缆每一个点进行角度计算，以实现按角度抽条
														//cout << "i = " << i << " 时,t = " << t << " 时角度等于" << angle << endl;
			if (((initial_angle + (strip_angle*i)) < angle) && (angle < (initial_angle + (strip_angle*(i + 1)))))
			{
				pointcloud_strip->push_back(cable_core_pointcloud->at(t));
				index_strip.push_back(t);
			}
			else
			{
				remain->push_back(cable_core_pointcloud->at(t));
			}
		}

		//cout << "第" << i << "条的点数= " << pointcloud_strip->size() << endl; //某条抽条结束后输出提示
		if (pointcloud_strip->size())   //将条抽出结果进行存储
		{
			Index_Strip index_strip_strcture;
			index_strip_strcture.index_each_strip.clear();
			index_strip_strcture.index_each_strip = index_strip;
			index_strip_strcture.remain_points = remain->makeShared();
			index_strip_strcture.each_strip = pointcloud_strip->makeShared();
			pointcloud_2_strip_all.push_back(index_strip_strcture);    //将每个条存起来方便返回
		}
		//showpointcloud_xyzrgb(pointcloud_strip);   //当前条抽条显示
		//showpointcloud_xyzrgb(remain);			//电缆抽出当前条后剩余点显示

	}

	return true;
}
/*RANSIC平面拟合  ,最小二乘：照顾所有人的想法 ，ransic：照顾多数人的想法*/
bool Cable::ransic_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delet, Plane_parameter &plane_parameter)
{
	/*ransic平面拟合*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//创建一个模型参数对象，用于记录结果			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers表示误差能容忍的点 记录的是点云的序号			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// 创建一个分割器			
	seg.setOptimizeCoefficients(true);			//Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。			
	seg.setModelType(pcl::SACMODEL_PLANE);		// Mandatory-设置目标几何形状，需要拟合的形状		
	seg.setMethodType(pcl::SAC_RANSAC);			//分割方法：随机采样法			
	seg.setDistanceThreshold(0.2);				//设置误差容忍范围，也就是阈值  同拟合平面的距离超过阈值的点，就被判定为无效数据。
	seg.setInputCloud(cable_core_top_plane_pointcloud_delet);	//输入点云			
	seg.segment(*inliers, *coefficients);		//分割点云，获得平面和法向量 coefficients中存储的是平面模型的系数A/B/C/D，inliers存储拟合出平面的点,可以描述模型的数据

	plane_parameter.plane_parameter_A = coefficients->values[0];
	plane_parameter.plane_parameter_B = coefficients->values[1];
	plane_parameter.plane_parameter_C = coefficients->values[2];  //从拟合平面结果中取出平面参数A/B/C/D
	plane_parameter.plane_parameter_D = coefficients->values[3];

	/*最小二乘平面拟合*/
	//Eigen::Vector4d centroid; //质心
	//Eigen::Matrix3d covariance_matrix; //协方差矩阵
	//pcl::computeMeanAndCovarianceMatrix(*cable_core_top_plane_pointcloud_delet, covariance_matrix, centroid);
	//Eigen::Matrix3d eigenVetors;
	//Eigen::Vector3d eigenValues;
	//pcl::eigen33(covariance_matrix, eigenVetors, eigenValues);
	////查找最小特征值的位置
	//Eigen::Vector3d::Index minRow, minCol;
	//eigenValues.minCoeff(&minRow, &minCol);
	////获取平面方程的系数
	//Eigen::Vector3d normal = eigenVetors.col(minCol);
	//double D = -normal.dot(centroid.head<3>());

	//plane_parameter.plane_parameter_A = normal[0];
	//plane_parameter.plane_parameter_B = normal[1];
	//plane_parameter.plane_parameter_C = normal[2];  //从拟合平面结果中取出平面参数A/B/C/D
	//plane_parameter.plane_parameter_D = D;

	return true;
}
/*RANSIC,进行线拟合*/
bool Cable::ransic_line_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud, Core_circle_center_line &core_circle_center_line)
{
	//平面拟合
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//创建一个模型参数对象，用于记录结果			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers表示误差能容忍的点 记录的是点云的序号			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// 创建一个分割器			
	seg.setOptimizeCoefficients(true);			//Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。			
	seg.setModelType(pcl::SACMODEL_LINE);		// Mandatory-设置目标几何形状，需要拟合的形状		
	seg.setMethodType(pcl::SAC_RANSAC);			//分割方法：随机采样法			
	seg.setDistanceThreshold(0.2);				//设置误差容忍范围，也就是阈值  同拟合平面的距离超过阈值的点，就被判定为无效数据。
	seg.setInputCloud(cable_core_circle_center_pointcloud);	//输入点云			
	seg.segment(*inliers, *coefficients);		//分割点云，获得平面和法向量 coefficients中存储的是平面模型的系数A/B/C/D，inliers存储拟合出平面的点,可以描述模型的数据

												/*//coefficient[0] 直线上一点的 x 坐标
												coefficient[1] 直线上一点的 y 坐标
												coefficient[2] 直线上一点的 z 坐标
												coefficient[3] 直线方向向量的 x 分量
												coefficient[4] 直线方向向量的 y 分量
												coefficient[5] 直线方向向量的 z 分量
												*/
	core_circle_center_line.core_circle_center_line_x = coefficients->values[3];  //拟合直线的方向向量
	core_circle_center_line.core_circle_center_line_y = coefficients->values[4];
	core_circle_center_line.core_circle_center_line_z = coefficients->values[5];

	return true;
}
/*RANSIC进行圆拟合*/
bool Cable::ransic_circle_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_main_insulation_circle_part_pointcloud, Circle &circle)
{

	pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>(cable_main_insulation_circle_part_pointcloud));	//选择拟合点云与几何模型
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3D);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(0.3);	//设置距离阈值，与模型距离小于0.3的点作为内点
	ransac.setMaxIterations(10000);		//设置最大迭代次数
	ransac.computeModel();				//执行模型估计
	std::vector<int> inliers;			//存储内点索引的向量
	ransac.getInliers(inliers);			//提取内点对应的索引

										// 根据索引提取内点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud<pcl::PointXYZRGB>(*cable_main_insulation_circle_part_pointcloud, inliers, *cloud_circle);
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	circle.m_center_x = coefficient[0];  //为了返回给调用函数处的圆心坐标与半径，引用传参
	circle.m_center_y = coefficient[1];
	circle.m_center_z = coefficient[2];
	circle.m_radius = coefficient[3];

	return true;
}
/*椭圆拟合函数*/
bool RGauss(const std::vector<std::vector<double> > & A, std::vector<double> & x)
{
	x.clear();
	int n = A.size();
	int m = A[0].size();
	x.resize(n);
	//复制系数矩阵，防止修改原矩阵;
	std::vector<std::vector<double> > Atemp(n);
	for (int i = 0; i < n; i++)
	{
		std::vector<double> temp(m);
		for (int j = 0; j < m; j++)
		{
			temp[j] = A[i][j];
		}
		Atemp[i] = temp;
		temp.clear();
	}
	for (int k = 0; k < n; k++)
	{
		//选主元;
		double max = -1;
		int l = -1;
		for (int i = k; i < n; i++)
		{
			if (abs(Atemp[i][k]) > max)
			{
				max = abs(Atemp[i][k]);
				l = i;
			}
		}
		if (l != k)
		{
			//交换系数矩阵的l行和k行;
			for (int i = 0; i < m; i++)
			{
				double temp = Atemp[l][i];
				Atemp[l][i] = Atemp[k][i];
				Atemp[k][i] = temp;
			}
		}
		//消元;
		for (int i = k + 1; i < n; i++)
		{
			double l = Atemp[i][k] / Atemp[k][k];
			for (int j = k; j < m; j++)
			{
				Atemp[i][j] = Atemp[i][j] - l*Atemp[k][j];
			}
		}
	}
	//回代;
	x[n - 1] = Atemp[n - 1][m - 1] / Atemp[n - 1][m - 2];
	for (int k = n - 2; k >= 0; k--)
	{
		double s = 0.0;
		for (int j = k + 1; j < n; j++)
		{
			s += Atemp[k][j] * x[j];
		}
		x[k] = (Atemp[k][m - 1] - s) / Atemp[k][k];
	}
	return true;
}
std::vector<double> getEllipseparGauss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in)
{
	std::vector<double> vec_result;
	double x3y1 = 0, x1y3 = 0, x2y2 = 0, yyy4 = 0, xxx3 = 0, xxx2 = 0, x2y1 = 0, yyy3 = 0, x1y2 = 0, yyy2 = 0, x1y1 = 0, xxx1 = 0, yyy1 = 0;
	int N = pointcloud_in->size();
	for (int m_i = 0; m_i < N; ++m_i)
	{
		double xi = pointcloud_in->at(m_i).x;
		double yi = pointcloud_in->at(m_i).y;
		x3y1 += xi*xi*xi*yi;
		x1y3 += xi*yi*yi*yi;
		x2y2 += xi*xi*yi*yi; ;
		yyy4 += yi*yi*yi*yi;
		xxx3 += xi*xi*xi;
		xxx2 += xi*xi;
		x2y1 += xi*xi*yi;

		x1y2 += xi*yi*yi;
		yyy2 += yi*yi;
		x1y1 += xi*yi;
		xxx1 += xi;
		yyy1 += yi;
		yyy3 += yi*yi*yi;
	}
	double resul[5];
	resul[0] = -(x3y1);
	resul[1] = -(x2y2);
	resul[2] = -(xxx3);
	resul[3] = -(x2y1);
	resul[4] = -(xxx2);
	long double Bb[5], Cc[5], Dd[5], Ee[5], Aa[5];
	Bb[0] = x1y3, Cc[0] = x2y1, Dd[0] = x1y2, Ee[0] = x1y1, Aa[0] = x2y2;
	Bb[1] = yyy4, Cc[1] = x1y2, Dd[1] = yyy3, Ee[1] = yyy2, Aa[1] = x1y3;
	Bb[2] = x1y2, Cc[2] = xxx2, Dd[2] = x1y1, Ee[2] = xxx1, Aa[2] = x2y1;
	Bb[3] = yyy3, Cc[3] = x1y1, Dd[3] = yyy2, Ee[3] = yyy1, Aa[3] = x1y2;
	Bb[4] = yyy2, Cc[4] = xxx1, Dd[4] = yyy1, Ee[4] = N, Aa[4] = x1y1;

	std::vector<std::vector<double>>Ma(5);
	std::vector<double>Md(5);
	for (int i = 0; i < 5; i++)
	{
		Ma[i].push_back(Aa[i]);
		Ma[i].push_back(Bb[i]);
		Ma[i].push_back(Cc[i]);
		Ma[i].push_back(Dd[i]);
		Ma[i].push_back(Ee[i]);
		Ma[i].push_back(resul[i]);
	}

	RGauss(Ma, Md);
	long double A = Md[0];
	long double B = Md[1];
	long double C = Md[2];
	long double D = Md[3];
	long double E = Md[4];
	double XC = (2 * B*C - A*D) / (A*A - 4 * B);
	double YC = (2 * D - A*C) / (A*A - 4 * B);
	long double fenzi = 2 * (A*C*D - B*C*C - D*D + 4 * E*B - A*A*E);
	long double fenmu = (A*A - 4 * B)*(B - sqrt(A*A + (1 - B)*(1 - B)) + 1);
	long double fenmu2 = (A*A - 4 * B)*(B + sqrt(A*A + (1 - B)*(1 - B)) + 1);
	double XA = sqrt(fabs(fenzi / fenmu));
	double XB = sqrt(fabs(fenzi / fenmu2));
	double Xtheta = 0.5*atan(A / (1 - B)) * 180 / 3.1415926;
	if (B < 1)
		Xtheta += 90;
	vec_result.push_back(XC);
	vec_result.push_back(YC);
	vec_result.push_back(XA);
	vec_result.push_back(XB);
	vec_result.push_back(Xtheta);
	return vec_result;
}
/*XYZRGB转XYZ*/
bool xyzrgb_2_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_in, pcl::PointCloud<pcl::PointXYZ>::Ptr point_out)
{
	pcl::PointXYZ pointtemp;
	for (int i = 0; i < point_in->size(); i++)
	{
		pointtemp.x = point_in->at(i).x;
		pointtemp.y = point_in->at(i).y;
		pointtemp.z = point_in->at(i).z;
		point_out->push_back(pointtemp);
	}
	return true;
}
/*全局抽条并保存*/
bool Cable::regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)
{

	if (pointcloud_filtered->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	int strip_angle = 5;
	std::vector<int> index_strip;  //存索引
	std::vector<double> angle_number;  //存角度
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的一部分点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的剩余部分点
	int initial_angle = -180;
	m_index_strip.clear();   //对存条和条索引的容器进行初始化
	for (int i = 0; i < (360 / strip_angle); i++)  //每一个循环得到一个条状抽条结果
	{
		index_strip.clear();	  //初始化
		pointcloud_strip->clear();  //初始化
		remain->clear();
		for (int t = 0; t < pointcloud_filtered->size(); t++)
		{
			double x = pointcloud_filtered->at(t).x;
			double y = pointcloud_filtered->at(t).y;
			if (x == 0)   //防止分母为0,只有噪点的x才可能为0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //对电缆每一个点进行角度计算，以实现按角度抽条
														//cout << "i = " << i << " 时,t = " << t << " 时角度等于" << angle << endl;
			if (((initial_angle + (strip_angle * i)) < angle) && (angle < (initial_angle + (strip_angle*(i + 1)))))
			{
				pointcloud_strip->push_back(pointcloud_filtered->at(t));
				index_strip.push_back(t);
			}
			else
			{
				remain->push_back(pointcloud_filtered->at(t));
			}
		}

		//cout << "第" << i << "条的点数= " << pointcloud_strip->size() << endl; //某条抽条结束后输出提示
		if (pointcloud_strip->size())   //将条抽出结果进行存储
		{
			int b, g, r; b = g = r = 0;
			random_colour(b, g, r);
			for (int j = 0; j < pointcloud_strip->size(); j++)  //给每一条分别上色
			{
				pointcloud_strip->at(j).b = b;
				pointcloud_strip->at(j).g = g;
				pointcloud_strip->at(j).r = r;
			}

			Index_Strip index_strip_strcture;
			index_strip_strcture.index_each_strip.clear();
			index_strip_strcture.index_each_strip = index_strip;
			index_strip_strcture.remain_points = remain->makeShared();
			index_strip_strcture.each_strip = pointcloud_strip->makeShared();
			m_index_strip.push_back(index_strip_strcture);    //将每个条状存至全局变量

		}
	}
	return true;
}
/*抽条函数重载，保存的结果不再是全局变量*/
bool Cable::regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<Index_Strip> &strip_vector, float strip_angle)
{

	if (pointcloud_filtered->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	//int strip_angle = 1;
	std::vector<int> index_strip;  //存索引
	std::vector<double> angle_number;  //存角度
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的一部分点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//存一条的剩余部分点
	int initial_angle = -180;
	strip_vector.clear();   //对存条和条索引的容器进行初始化
	for (int i = 0; i < (360 / strip_angle); i++)  //每一个循环得到一个条状抽条结果
	{
		index_strip.clear();	  //初始化
		pointcloud_strip->clear();  //初始化
		remain->clear();
		for (int t = 0; t < pointcloud_filtered->size(); t++)
		{
			double x = pointcloud_filtered->at(t).x;
			double y = pointcloud_filtered->at(t).y;
			if (x == 0)   //防止分母为0,只有噪点的x才可能为0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //对电缆每一个点进行角度计算，以实现按角度抽条
														//cout << "i = " << i << " 时,t = " << t << " 时角度等于" << angle << endl;
			if (((initial_angle + (strip_angle * i)) < angle) && (angle < (initial_angle + (strip_angle*(i + 1)))))
			{
				pointcloud_strip->push_back(pointcloud_filtered->at(t));
				index_strip.push_back(t);
			}
			else
			{
				remain->push_back(pointcloud_filtered->at(t));
			}
		}

		//cout << "第" << i << "条的点数= " << pointcloud_strip->size() << endl; //某条抽条结束后输出提示
		//给不同条上不同色,有这一步就能给不同颜色赋以不同颜色
		if (pointcloud_strip->size())   //将条抽出结果进行存储
		{
			/*上色*/
			//int b, g, r; b = g = r = 0;
			//random_colour(b, g, r);
			//for (int j = 0; j < pointcloud_strip->size(); j++)  //给每一条分别上色
			//{
			//	pointcloud_strip->at(j).b = b;
			//	pointcloud_strip->at(j).g = g;
			//	pointcloud_strip->at(j).r = r;
			//}
			/*上色*/

			Index_Strip index_strip_strcture;
			index_strip_strcture.index_each_strip.clear();
			index_strip_strcture.index_each_strip = index_strip;
			index_strip_strcture.remain_points = remain->makeShared();
			index_strip_strcture.each_strip = pointcloud_strip->makeShared();
			strip_vector.push_back(index_strip_strcture);    //将每个条状存至全局变量

		}
	}
	return true;
}
/*圆柱拟合*/
bool Cable::ransac_cylindrical_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cylindrical_pointcloud, Cylinder &cylinder)
{
	//-----------------------------目标点法线估计--------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;	// 创建法向量估计对象
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	n.setSearchMethod(tree);						       // 设置搜索方式
	n.setInputCloud(Cylindrical_pointcloud);						           // 设置输入点云
	n.setKSearch(20);								       // 设置K近邻搜索点的个数
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);						          // 计算法向量，并将结果保存到normals中
														  //----------------------------圆柱拟合--------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;// 创建圆柱体分割对象
	seg.setInputCloud(Cylindrical_pointcloud);										// 设置输入点云
	seg.setInputNormals(normals);								    // 设置输入法向量
	seg.setOptimizeCoefficients(true);								// 设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);						// 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);								// 设置采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.3);								// 设置表面法线权重系数
	seg.setMaxIterations(10000);									// 设置迭代的最大次数
	seg.setDistanceThreshold(0.1);									// 设置内点到模型距离的最大值
	seg.setRadiusLimits(20, 40);									// 设置圆柱模型半径的范围

	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);	// 保存分割结果
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);	// 保存圆柱体模型系数
	seg.segment(*inliers_cylinder, *coefficients_cylinder);			// 执行分割，将分割结果的索引保存到inliers_cylinder中，同时存储模型系数coefficients_cylinder

																	//cout << "轴线一点坐标：(" << coefficients_cylinder->values[0] << ", "
																	//	<< coefficients_cylinder->values[1] << ", "
																	//	<< coefficients_cylinder->values[2] << ")"
																	//	<< endl;
																	//cout << "轴线方向向量：(" << coefficients_cylinder->values[3] << ", "
																	//	<< coefficients_cylinder->values[4] << ", "
																	//	<< coefficients_cylinder->values[5] << ")"
																	//	<< endl;
																	//cout << "圆柱体半径：" << coefficients_cylinder->values[6] << endl;	

	cylinder.point_on_axis_x = coefficients_cylinder->values[0];
	cylinder.point_on_axis_y = coefficients_cylinder->values[1];
	cylinder.point_on_axis_z = coefficients_cylinder->values[2];
	cylinder.axis_direction_x = coefficients_cylinder->values[3];
	cylinder.axis_direction_y = coefficients_cylinder->values[4];
	cylinder.axis_direction_z = coefficients_cylinder->values[5];
	cylinder.radius = coefficients_cylinder->values[6];

	return 0;
}
/*条切片*/
bool Cable::Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)   	//对每个竖条进行z向均分,将每一条分割成一小片一小片
{
	pcl::PointXYZRGB point;
	pcl::PointXYZRGB point2;
	std::vector<int> once_strip_slice_index;
	std::vector<Strip_slice> once_strip_get_slice;			 //存条上片和条上片索引的容器
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice(new pcl::PointCloud<pcl::PointXYZRGB>);
	double cable_z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;
	double cable_z_mim = pointcloud_filtered->at(0).z;
	double slice_point_heoght = 1;
	double increase_rate = slice_point_heoght;  //小片点云高度控制
												//double increase_rate = (cable_z_max - cable_z_mim) / ( 2*(cable_z_max - cable_z_mim)  );  //保证每片是2mm

	m_cable_strip_slice_all.clear();

	//std::cout << "！！！！！！！！！！！！！！！！！！！！！！！！！！条切片开始！！！！！！！！！！！！！！！！！！！！！！" << endl;
	for (int strip = 0; strip < m_index_strip.size(); strip++)	//对电缆的每个条进行处理
	{
		//cout << "第 " << strip << "条电缆的长度为" << m_index_strip.at(strip).each_strip->at(m_index_strip.at(strip).each_strip->size() - 1).z - m_index_strip.at(strip).each_strip->at(0).z << endl;
		//std::cout << "第 " << strip << " 条切片开始！" << endl;
		once_strip_get_slice.clear();  //存储一条上的片与片的索引
		point = m_index_strip.at(strip).each_strip->at(0);
		for (double k = cable_z_mim + (slice_point_heoght / 2); k < cable_z_max; k += increase_rate)   //对每个小条按找等z间距进行切片处理
		{
			point2 = point;
			point.z = k;
			point.x = point2.x;
			point.y = point2.y;
			pointcloud_strip_slice->clear();   //存储一个片
			once_strip_slice_index.clear();	   //存储一个片的索引
			for (int t = 0; t < m_index_strip.at(strip).each_strip->size(); t++)		//找出一定z值范围内的片
			{
				double theta = m_index_strip.at(strip).each_strip->at(t).z - point2.z;
				if (-(slice_point_heoght / 2) < theta && theta < (slice_point_heoght / 2))
				{
					pointcloud_strip_slice->push_back(m_index_strip.at(strip).each_strip->at(t));
					once_strip_slice_index.push_back(t);
				}
			}
			//showpointcloud_xyzrgb(pointcloud_strip_slice);


			if (pointcloud_strip_slice->size())   //将找出来的小片和小片索引进行存储
			{
				Strip_slice strip_slice;
				strip_slice.index_strip_slice = once_strip_slice_index;  //把找出来的小片的索引传给结构体
				strip_slice.pointcloud_strip_slice = pointcloud_strip_slice->makeShared();
				once_strip_get_slice.push_back(strip_slice);     //将一个小片和小片的索引存起来
			}

			//给不同片上不同色
			int b, g, r;
			for (int m = 0; m < once_strip_get_slice.size(); m++)
			{
				random_colour(b, g, r);	//random(0, 10)产生一个0-10的随机数，这里是为了给不同的片提供不同的b、g、r，从而实现不同颜色的显示
				for (int n = 0; n < m_index_strip.at(strip).each_strip->size(); n++)
				{
					if (m_index_strip.at(strip).each_strip->at(n).z >= once_strip_get_slice.at(m).pointcloud_strip_slice->at(0).z &&
						m_index_strip.at(strip).each_strip->at(n).z <= once_strip_get_slice.at(m).pointcloud_strip_slice->at(once_strip_get_slice.at(m).pointcloud_strip_slice->size() - 1).z)
					{
						m_index_strip.at(strip).each_strip->at(n).b = b;
						m_index_strip.at(strip).each_strip->at(n).g = g;
						m_index_strip.at(strip).each_strip->at(n).r = r;
					}
				}
			}

		}
		//showpointcloud_xyzrgb(m_index_strip.at(strip).each_strip);
		m_cable_strip_slice_all.push_back(once_strip_get_slice);    //将存储每条小片的容器存储起来，cable_strip_slice_all的成员是每条电缆的小片及索引
																	//std::cout << "第 " << strip << " 条切片数量= " << once_strip_get_slice.size() << endl;
	}
	return true;
}
/*切片函数重载，保存的不再是全局的切片变量*/
bool Cable::Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<std::vector<Strip_slice>> &slice_vector,
	std::vector<Index_Strip> XLPE_strip_vector, double slice_point_heoght)   	//对每个竖条进行z向均分,将每一条分割成一小片一小片
{
	pcl::PointXYZRGB point;
	pcl::PointXYZRGB point2;
	std::vector<int> once_strip_slice_index;
	std::vector<Strip_slice> once_strip_get_slice;			 //存条上片和条上片索引的容器
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice(new pcl::PointCloud<pcl::PointXYZRGB>);
	double cable_z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;
	double cable_z_mim = pointcloud_filtered->at(0).z;
	//double slice_point_heoght = 0.5;
	double increase_rate = slice_point_heoght;  //小片点云高度控制
												//double increase_rate = (cable_z_max - cable_z_mim) / ( 2*(cable_z_max - cable_z_mim)  );  //保证每片是2mm

	slice_vector.clear();

	//std::cout << "！！！！！！！！！！！！！！！！！！！！！！！！！！条切片开始！！！！！！！！！！！！！！！！！！！！！！" << endl;
	for (int strip = 0; strip < XLPE_strip_vector.size(); strip++)	//对电缆的每个条进行处理
	{
		//cout << "第 " << strip << "条电缆的长度为" << XLPE_strip_vector.at(strip).each_strip->at(XLPE_strip_vector.at(strip).each_strip->size() - 1).z - XLPE_strip_vector.at(strip).each_strip->at(0).z << endl;
		//std::cout << "第 " << strip << " 条切片开始！" << endl;
		once_strip_get_slice.clear();  //存储一条上的片与片的索引
		point = XLPE_strip_vector.at(strip).each_strip->at(0);
		for (double k = cable_z_mim + (slice_point_heoght / 2); k < cable_z_max; k += increase_rate)   //对每个小条按找等z间距进行切片处理
		{
			point2 = point;
			point.z = k;
			point.x = point2.x;
			point.y = point2.y;
			pointcloud_strip_slice->clear();   //存储一个片
			once_strip_slice_index.clear();	   //存储一个片的索引
			for (int t = 0; t < XLPE_strip_vector.at(strip).each_strip->size(); t++)		//找出一定z值范围内的片
			{
				double theta = XLPE_strip_vector.at(strip).each_strip->at(t).z - point2.z;
				if (-(slice_point_heoght / 2) < theta && theta < (slice_point_heoght / 2))
				{
					pointcloud_strip_slice->push_back(XLPE_strip_vector.at(strip).each_strip->at(t));
					once_strip_slice_index.push_back(t);
				}
			}
			//showpointcloud_xyzrgb(pointcloud_strip_slice);


			if (pointcloud_strip_slice->size())   //将找出来的小片和小片索引进行存储
			{
				Strip_slice strip_slice;
				strip_slice.index_strip_slice = once_strip_slice_index;  //把找出来的小片的索引传给结构体
				strip_slice.pointcloud_strip_slice = pointcloud_strip_slice->makeShared();
				once_strip_get_slice.push_back(strip_slice);     //将一个小片和小片的索引存起来
			}

			//给不同片上不同色,有这一步就能给不同颜色赋以不同颜色
			//int b, g, r;
			//for (int m = 0; m < once_strip_get_slice.size(); m++)
			//{
			//	random_colour(b, g, r);	//random(0, 10)产生一个0-10的随机数，这里是为了给不同的片提供不同的b、g、r，从而实现不同颜色的显示
			//	for (int n = 0; n < XLPE_strip_vector.at(strip).each_strip->size(); n++)
			//	{
			//		if (XLPE_strip_vector.at(strip).each_strip->at(n).z >= once_strip_get_slice.at(m).pointcloud_strip_slice->at(0).z &&
			//			XLPE_strip_vector.at(strip).each_strip->at(n).z <= once_strip_get_slice.at(m).pointcloud_strip_slice->at(once_strip_get_slice.at(m).pointcloud_strip_slice->size() - 1).z)
			//		{
			//			XLPE_strip_vector.at(strip).each_strip->at(n).b = b;
			//			XLPE_strip_vector.at(strip).each_strip->at(n).g = g;
			//			XLPE_strip_vector.at(strip).each_strip->at(n).r = r;
			//		}
			//	}
			//}
			/*上色*/

		}
		//showpointcloud_xyzrgb(XLPE_strip_vector.at(strip).each_strip);
		slice_vector.push_back(once_strip_get_slice);    //将存储每条小片的容器存储起来，cable_strip_slice_all的成员是每条电缆的小片及索引
														 //std::cout << "第 " << strip << " 条切片数量= " << once_strip_get_slice.size() << endl;
	}
	return true;
}
/*随机赋值函数*/
bool random_colour(int &b, int &g, int &r)
{
	b = g = r = 0;
	b = random(0, 255);
	g = random(0, 255);
	r = random(0, 255);

	return true;
}
/*HSV转RGB*/
bool Cable::HSVtoRGB(std::vector<int> hsv, std::vector<int>& rgb)
{
	int h = hsv[0], s = hsv[1], v = hsv[2];
	float RGB_min, RGB_max;
	RGB_max = v*2.55f * 100;
	RGB_min = RGB_max*(100 - s * 100) / 100.0f;
	int i = h / 60;
	int difs = h % 60; // factorial part of h
	float RGB_Adj = (RGB_max - RGB_min)*difs / 60.0f;

	switch (i)
	{
	case 0:
		rgb.push_back(RGB_max);
		rgb.push_back(RGB_min + RGB_Adj);
		rgb.push_back(RGB_min);
		break;
	case 1:
		rgb.push_back(RGB_max - RGB_Adj);
		rgb.push_back(RGB_max);
		rgb.push_back(RGB_min);
		break;
	case 2:
		rgb.push_back(RGB_min);
		rgb.push_back(RGB_max);
		rgb.push_back(RGB_min + RGB_Adj);
		break;
	case 3:
		rgb.push_back(RGB_min);
		rgb.push_back(RGB_max - RGB_Adj);
		rgb.push_back(RGB_max);
		break;
	case 4:
		rgb.push_back(RGB_min + RGB_Adj);
		rgb.push_back(RGB_min);
		rgb.push_back(RGB_max);
		break;
	default:		// case 5:
		rgb.push_back(RGB_max);
		rgb.push_back(RGB_min);
		rgb.push_back(RGB_max - RGB_Adj);
		break;
	}
	return true;
}