#include "Cable.h"

Cable::Cable()
{
}

Cable::~Cable()
{
}

/*���캯��*/
Cable::Cable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb)  //���캯������
{
	m_pointcloud.clear();
	m_pointcloud = *pointcloud_rgb;
	m_pointcloud_rgb = m_pointcloud.makeShared();   //��ԭʼָ��������
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
/*ȱ�ݼ��*/
bool Cable::concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, float each_strip_angle, double slice_height, double conductEnd_2_outEnd_distance,
	std::vector<defection_quantification> &Defection_quantification, std::vector<defection_quantification> &Defection_quantification_concave)
{
	/*����ƽ�淽��������ɢ��*/
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
	/*��ȡ����ƽ�淽��������ɢ��ĵ���*/
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_plane_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pointcloud_plane_xyz->clear();
	//loadPointcloud("plane.ply", pointcloud_plane_xyz);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	//xyz2rgb(pointcloud_plane_xyz, pointcloud_plane);			 //xyzתxyzrgb

	/*��Ϊ��ƽ����ɢ�������ȱ��*/
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
	/*��֤����ƽ����Ϸ�����Ч��*/
	//float RA, RB, RC, RD; //�������һ�������
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Inner_point (new pcl::PointCloud<pcl::PointXYZRGB>);
	//ransac_plane_estimation(pointcloud_plane_add_noise, RA, RB, RC, RD, Inner_point);  //RANSACƽ�����
	//float DA, DB, DC, DD; //��С�������
	//least_square_plane_estimation(pointcloud_plane_add_noise, DA, DB, DC, DD);
	//float ADA, ADB, ADC, ADD; //�������һ���Խ����С����
	//least_square_plane_estimation(Inner_point, ADA, ADB, ADC, ADD);
	cout << endl;
	cout << "���ڽ�������Ե���氼��͹��ȱ�ݼ������������!" << endl;
	//����µ���ߵ㣬ֻ��һ���㲻׼ȷ�����������m���㲢��ƽ�����������һ�����Բ��Բ��ԭ������Ϊ����ɨ�����
	int cable_max_point_number = 20;
	double sum_cable_max_point = 0;
	for (int i = 1; i <= cable_max_point_number; i++)
	{
		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
	}
	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //��о����ߵ㣬����о�յ�

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cylinder_pointcloud = pointcloud_filtered->makeShared();
	//�����е�תΪ��ɫ
	for (int i = 0; i<cylinder_pointcloud->size(); i++)
	{
		cylinder_pointcloud->at(i).r = 255;
		cylinder_pointcloud->at(i).g = 255;
		cylinder_pointcloud->at(i).b = 255;
	}
	//showpointcloud_xyzrgb(cylinder_pointcloud);

	/*�������*/
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
	/*Բ����ϣ����ݸ���������ߵľ����Բ���뾶��ֵ���ҳ���͹��*/
	Cylinder cylinder;  //Բ�����
	ransac_cylindrical_estimation(cylinder_pointcloud, cylinder);
	//for (int i = 0; i < cylinder_pointcloud->size(); i++)
	//{
	//	Eigen::Vector4f point;
	//	point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //Բ��������һ��
	//	Eigen::Vector4f normal;
	//	normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //Բ�����߷�������
	//	Eigen::Vector4f point_cloud_point;
	//	point_cloud_point << cylinder_pointcloud->at(i).x, cylinder_pointcloud->at(i).y, cylinder_pointcloud->at(i).z;
	//	double point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// �ռ�㵽�ռ�ֱ�߾��룬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
	//	if (abs(cylinder.radius - point_2_line_distance) > 0.45 )   //���������һ��������߾�����Բ�����߰뾶�����һ��ֵ
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
	//float each_strip_angle = 2;  //�����Ƕ�
	regionSegmentation(cylinder_pointcloud, XLPE_strip_vector, each_strip_angle); //�ֲ����Ƴ���
																				  //showpointcloud_xyzrgb(XLPE_strip_vector.at(0).each_strip);  //��ʾһ����״����
																				  /*���һ������*/
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
																				  /*չʾ�����㷨����Ч��*/
	for (int i = 0; i<XLPE_strip_vector.size(); i++)
	{
		for (int j = 0; j<XLPE_strip_vector.at(i).each_strip->size(); j++)
		{
			point_cloud_colour->push_back(XLPE_strip_vector.at(i).each_strip->at(j));
		}
	}
	//showpointcloud_xyzrgb(point_cloud_colour);//չʾ�����㷨����Ч��
	std::vector<std::vector<Strip_slice>> slice_vector;  //�洢�ֲ�Ƭ״���Ƶ�����

	Strip_2_slice(cylinder_pointcloud, slice_vector, XLPE_strip_vector, slice_height);  //�ֲ�������Ƭ
	std::vector<std::vector<Strip_slice>> slice_vector1;
	Strip_2_slice(cylinder_pointcloud, slice_vector1, XLPE_strip_vector, slice_height);  //����һ�ε�Ŀ����Ϊ�˹���������Ƭ������������ɫӳ��Ų����׳���
																						 //slice_vector��slice_vector1/slice_vector2����һģһ����ֻ�ǵ�ַ��һ��
																						 //showpointcloud_xyzrgb(slice_vector.at(0).at(0).pointcloud_strip_slice);
																						 /*չʾ��Ƭ�㷨����Ч��*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_local_slice_colour(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < XLPE_strip_vector.size(); i++)
	{
		for (int j = 0; j < XLPE_strip_vector.at(i).each_strip->size(); j++)
		{
			pointcloud_local_slice_colour->push_back(XLPE_strip_vector.at(i).each_strip->at(j));
		}
	}
	//showpointcloud_xyzrgb(pointcloud_local_slice_colour);  //չʾ��Ƭ�㷨����Ч��

	/*ÿ�����ƽ�棬Ȼ����ݵ㵽ƽ����빹����άͼ*/
	std::vector<Mapping_2D_3D> mapping_2D_3D; mapping_2D_3D.clear();
	std::vector<float> slice_distance; slice_distance.clear();
	cv::Mat pointcloud_img = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_32FC1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*Ϊ����ɫ��ʾ*/
	std::vector<Color_mapping> convex_points; convex_points.clear();
	std::vector<Color_mapping> concave_points; concave_points.clear();
	std::vector<Color_mapping> regular_points; regular_points.clear();
	std::vector<double>convex_distance; convex_distance.clear();
	std::vector<double>concave_distance; concave_distance.clear();
	double max_convex = 0; double max_concave = 0;
	/*Ϊ����ɫ��ʾ*/
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr Plan_mapping(new pcl::PointCloud<pcl::PointXYZHSV>);   //����������ӳ��ɶ�άƽ�����
	for (int i = 0; i < slice_vector.size(); i++) //��������,��ӳ���ȥ��y��Ƭӳ���ȥ��x
	{
		plane_pointcloud->clear();
		//�����ƽ��ĵ��ҳ������õ������������е�
		for (int j = 0; j<slice_vector.at(i).size(); j++) //ÿ����Ƭ������
		{
			for (int k = 0; k<slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//ÿƬ�ϵ������
			{
				plane_pointcloud->push_back(slice_vector.at(i).at(j).pointcloud_strip_slice->at(k));
			}
		}
		//for (int j = 0; j < slice_vector.at(i).size(); j++) //ÿ����Ƭ������
		//{
		//	for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//ÿƬ�ϵ������
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
		ransac_plane_estimation(plane_pointcloud, A, B, C, D, interior_point);  //RANSACƽ�����
																				//float A1, B1, C1, D1;
																				//least_square_plane_estimation(interior_point, A1, B1, C1, D1);  //��С����ƽ�����
		least_square_plane_estimation(interior_point, A, B, C, D);  //��С����ƽ�����

																	/*��һ����ɫ��ʽ,ͨ���ж�ÿ�������ƽ��ľ��������������ɫ���������ɶ�ά�ٷ�ת��ά*/
		for (int j = 0; j < slice_vector.at(i).size(); j++)  //һ����Ƭ������
		{
			for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//һƬ�ϵ������
			{
				pcl::PointXYZRGB once_point = slice_vector.at(i).at(j).pointcloud_strip_slice->at(k);
				float point_22_plane_distance = abs(pcl::pointToPlaneDistanceSigned(once_point, A, B, C, D));
				Eigen::Vector4f point1;
				point1 << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //Բ��������һ��
				Eigen::Vector4f normal1;
				normal1 << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //Բ�����߷�������
				Eigen::Vector4f point_cloud_point1;
				point_cloud_point1 << slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z; //СƬ��������
				float point_22_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point1, point1, normal1));// �ռ�㵽�ռ�ֱ�߾�����㣬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ���
				if (point_22_line_distance > cylinder.radius)   //�㵽���߾������Բ���뾶��͹��
				{
					Color_mapping color_mapping;
					color_mapping.point_2_plane_distance = point_22_plane_distance;
					color_mapping.point_index_of_slice = k;
					color_mapping.slice_index_of_strip = j;
					color_mapping.strip_index = i;
					convex_points.push_back(color_mapping);
					convex_distance.push_back(point_22_plane_distance);
				}
				else if (point_22_line_distance < cylinder.radius)	//�㵽���߾���С��Բ���뾶������
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

		/*��һ����ɫ��ʽ,ͨ���ж�ÿ�������ƽ��ľ��������������ɫ���������ɶ�ά�ٷ�ת��ά*/

		/*����㵽ƽ��ľ��룬Ȼ��������ı�ţ�Ƭ�ı�ţ��Լ��㵽ƽ��ľ������ӳ�䣬���ı����Ϊͼ���yֵ��Ƭ�ı����Ϊxֵ���㵽ƽ�������Ϊ����ֵ*/
		for (int j = 0; j < slice_vector.at(i).size(); j++) //ÿ����Ƭ������
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			slice_pointcloud->clear();
			float slice_point_2_plane_distance_count = 0;
			for (int k = 0; k < slice_vector.at(i).at(j).pointcloud_strip_slice->size(); k++)//ÿƬ�ϵ������
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
			float slice_2_plane_distance = slice_point_2_plane_distance_count / slice_vector.at(i).at(j).pointcloud_strip_slice->size();  //СƬ���ƾ������ߵľ���֮�ͳ��Ե���

																																		  /*�жϿ�Ԫ�İ�͹��ϵ*/
			Eigen::Vector4f centroid;					// �������ļ���
			pcl::compute3DCentroid(*slice_pointcloud, centroid);	// ������꣬��c0,c1,c2,1��,СƬ�������ģ���СƬ����������ƽ�淨�����н�С��90�����ڷ���������һ�࣬��������һ��
			Eigen::Vector4f point;
			point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //Բ��������һ��
			Eigen::Vector4f normal;
			normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //Բ�����߷�������
			Eigen::Vector4f point_cloud_point;
			point_cloud_point << centroid(0), centroid(1), centroid(2); //СƬ��������
			float point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// �ռ�㵽�ռ�ֱ�߾�����㣬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
			Eigen::Vector4f normal_of_point;  //��״�������ƽ����һ��,���ƽ����һ����Ԫ���ĵ�����(0,0,-(D/C)����״�������ƽ����һ��)
			normal_of_point << (centroid(0) - 0), (centroid(1) - 0), (centroid(2) - (-(D / C)));
			float direction = A*normal_of_point(0) + B*normal_of_point(1) + C*normal_of_point(2);   //�жϿ�Ԫ�����뷨���������һ���ԣ������ʽ����0�������ÿ�Ԫ�ڷ����������ͷ��һ�࣬������β����һ��
																									/*�ж�ƽ�淨�����ĳ��򣬼�ָ�����߻��Ǳ�������*/
			Eigen::Vector4f centroid1;					// ��״�������ļ���
			pcl::compute3DCentroid(*plane_pointcloud, centroid1);
			float plane_x = A; float plane_y = B;
			float strip_x = centroid1(0);	float strip_y = centroid1(1);
			float cos_seta = (plane_x*strip_x + plane_y*strip_y) / (sqrt(pow(plane_x, 2) + pow(plane_y, 2)) * sqrt(pow(strip_x, 2) + pow(strip_y, 2)));
			/*ͨ��ƽ�������ƽ����һ���γɵķ���������ƽ�������ļнǣ��Լ���״��������������ƽ�������ļн��жϵ�λ��Բ���ڲ໹�����*/
			if (((direction > 0) && (cos_seta < 0)) || ((direction < 0) && (cos_seta > 0)))
			{
				slice_2_plane_distance = -slice_2_plane_distance;
			}
			/*ͨ���Ƚ�С��������ľ����ߵľ�����Բ���뾶�ľ��룬�жϵ�λ��Բ���ڲ໹�����*/
			//if (point_2_line_distance < cylinder.radius)  //���ݿ�Ԫ���ľ������ߵľ������ж��ǰ�����͹�����͸���ֵ��͹�͸���ֵ
			//{
			//	slice_2_plane_distance = -slice_2_plane_distance;
			//}
			Mapping_2D_3D mapping;
			mapping.x = i;  //x�����ı��
			mapping.y = j;	//y��Ƭ�ı��
			mapping.pixel_value = slice_2_plane_distance; //��Ԫ���ľ������ƽ��ľ���
			mapping_2D_3D.push_back(mapping);
			slice_distance.push_back(slice_2_plane_distance);
			pointcloud_img.at<float>(i, j) = slice_2_plane_distance;

			/*��ά����ӳ���ƽ�������ʾ*/
			pcl::PointXYZHSV hsv_point_tmp;  //ÿ����xΪ���ı�ţ�yΪ��ı�ţ�zΪ�����ĵ����ƽ��ľ���
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
			/*��ά����ӳ���ƽ�������ʾ*/
		}
	}
	sort(slice_distance.begin(), slice_distance.end());
	cv::Mat orgin_image = pointcloud_img.clone();  //ԭʼӳ��ͼ��
												   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\orgin_image.png", orgin_image);
												   //showpointcloud_xyzhsv(Plan_mapping); //��ά����ӳ���ƽ�������ʾ

												   /*��һ����ɫ��ʽ,ͨ���ж�ÿ�������ƽ��ľ��������������ɫ��������ʾ��ϸ�壬�������ɶ�ά�ٷ�ת��ά*/
	sort(convex_distance.begin(), convex_distance.end());
	max_convex = convex_distance.back();
	//double convex_rate = 120 / max_convex;    //����͹�ĳ̶Ⱦ�����ɫ����ǳ
	double convex_rate = 120;  //ֱ����1mmΪ��ɫ����㣬���ڵ���1mm��͹�㶼�����ģ�����������ȱ��
	sort(concave_distance.begin(), concave_distance.end());
	max_concave = concave_distance.back();
	//double concave_rate = 120 / max_concave;
	double concave_rate = 120;	//ֱ����1mmΪ��ɫ����㣬���ڵ���1mm�İ��㶼�������ģ�����������ȱ��
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr colour_point_cloud_hsv_all(new pcl::PointCloud<pcl::PointXYZHSV>);
	//HSV��ɫ�ռ䣬H=120����ɫ��120����360��������ɫ��120����0��������ɫ
	for (int m = 0; m < convex_points.size(); m++)
	{
		pcl::PointXYZHSV point_temp_HSV_convex;
		int strip_index = convex_points.at(m).strip_index;
		int slice_index = convex_points.at(m).slice_index_of_strip;
		int point_index = convex_points.at(m).point_index_of_slice;
		point_temp_HSV_convex.x = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).x;
		point_temp_HSV_convex.y = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).y;
		point_temp_HSV_convex.z = slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(point_index).z;
		if (convex_points.at(m).point_2_plane_distance > 1)  //͹�𳬹�1mm�ģ�ֱ������Ϊ���ɫ
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
	}   //͹�Ĳ����ó���
	for (int m = 0; m < concave_points.size(); m++)
	{
		pcl::PointXYZHSV point_temp_HSV_concave;
		int strip_index_concave = concave_points.at(m).strip_index;
		int slice_index_concave = concave_points.at(m).slice_index_of_strip;
		int point_index_concave = concave_points.at(m).point_index_of_slice;
		point_temp_HSV_concave.x = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).x;
		point_temp_HSV_concave.y = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).y;
		point_temp_HSV_concave.z = slice_vector.at(strip_index_concave).at(slice_index_concave).pointcloud_strip_slice->at(point_index_concave).z;
		if ((concave_points.at(m).point_2_plane_distance) < -1)    //���ӳ���1mm�ģ�ֱ������Ϊ����ɫ
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

	}   //���Ĳ����ó���
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
	} //�����Ĳ����ó���
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_all);
	__colour_point_cloud_hsv_all__ = colour_point_cloud_hsv_all->makeShared();
	/*��һ����ɫ��ʽ,ͨ���ж�ÿ�������ƽ��ľ��������������ɫ���������ɶ�ά�ٷ�ת��ά*/

	/*��ӳ��ͼ���еĸ�ֵȫ����Ϊ��ֵ*/
	cv::Mat pointcloud_img_plus = cv::Mat::zeros(pointcloud_img.size(), CV_32FC1);
	float pixel_small = slice_distance.front();
	cv::Mat  img_tmp = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_32FC1);
	//float trans_rate = 240 / (slice_distance.back()+ abs(pixel_small));
	for (int i = 0; i < pointcloud_img.rows; i++)
	{
		for (int j = 0; j<pointcloud_img.cols; j++)
		{
			pointcloud_img_plus.at<float>(i, j) = pointcloud_img.at<float>(i, j) + abs(pixel_small);//�ø��Ĳ���ȫ����Ϊ���ģ����ں�������
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\pointcloud_img_plus.png", pointcloud_img_plus);

	/*���طŴ�͹��ȱ��*/
	float minValue = *max_element(pointcloud_img_plus.begin<float>(), pointcloud_img_plus.end<float>());  //���ԭʼͼ��ȫ��ӳ��Ϊ��ֵ������ֵ������������Ŵ���
	int enlarge_rate = 250 / minValue; //���طŴ���
	for (int i = 0; i < pointcloud_img_plus.rows; i++)
	{
		for (int j = 0; j < pointcloud_img_plus.cols; j++)
		{
			img_tmp.at<float>(i, j) = pointcloud_img_plus.at<float>(i, j) * enlarge_rate;    //�Ŵ�ȱ�����������صĲ��
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\enlarge_img_A"+to_string(each_strip_angle)+"_h_"+ to_string(slice_height)+".png", img_tmp);

	/*��float����תΪint����Ϊ�ܶ�opencv�ܶ����ֻ֧��8λ�޷�����*/
	cv::Mat HSV_img_h = cv::Mat::zeros(cv::Size(slice_vector.at(0).size(), slice_vector.size()), CV_8UC1);
	for (int i = 0; i < img_tmp.rows; i++)
	{
		for (int j = 0; j < img_tmp.cols; j++)
		{
			HSV_img_h.at<uchar>(i, j) = (int)(img_tmp.at<float>(i, j));
		}
	}
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\float_2_int.png", HSV_img_h);

	/*ͼ������������ȡ*/
	std::vector<int> image_pixel_value; image_pixel_value.clear();
	for (int i = 0; i<HSV_img_h.rows; i++)
	{
		for (int j = 0; j<HSV_img_h.cols; j++)
		{
			image_pixel_value.push_back(HSV_img_h.at<uchar>(i, j));  //�Ȱ�����ֵ���Ž�����
		}
	}
	sort(image_pixel_value.begin(), image_pixel_value.end()); //����
															  //int mode_pixel_value = image_pixel_value[image_pixel_value.size() / 2];  //���������ʱ������ռ�ȷǳ����������������͵����������м��Ǹ���
	std::vector<int> pixel_value; pixel_value.clear();   //������ÿһ�����ظ�������ֵ
	pixel_value.push_back(image_pixel_value[0]);		//��һ��Ԫ��Ϊ����ֵ�������ֵĵ�һ��ֵ
	std::vector<int> frequency; frequency.clear();//������¼ÿһ������ֵ�����˶��ٴ�
	frequency.push_back(1);	//��һ��Ԫ�ظ���ʼֵ������һ��
	int m = 0;
	for (int i = 1; i<image_pixel_value.size(); i++)
	{
		if (image_pixel_value[i] != image_pixel_value[i - 1])  //�����ǰ����ֵ��ǰһ������ֵ�����
		{
			pixel_value.push_back(image_pixel_value[i]);  //�ѵ�ǰ����ֵ������
			frequency.push_back(1);	//���Ҹ����ֵĴ���Ϊ1
			m++;	//ͬʱ�ü�¼������λ�ú���һλ
		}
		else
		{
			frequency[m]++; //�����ǰ����ֵ��ǰһ������ֵ��ȣ���ǰ����ֵ���ֵĴ�����1
		}
	}
	int maxPosition = max_element(frequency.begin(), frequency.end()) - frequency.begin(); //���ִ�����������ֵ���±�
	int mode_value = pixel_value.at(maxPosition);//���ִ�����������ֵ������

												 /*��Ե���*/
	cv::Mat boundary;
	cv::Canny(HSV_img_h, boundary, 100, 200, 3);

	/*��ֵͼ����ȡ*/
	Mat image_mean;
	Mat image_std;
	cv::meanStdDev(HSV_img_h, image_mean, image_std);
	int image_mean_value = (int)(image_mean.at<double>(0, 0));
	int image_media_value = image_pixel_value.at(image_pixel_value.size() / 2);
	cv::Mat mean_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(image_mean_value - 10));  //����ֵȫΪ��ֵ���ص�ͼ��
																					   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\mean_image.png", mean_image);
	cv::Mat mode_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(mode_value));  //����ֵȫΪ�������ص�ͼ��
																			//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\mode_image.png", mode_image);
	cv::Mat media_image(HSV_img_h.size(), CV_8UC1, cv::Scalar(image_media_value + 10));  //����ֵȫΪ��ֵ���ص�ͼ��
																						 //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\media_image.png", media_image);

																						 /*ӳ��ͼ������ֵ����д��*/
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
																						 //cout << "����ֵ������!" << endl;


	/*͹������ȡ��ӳ��ͼ�����ֵͼ��*/	cv::Mat Convex;
	cv::subtract(HSV_img_h, mode_image, Convex);
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Convex_noise.png", Convex);

	Mat Convex_clour(Convex.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	/*��ֵͼ���ӳ��ͼ��������*/
	cv::Mat concave;
	cv::subtract(mode_image, HSV_img_h, concave);
	//cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Concave_noise.png", concave);
	Mat concave_clour(concave.size(), CV_8UC3, cv::Scalar(255, 255, 255));

	/*������ȱ��������ȡ����*/
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

	/******************************͹ȱ�ݴ���*******************************/
	cout << "͹��ȱ�ݼ����������ʼ��" << endl;
	/*ͼ���ֵ�����˲�*/
	float minimum_defect = 0.2;	//����͹��ȱ�ݴ���minimum_defect mm�ĲŻᱻ���
	int Binarization_thresholdabs = (int)((minimum_defect + abs(pixel_small))*enlarge_rate) - mode_value;
	Mat Convex_binary;
	//threshold(Convex, Convex_binary, 8, 255, THRESH_BINARY);  //��ֵ��
	cv::threshold(Convex, Convex_binary, Binarization_thresholdabs, 255, THRESH_BINARY);  //��ֵ��,����Binarization_thresholdabs����ֵ��ȫ������Ϊ255��Binarization_thresholdabs�����ټ���͹�఼��ȱ�ݿ���
	Mat Convex_binary_filter;
	cv::medianBlur(Convex_binary, Convex_binary_filter, 3);  //��ֵ��ͼ���˲�
															 //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Convex_binary_filter" + to_string(minimum_defect) +".png", Convex_binary_filter);

															 /*ȱ��������ȡ*/
	vector<vector<Point>> contours; contours.clear();
	vector<Vec4i> hierarchy;
	cv::findContours(Convex_binary_filter, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point());  //������ȡ
	Mat imageContours = Mat::zeros(Convex.size(), CV_8UC1);
	Mat Contours = Mat::zeros(Convex.size(), CV_8UC1);  //����
	for (int i = 0; i < contours.size(); i++) //��������
	{
		//contours[i]������ǵ�i��������contours[i].size()������ǵ�i�����������е����ص���
		for (int j = 0; j < contours[i].size(); j++)
		{
			//���Ƴ�contours���������е����ص�
			Point P = Point(contours[i][j].x, contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		//��������
		drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
	}
	//imshow("Contours Image", imageContours); //����
	//imshow("Point of Contours", Contours);   //����contours�ڱ�������������㼯

	/*ȱ��������Ӿ�*/
	std::vector<Contour_point> Contour_point_4; Contour_point_4.clear();
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(contours[i]);  //����͹����Ȼ�������Ӿ�
		Point2f P[4];
		rect.points(P); //������Ӿ�4������
						//��ÿ��������4���������
		Contour_point contour_point;
		contour_point.point_1 = P[0]; contour_point.point_2 = P[1];	contour_point.point_3 = P[2]; contour_point.point_4 = P[3]; //��Ӿص�4������
		Contour_point_4.push_back(contour_point);	//�洢
		for (int j = 0; j <= 3; j++)
		{
			//Convex_binary_filter,�������P[j]�ߵĵ�һ���㣬P[(j + 1) % 4]�ߵĵڶ����㣬cv::Scalar(0,0,255)�ߵ���ɫ��1�ߵĴ�ϸ��8�ߵ����ͣ�0�ߵ�ƫ����
			line(Convex_binary_filter, P[j], P[(j + 1) % 4], cv::Scalar(255), 1, 8, 0);  //CV_8UC1�Ҷ�ͼ���ܻ��Ʋ�ɫ����
		}
		//cout << "�� "<< i <<" ��͹��ȱ����Ӿض�����ȡ���..." << endl;
	}
	/*ȷ��ÿһ��ȱ�ݰ��������ص�*/
	std::vector<std::vector<Point2f>> defect_location; defect_location.clear();
	for (int i = 0; i < Contour_point_4.size(); i++) //����ÿһ����Ӿ�
	{
		std::vector<int> x_vector; x_vector.clear(); std::vector<int> y_vector; y_vector.clear();  //��ÿһ��ȱ����Ӿ��ο��4�����ó���
		int x1 = Contour_point_4.at(i).point_1.x; int y1 = Contour_point_4.at(i).point_1.y;
		x_vector.push_back(x1); y_vector.push_back(y1);
		int x2 = Contour_point_4.at(i).point_2.x; int y2 = Contour_point_4.at(i).point_2.y;
		x_vector.push_back(x2); y_vector.push_back(y2);
		int x3 = Contour_point_4.at(i).point_3.x; int y3 = Contour_point_4.at(i).point_3.y;
		x_vector.push_back(x3); y_vector.push_back(y3);
		int x4 = Contour_point_4.at(i).point_4.x; int y4 = Contour_point_4.at(i).point_4.y;
		x_vector.push_back(x4); y_vector.push_back(y4);
		sort(x_vector.begin(), x_vector.end());	sort(y_vector.begin(), y_vector.end());   //����õ�ȱ����Ӿ��ο�4����������Сxֵ��yֵ
		int x_small = x_vector.front(); int x_big = x_vector.back(); //����ѡ��Ѹղ����ȱ����Ӿ�������4�������ϣ����������Ҷ���һ�����أ���Ϊ��Ӿ���ѹ�ߵ����
		int y_small = y_vector.front(); int y_big = y_vector.back();
		/*����ȱ�ݷֲ�ͼ�����������أ�Ȼ������ÿһ�����ο�x_small��y_small���루x_big��y_big���в�����0�����������ó���*///�൱�ڸ�ȱ�ݶ�λ
		std::vector<Point2f> once_defet_location; once_defet_location.clear();
		//for (int j = 0; j < Convex.cols;j++)
		//{
		//	for (int k = 0; k<Convex.rows; k++)
		//	{
		//		if ((x_small < j) && (j < x_big) && (y_small < k) && (k < y_big) && (Convex.at<uchar>(k,j)>0)) //���ȱ�ݷֲ�ͼ�������������ھ��ο��ڣ���������ֵ����0
		//		{
		//			Point2f once =  Point2f(j, k);
		//			once_defet_location.push_back(once);
		//		}
		//	}
		//}
		for (int j = 0; j < Convex.cols; j++) //�������Ӿ��ڵ����ش���Binarization_thresholdabs������Ϊ��ʵȱ������
		{
			for (int k = 0; k < Convex.rows; k++)
			{
				if ((x_small <= j) && (j <= x_big) && (y_small <= k) && (k <= y_big) && (Convex.at<uchar>(Point(j, k)) >= Binarization_thresholdabs)) //���ȱ�ݷֲ�ͼ�������������ھ��ο��ڣ���������ֵ����Binarization_thresholdabs
				{
					Point2f once = Point2f(j, k);  //j�����ı�ţ�k�ǿ�ı��
					once_defet_location.push_back(once);
				}
			}
		}
		if (once_defet_location.size() < 5) //���һ��ȱ��С��5�����أ�������
		{
			continue;
		}
		defect_location.push_back(once_defet_location);
		//cout << "�� " << i << " ��͹��ȱ��������ȡ���..." << endl;
	}
	cout << "͹��ȱ�ݼ����������ɣ�" << endl;

	/*����ȱ�ݸ���ͬ��ɫ���ڶ�άͼ���Ͻ�����ʾ�������ж�ȱ�ݶ�λ����ȷ�ԣ����Ƿ��в��ǵ�ǰȱ�ݵĵ㱻�жϽ���*/
	for (int i = 0; i<defect_location.size(); i++) //ȱ�ݸ���
	{
		int b, g, r; b = g = r = 0;
		random_colour(b, g, r);
		for (int j = 0; j < defect_location.at(i).size(); j++)
		{
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[0] = b;  //������ͨ����bͨ��������x = i.y = j����λ�ø�ֵ
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[1] = g;
			Convex_clour.at<Vec3b>(defect_location.at(i).at(j))[2] = r;
		}
	}

	/*��ȱ����������������ֵ����*/
	//ͼ��x��Ӧ����ԭʼԲ���ϵ�����߶ȷ���ͼ��y��Ӧ����ԭʼԲ������(x,y)��Ӧ��ά���ƴ�����ǵ�y���ϵ�xƬ
	Defection_quantification.clear();
	double max_convex_pixel = 0;  //������¼����������͹���͹ֵ��������ɫӳ�䣬������ɫӳ��Խ��
	for (int i = 0; i < defect_location.size(); i++)
	{
		double each_slice_height = slice_height; //ǰ������߶��Ѿ�����
		double each_slice_width = (each_strip_angle * cylinder.radius*M_PI) / 180;	//�������ڽǶ�x�뾶������ֱ���õ�Բ���뾶��û�м������Ԫ���İ뾶����Ϊ������̫��
		double defection_one_pixel_area = each_slice_height * each_slice_width;     //�����һ�����ش�������������Ļ��͵���ԭʼͼ������ֵ���Ը����
		double defection_area = defection_one_pixel_area*defect_location.at(i).size();	//ȱ���������ÿ�����ص��������ȱ����ռ��������
																						//��ԭʼӳ��ͼ����ȥ�Ҷ�Ӧλ�õ����ؼ������
		double max_pixel_value = orgin_image.at<float>(defect_location.at(i).at(0));  //�ȼٶ�ȱ�ݾ����������ֵ��ΪĿ���ڵ�һ��
		double once_defection_volume = 0;
		int x_tmp = 0; int y_tmp = 0;  //��ȱ����͹�ĵط���¼��������������ά��ָ��λ��
		for (int k = 0; k < defect_location.at(i).size(); k++)
		{
			if (max_pixel_value < orgin_image.at<float>(defect_location.at(i).at(k)))
			{
				max_pixel_value = orgin_image.at<float>(defect_location.at(i).at(k));//�ѵ�ǰȱ�����������ֵ�ҳ���
				x_tmp = defect_location.at(i).at(k).x;  //���ı��
				y_tmp = defect_location.at(i).at(k).y;	//��ı��
			}
			once_defection_volume = once_defection_volume + (orgin_image.at<float>(defect_location.at(i).at(k))*defection_one_pixel_area);  //����ȱ�������ȱ�����=ȱ�����������ֵ*���
		}
		if (max_pixel_value>max_convex_pixel)  //�ѵ�ǰ�������������ֵ�ҳ���
		{
			max_convex_pixel = max_pixel_value;
		}

		defection_quantification once_detection;	//����ȱ�ݵĸ��������д洢
		once_detection.defection_area = defection_area;		//ȱ��Ӱ��Բ�����
		once_detection.defection_area_defection = defection_area + defection_area/10;  //ȱ�ݱ����
		once_detection.defection_deepest_point = max_pixel_value;//��͹���͹ֵ
		once_detection.defection_loaction_x = x_tmp;		//ȱ����͹���xֵ����ı��
		once_detection.defection_loaction_y = y_tmp;		//ȱ����͹���yֵ�����ı��
		once_detection.defection_volume = once_defection_volume; //���
		once_detection.conductEnd_2_outEnd_distance1 = conductEnd_2_outEnd_distance - x_tmp *each_slice_height;//��͹�������о���˵ľ���ֵ
		once_detection.defection_flag = i; //�ѵ�ǰȱ�ݵı�ż�¼����
		Defection_quantification.push_back(once_detection);

		/*�������ȷ����Ѳ���*/
		//fstream f1;
		//f1.open("convex_R40.txt", ios::out | ios::app);
		//f1 << "H = ," << slice_height << "," << "A = ," << each_strip_angle << ",";
		//f1 << defection_area;
		//f1 << ",";
		//f1 << once_defection_volume;
		//f1 << endl;
		//f1.close();

		//cout << "����׶� " << x_tmp *each_slice_height << " mm��͹��ȱ��������ɣ�ȱ�ݱ��Ϊ " << i << endl;
	}

	/*����ά�н�ȱ�ݽ�����ʾ����ͬ��ɫ������ʾΪ��ͬ��ȱ���ֱ�ʾΪ��ͬ�̶ȵ�͹��*/
	//int colour_rate = 120 / max_convex_pixel;  //����λ����ֵ���ĵط�
	//for (int i = 0;i < Defection_quantification.size();i++)  //ȱ�ݸ���
	//{
	//	int defect_location_index = Defection_quantification.at(i).defection_flag;
	//	for (int j = 0; j < defect_location.at(defect_location_index).size();j++)  //��ǰȱ�ݰ�����СƬ���Ƹ���
	//	{
	//		int slice_index = defect_location.at(defect_location_index).at(j).x;
	//		int strip_index = defect_location.at(defect_location_index).at(j).y;
	//		for (int k = 0;k < slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->size();k++)  //�ҵ�ָ��СƬ���и�ֵ,СƬȱ����������ĵ���
	//		{
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 0;
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
	//			slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = orgin_image.at<float>(defect_location.at(i).at(j))*colour_rate;  //����͹��߶�ֵ����ɫ
	//		}
	//	}
	//}
	int colour_rate = 120 / 1;  //�̶�����Ϊ1mm�����ı��ĸ���ܸı�����
	for (int i = 0; i < Defection_quantification.size(); i++)  //ȱ�ݸ���
	{
		int defect_location_index = Defection_quantification.at(i).defection_flag;
		for (int j = 0; j < defect_location.at(defect_location_index).size(); j++)  //��ǰȱ�ݰ�����СƬ���Ƹ���
		{
			int slice_index = defect_location.at(defect_location_index).at(j).x;
			int strip_index = defect_location.at(defect_location_index).at(j).y;
			for (int k = 0; k < slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->size(); k++)  //�ҵ�ָ��СƬ���и�ֵ,СƬȱ����������ĵ���
			{
				slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 0;
				slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
				if (orgin_image.at<float>(defect_location.at(i).at(j)) > 1)  //����HSV��ɫ�ռ�Խ��
				{
					slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = 120; //����Խ��
				}
				else
				{
					slice_vector.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = (orgin_image.at<float>(defect_location.at(i).at(j)) / 1)*(colour_rate);  //����͹��߶�ֵ����ɫ
				}
			}
		}
	}

	/*��ʾ��ɫ�����ά����*/
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
	//showpointcloud_xyzrgb(colour_point_cloud);  //͹ȱ��Ϊ��ɫ����������ɫΪ��ɫ
	/*HSV��ɫ��ʾ������Ϊ��ɫ��͹��Ϊ��ɫ*/
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
	//showpointcloud_xyzhsv(colour_point_cloud_hsv);  //͹ȱ����ʾ��͹ȱ����ʾΪ��ɫ��Խ͹Խ�죬�����Ͱ�ȱ�ݶ�Ϊ��ɫ
	/******************************͹ȱ�ݴ���*******************************/

	/*************************************��ȱ�ݴ���**************************************/
	cout << "����ȱ�ݼ����������ʼ��" << endl;
	/*ͼ���ֵ�����˲�*/
	float minimum_defect_concave = -0.2;	//���ư�ȱ�ݴ���minimum_defect mm�ĲŻᱻ���
	int Binarization_thresholdabs_concave = mode_value - (int)((minimum_defect_concave + abs(pixel_small))*enlarge_rate);
	Mat concave_binary;
	cv::threshold(concave, concave_binary, Binarization_thresholdabs_concave, 255, THRESH_BINARY);  //��ֵ��
	Mat concave_binary_filter;
	cv::medianBlur(concave_binary, concave_binary_filter, 3);  //��ֵ��ͼ���˲�
															   //cv::imwrite("C:\\Users\\15734\\Desktop\\big_paper_8_27\\region_growing_12_14\\region_growing\\region_growing\\paperImag\\standerd\\Concave_binary_filter" + to_string(minimum_defect)+".png", concave_binary_filter);
															   /*ȱ��������ȡ*/
	vector<vector<Point>> contours_concave; contours_concave.clear();
	vector<Vec4i> hierarchy_concave;
	cv::findContours(concave_binary_filter, contours_concave, hierarchy_concave, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, Point());  //������ȡ
	Mat imageContours_concave = Mat::zeros(concave.size(), CV_8UC1);
	Mat Contours_concave = Mat::zeros(concave.size(), CV_8UC1);  //����
	for (int i = 0; i < contours_concave.size(); i++) //��������
	{
		//contours_concave[i]������ǵ�i��������contours_concave[i].size()������ǵ�i�����������е����ص���
		for (int j = 0; j < contours_concave[i].size(); j++)
		{
			//���Ƴ�contours_concave���������е����ص�
			Point P_concave = Point(contours_concave[i][j].x, contours_concave[i][j].y);
			Contours_concave.at<uchar>(P_concave) = 255;
		}
		//��������
		drawContours(imageContours_concave, contours_concave, i, Scalar(255), 1, 8, hierarchy_concave);
	}
	//imshow("Contours_concave Image", imageContours_concave); //����
	//imshow("Point of Contours_concave", Contours_concave);   //����contours_concave�ڱ�������������㼯

	/*ȱ��������Ӿ���ȡ*/
	std::vector<Contour_point> Contour_point_4_concave; Contour_point_4_concave.clear();
	for (int i = 0; i < contours_concave.size(); i++)
	{
		RotatedRect rect_concave = minAreaRect(contours_concave[i]);
		Point2f P_concave[4];
		rect_concave.points(P_concave);
		//��ÿ��������4���������
		Contour_point contour_point_concave;
		contour_point_concave.point_1 = P_concave[0]; contour_point_concave.point_2 = P_concave[1];	contour_point_concave.point_3 = P_concave[2]; contour_point_concave.point_4 = P_concave[3]; //��Ӿص�4������
		Contour_point_4_concave.push_back(contour_point_concave);	//�洢
		for (int j = 0; j <= 3; j++)
		{
			//Convex_binary_filter,�������P_concave[j]�ߵĵ�һ���㣬P_concave[(j + 1) % 4]�ߵĵڶ����㣬cv::Scalar(0,0,255)�ߵ���ɫ��1�ߵĴ�ϸ��8�ߵ����ͣ�0�ߵ�ƫ����
			line(concave_binary_filter, P_concave[j], P_concave[(j + 1) % 4], cv::Scalar(255), 1, 8, 0);  //CV_8UC1�Ҷ�ͼ���ܻ��Ʋ�ɫ����
		}
		//cout << "�� " << i << " ����ȱ����Ӿض�����ȡ���..." << endl;
	}
	/*������Ӿ�ȷ��ÿһ��ȱ�ݰ��������ص�����*/
	std::vector<std::vector<Point2f>> defect_location_concave; defect_location_concave.clear();  //�洢ÿ��ȱ�ݰ�������������
	for (int i = 0; i < Contour_point_4_concave.size(); i++) //����ÿһ����Ӿ�
	{
		std::vector<int> x_vector; x_vector.clear(); std::vector<int> y_vector; y_vector.clear();  //��ÿһ��ȱ����Ӿ��ο��4�����ó���
		int x1_concave = Contour_point_4_concave.at(i).point_1.x; int y1_concave = Contour_point_4_concave.at(i).point_1.y;
		x_vector.push_back(x1_concave); y_vector.push_back(y1_concave);
		int x2_concave = Contour_point_4_concave.at(i).point_2.x; int y2_concave = Contour_point_4_concave.at(i).point_2.y;
		x_vector.push_back(x2_concave); y_vector.push_back(y2_concave);
		int x3_concave = Contour_point_4_concave.at(i).point_3.x; int y3_concave = Contour_point_4_concave.at(i).point_3.y;
		x_vector.push_back(x3_concave); y_vector.push_back(y3_concave);
		int x4_concave = Contour_point_4_concave.at(i).point_4.x; int y4_concave = Contour_point_4_concave.at(i).point_4.y;
		x_vector.push_back(x4_concave); y_vector.push_back(y4_concave);
		sort(x_vector.begin(), x_vector.end());	sort(y_vector.begin(), y_vector.end());   //����õ�ȱ����Ӿ��ο�4����������Сxֵ��yֵ
		int x_small_concave = x_vector.front(); int x_big_concave = x_vector.back(); //����ѡ��Ѹղ����ȱ����Ӿ�������4�������ϣ����������Ҷ���һ�����أ���Ϊ��Ӿ���ѹ�ߵ����
		int y_small_concave = y_vector.front(); int y_big_concave = y_vector.back();
		/*����ȱ�ݷֲ�ͼ�����������أ�Ȼ������ÿһ�����ο�x_small_concave��y_small_concave���루x_big_concave��y_big_concave���в�����0�����������ó���*///�൱�ڸ�ȱ�ݶ�λ
		std::vector<Point2f> once_defet_location_concave; once_defet_location_concave.clear();
		//for (int j = 0; j < Convex.cols;j++)
		//{
		//	for (int k = 0; k<Convex.rows; k++)
		//	{
		//		if ((x_small < j) && (j < x_big) && (y_small < k) && (k < y_big) && (Convex.at<uchar>(k,j)>0)) //���ȱ�ݷֲ�ͼ�������������ھ��ο��ڣ���������ֵ����0
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
				//���ȱ�ݷֲ�ͼ�������������ھ��ο��ڣ���������ֵС��minimum_defect_concave
				if ((x_small_concave <= j) && (j <= x_big_concave) && (y_small_concave <= k) && (k <= y_big_concave) && (concave.at<uchar>(Point(j, k)) >= Binarization_thresholdabs_concave))
				{
					Point2f once_concave = Point2f(j, k);
					once_defet_location_concave.push_back(once_concave);
				}
			}
		}
		if (once_defet_location_concave.size() < 5) //���һ��ȱ��С��5�����أ�������
		{
			continue;
		}
		defect_location_concave.push_back(once_defet_location_concave);
		//cout << "�� " << i << " ����ȱ��������ȡ���..." << endl;
	}
	/*����ȱ�ݸ���ͬ��ɫ�ģ��ڶ�άͼ������ʾ*/
	for (int i = 0; i < defect_location_concave.size(); i++)
	{
		int b, g, r; b = g = r = 0;
		random_colour(b, g, r);
		for (int j = 0; j < defect_location_concave.at(i).size(); j++)
		{
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[0] = b;  //������ͨ����bͨ��������x = i.y = j����λ�ø�ֵ
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[1] = g;
			concave_clour.at<Vec3b>(defect_location_concave.at(i).at(j))[2] = r;
		}
	}

	/*��ȱ����������������ֵ����*/
	//ͼ��x��Ӧ����ԭʼԲ���ϵ�����߶ȷ���ͼ��y��Ӧ����ԭʼԲ������(x,y)��Ӧ��ά���ƴ�����ǵ�y���ϵ�xƬ
	Defection_quantification_concave.clear();
	double min_concave_pixel = 0; //�������µ���͵�
	for (int i = 0; i < defect_location_concave.size(); i++)	//ȱ������
	{
		double each_slice_height_concave = slice_height; //ǰ������߶��Ѿ�����
		double each_slice_width_concave = (each_strip_angle * cylinder.radius*M_PI) / 180;	//�������ڽǶ�x�뾶������ֱ���õ�Բ���뾶��û�м������Ԫ���İ뾶����Ϊ������̫��
		double defection_one_pixel_area_concave = each_slice_height_concave * each_slice_width_concave;     //�����һ�����ش�������������Ļ��͵���ԭʼͼ������ֵ���Ը����
		double defection_area_concave = defection_one_pixel_area_concave*defect_location_concave.at(i).size();	//ȱ���������ÿ�����ص��������ȱ����ռ��������
																												//��ԭʼӳ��ͼ����ȥ�Ҷ�Ӧλ�õ����ؼ��������ͬʱ�ҳ���С����ֵ�����ֵ��Ӧ������͵�
		double min_pixel_value_concave = orgin_image.at<float>(defect_location_concave.at(i).at(0));  //�ȼٶ�ȱ�ݾ�����С����ֵ��ΪĿ���ڵ�һ��
		double once_defection_volume_concave = 0;
		int x_tmp_concave = 0; int y_tmp_concave = 0;  //��ȱ����͹�ĵط���¼��������������ά��ָ��λ��
		for (int k = 0; k < defect_location_concave.at(i).size(); k++)
		{
			if (min_pixel_value_concave > orgin_image.at<float>(defect_location_concave.at(i).at(k)))
			{
				min_pixel_value_concave = orgin_image.at<float>(defect_location_concave.at(i).at(k));
				x_tmp_concave = defect_location_concave.at(i).at(k).x;  //���ı��
				y_tmp_concave = defect_location_concave.at(i).at(k).y;	//��ı��
			}
			once_defection_volume_concave = once_defection_volume_concave + (abs(orgin_image.at<float>(defect_location_concave.at(i).at(k)))*defection_one_pixel_area_concave);  //����ȱ�������ȱ�����=ȱ�����������ֵ*���
		}
		if (min_pixel_value_concave < min_concave_pixel)  //�ѵ�ǰ������ȱ����С����ֵ�ҳ���
		{
			min_concave_pixel = min_pixel_value_concave;
		}

		defection_quantification once_detection_concave;			//����ȱ�ݵĸ��������д洢
		once_detection_concave.defection_area = defection_area_concave;		//ȱ��Ӱ���Բ�����
		once_detection_concave.defection_area_defection = defection_area_concave + defection_area_concave/10;  //ȱ�ݱ����
		once_detection_concave.defection_deepest_point = min_pixel_value_concave;//���İ�ֵ
		once_detection_concave.defection_loaction_x = x_tmp_concave;		//ȱ������xֵ����Ӧ�������ı�ţ�x,y����Ӧ���ǵ�x���y��
		once_detection_concave.defection_loaction_y = y_tmp_concave;		//ȱ������yֵ����Ӧ���ǿ�ı��
		once_detection_concave.defection_volume = once_defection_volume_concave; //���
		once_detection_concave.conductEnd_2_outEnd_distance1 = conductEnd_2_outEnd_distance - x_tmp_concave * each_slice_height_concave;//��͹�������о���˵ľ���ֵ
		once_detection_concave.defection_flag = i; //�ѵ�ǰȱ�ݵı�ż�¼����
		Defection_quantification_concave.push_back(once_detection_concave);

		/*�������ȷ����Ѳ���*/
		//fstream f;
		//f.open("concave_R40.txt", ios::out | ios::app);
		//f << "H = ," << slice_height << "," << "A = ," << each_strip_angle << ",";
		//f << defection_area_concave;
		//f << ",";
		//f << once_defection_volume_concave;
		//f << endl;
		//f.close();

		//cout << "����׶� " << x_tmp_concave * each_slice_height_concave << " mm����ȱ��������ɣ�ȱ�ݱ��Ϊ " << i << endl;
	}
	cout << "����ȱ�ݼ����������ɣ�" << endl;

	/*����ά�н�ȱ�ݽ�����ʾ����ͬ��ɫ������ʾΪ��ͬ��ȱ���ֱ�ʾΪ��ͬ�̶ȵ�͹��*/   //��͹���Ѿ���͹����ɫ�������������ɫ
												//int colour_rate_concave = 120 / (abs(min_concave_pixel));   //��������ֵ��ӳ��ɫӳ��ֵ
	int colour_rate_concave = 120;  //�޶�����Ϊ1mm��1mm�����϶�������ɫ��ʾ
	for (int i = 0; i < Defection_quantification_concave.size(); i++)  //ȱ�ݸ���
	{
		int defect_location_index_concave = Defection_quantification_concave.at(i).defection_flag;
		for (int j = 0; j < defect_location_concave.at(defect_location_index_concave).size(); j++)  //��ǰȱ�ݰ�����СƬ���Ƹ���
		{
			int slice_index = defect_location_concave.at(defect_location_index_concave).at(j).x;
			int strip_index = defect_location_concave.at(defect_location_index_concave).at(j).y;
			for (int k = 0; k < slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->size(); k++)  //�ҵ�ָ��СƬ���и�ֵ,СƬȱ����������ĵ���
			{
				if (orgin_image.at<float>(defect_location_concave.at(i).at(j)) < -1)//����HSV��ɫ�ռ�Խ��
				{
					slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = 120;
				}
				else
				{
					slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).b = (abs(orgin_image.at<float>(defect_location_concave.at(i).at(j))) / 1)*colour_rate_concave;
				}
				slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).g = 0;
				slice_vector1.at(strip_index).at(slice_index).pointcloud_strip_slice->at(k).r = 0;  //����͹��߶�ֵ����ɫ
			}
		}
	}
	/*��ʾ��ɫ�����ά���ƣ������ǰ�ɫ������������ɫ*/
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
	//showpointcloud_xyzrgb(colour_point_cloud_concave);  //��ʾ��������,xyzrgb������ʾ����ֱ���ʾ����vector1����һ����ʾ����vector

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr colour_point_cloud_hsv_concave(new pcl::PointCloud<pcl::PointXYZHSV>);
	for (int i = 0; i < colour_point_cloud_concave->size(); i++)
	{
		pcl::PointXYZHSV point_temp_concave;
		point_temp_concave.x = colour_point_cloud_concave->at(i).x;
		point_temp_concave.y = colour_point_cloud_concave->at(i).y;
		point_temp_concave.z = colour_point_cloud_concave->at(i).z;
		if (colour_point_cloud_concave->at(i).b != 255)    //�ҳ�������
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
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);  //������ʾ��ȱ�ݣ�����Ϊ��ɫ������Ϊ��ɫ

	for (int i = 0; i<colour_point_cloud_hsv->size(); i++)	//ͬһ����������ʾ��͹ȱ��
	{
		if (colour_point_cloud_hsv->at(i).h != 120)
		{
			colour_point_cloud_hsv_concave->push_back(colour_point_cloud_hsv->at(i));
		}
	}
	//showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);  //������Ե������ͬʱ��ʾ������͹��ȱ��

	//fstream fx;
	//fx.open("data.txt", ios::out | ios::app);
	//fx << "ֵ�ֱ��ǰ���ȱ������������͹����������_A= " + to_string(each_strip_angle) + "_h= " + to_string(slice_height) << "," << Defection_quantification_concave.at(0).defection_area << " ," << Defection_quantification_concave.at(0).defection_volume
	//	<< "," << "," << Defection_quantification.at(0).defection_area << "," << Defection_quantification.at(0).defection_volume << endl;
	////showpointcloud_xyzhsv(colour_point_cloud_hsv_concave);
	//cout << "ȱ�����" << "A" + to_string(each_strip_angle) + "_h_" + to_string(slice_height) + "����ȱ�����ֵ = "
	//	<< Defection_quantification_concave.at(0).defection_area << " ���ֵ = " << Defection_quantification_concave.at(0).defection_volume << endl;
	//cout << "ȱ�����" << "A" + to_string(each_strip_angle) + "_h_" + to_string(slice_height) + "͹��ȱ�����ֵ = "
	//	<< Defection_quantification.at(0).defection_area << " ���ֵ = " << Defection_quantification.at(0).defection_volume << endl;
	/*************************************��ȱ�ݴ���**************************************/
	/*����������*/
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

	/*����СƬ�����ߵķ�ʽ������άͼ*/
	//std::vector<Mapping_2D_3D> mapping_2D_3D; mapping_2D_3D.clear();
	//std::vector<float> slice_distance; slice_distance.clear();
	//cv::Mat pointcloud_img = cv::Mat::zeros(cv::Size( slice_vector.at(0).size(), slice_vector.size()),CV_32FC1);
	//for (int i = 0; i < slice_vector.size(); i++) //��������
	//{
	//	for (int j = 0;j<slice_vector.at(i).size();j++) //ÿ����Ƭ������
	//	{
	//		float slice_point_2_line_distance_count = 0;
	//		for (int k = 0;k<slice_vector.at(i).at(j).pointcloud_strip_slice->size();k++)
	//		{
	//			Eigen::Vector4f point;
	//			point << cylinder.point_on_axis_x, cylinder.point_on_axis_y, cylinder.point_on_axis_z;   //Բ��������һ��
	//			Eigen::Vector4f normal;
	//			normal << cylinder.axis_direction_x, cylinder.axis_direction_y, cylinder.axis_direction_z;  //Բ�����߷�������
	//			Eigen::Vector4f point_cloud_point;
	//			point_cloud_point << slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).x, slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).y,
	//				slice_vector.at(i).at(j).pointcloud_strip_slice->at(k).z;
	//			float point_2_line_distance = sqrt(pcl::sqrPointToLineDistance(point_cloud_point, point, normal));// �ռ�㵽�ռ�ֱ�߾��룬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
	//			slice_point_2_line_distance_count = slice_point_2_line_distance_count + point_2_line_distance;
	//		}
	//		float slice_2_line_distance = slice_point_2_line_distance_count / slice_vector.at(i).at(j).pointcloud_strip_slice->size();
	//		Mapping_2D_3D mapping;
	//		mapping.x = i;
	//		mapping.y = j;
	//		mapping.pixel_value = slice_2_line_distance;
	//		mapping_2D_3D.push_back(mapping);	
	//		float sitance_average = slice_2_line_distance - cylinder.radius;  //��Ԫ��Բ�����߾����ȥԲ���뾶
	//		slice_distance.push_back(sitance_average);
	//		pointcloud_img.at<float>(i, j) = sitance_average;
	//	}
	//}
	//sort(slice_distance.begin(), slice_distance.end());

	/*����ͼ���лҶȷֲ�ֱ��ͼ*/
	/*ֱ��ͼ*/
	//������ȡֱ��ͼ����ر���
	//Mat hist;  //���ڴ��ֱ��ͼ������
	//const int channels[1] = { 0 };  //ͨ������
	//float inRanges[2] = { 0,255 };
	//const float* ranges[1] = { inRanges };  //���ػҶ�ֵ��Χ
	//const int bins[1] = { 256 };  //ֱ��ͼ��ά�ȣ���ʵ�������ػҶ�ֵ�����ֵ
	//calcHist(&HSV_img_h, 1, channels, Mat(), hist, 1, bins, ranges);  //����ͼ��ֱ��ͼ
	////׼������ֱ��ͼ
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

	cout << "����Ե���氼��͹��ȱ�ݼ��������������ɣ�" << endl;


	return true;
}
/*������Ӹ�˹����*/
//bool add_gauss_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)
//{
//	cout << "******��Ӹ�˹������******" << endl;
//	cout << "��ӽ������Ƴߴ磺" << pointcloud_filtered->size() << endl;
//	showpointcloud_xyzrgb(pointcloud_filtered);
//	// ����XYZ��γ�ȵľ�ֵ�ͱ�׼��
//	float xmean = 0, ymean = 0, zmean = 0;
//	float xstddev = 0.1, ystddev = 0.1, zstddev = 0.1;
//	// ---------------------------���ɸ�˹�ֲ��ĵ�������---------------------------------------
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
//	// ---------------------------��Ӹ�˹�ֲ����������--------------------------------------
//	for (size_t i = 0; i < pointcloud_filtered->points.size(); ++i)
//	{
//		gauss_cloud->points[i].x += pointcloud_filtered->points[i].x;
//		gauss_cloud->points[i].y += pointcloud_filtered->points[i].y;
//		gauss_cloud->points[i].z += pointcloud_filtered->points[i].z;
//	}
//
//
//	printf("��˹���������ϣ�����");
//	cout << "��ӽ������Ƴߴ磺" << pointcloud_filtered->size() <<endl;
//	showpointcloud_xyzrgb(gauss_cloud);
//	return true;
//}
/*��С����ƽ�����*/
bool least_square_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pointCloud, float &A, float &B, float &C, float &D)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Vector4d centroid;                    // ����
	Eigen::Matrix3d covariance_matrix;           // Э�������
												 // �����һ��Э������������
	pcl::computeMeanAndCovarianceMatrix(*plane_pointCloud, covariance_matrix, centroid);
	// ����Э������������ֵ����������
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);
	// ������С����ֵ��λ��
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);
	// ��ȡƽ�淽�̣�AX+BY+CZ+D = 0��ϵ��
	Eigen::Vector3d normal = eigenVectors.col(minCol);
	D = -normal.dot(centroid.head<3>());
	A = normal[0];
	B = normal[1];
	C = normal[2];

	return true;
}
/*RANSACƽ�����*/
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D)
{
	A = B = C = D = 0;
	//��ÿһ��СƬ����ƽ�����
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//����һ��ģ�Ͳ����������ڼ�¼���			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// ����һ���ָ���			
	seg.setOptimizeCoefficients(true);	// Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣			
	seg.setModelType(pcl::SACMODEL_PLANE);	// Mandatory-����Ŀ�꼸����״����Ҫ��ϵ���״		
	seg.setMethodType(pcl::SAC_RANSAC);	//�ָ�������������			
	seg.setDistanceThreshold(0.1);	//����������̷�Χ��Ҳ������ֵ  ͬ���ƽ��ľ��볬����ֵ�ĵ㣬�ͱ��ж�Ϊ��Ч���ݡ�
	seg.setInputCloud(strip_one_slice);	//�������			
	seg.segment(*inliers, *coefficients);	//�ָ���ƣ����ƽ��ͷ����� coefficients�д洢����ƽ��ģ�͵�ϵ��A/B/C/D��inliers�洢��ϳ�ƽ��ĵ�,��������ģ�͵�����

	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];
	return 0;
}
/*RANSACƽ����Ϻ�������,���ڷָ�*/
//bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice,float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point)
//{
//	A = B = C = D = 0;
//	//��ÿһ��СƬ����ƽ�����
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//����һ��ģ�Ͳ����������ڼ�¼���			
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����			
//	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// ����һ���ָ���			
//	seg.setOptimizeCoefficients(true);	// Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣			
//	seg.setModelType(pcl::SACMODEL_PLANE);	// Mandatory-����Ŀ�꼸����״����Ҫ��ϵ���״		
//	seg.setMethodType(pcl::SAC_RANSAC);	//�ָ�������������			
//	seg.setDistanceThreshold(0.1);	//����������̷�Χ��Ҳ������ֵ  ͬ���ƽ��ľ��볬����ֵ�ĵ㣬�ͱ��ж�Ϊ��Ч���ݡ�
//	seg.setInputCloud(strip_one_slice);	//�������			
//	seg.segment(*inliers, *coefficients);	//�ָ���ƣ����ƽ��ͷ����� coefficients�д洢����ƽ��ģ�͵�ϵ��A/B/C/D��inliers�洢��ϳ�ƽ��ĵ�,��������ģ�͵�����
//
//	A = coefficients->values[0];
//	B = coefficients->values[1];
//	C = coefficients->values[2];
//	D = coefficients->values[3];
//
//	//ȡ���ڵ�
//	pcl::copyPointCloud(*strip_one_slice, inliers->indices, *interior_point);
//
//	return 0;
//}
/*RANSACƽ����Ϻ������أ��������*/
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point)
{
	interior_point->clear();
	A = B = C = D = 0;
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(strip_one_slice));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);//����RANSAC�㷨ģ��
	ransac.setDistanceThreshold(0.01);//�趨������ֵ
	ransac.setMaxIterations(1000);     //��������������
	ransac.setProbability(0.99);      //���ô���Ⱥֵ��ѡ������һ����������������
	ransac.computeModel();            //���ƽ��
	vector<int> inliers;              //���ڴ���ڵ�������vector
	ransac.getInliers(inliers);       //��ȡ�ڵ�����

	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);  //��ȡ���ƽ�������coeff�ֱ�˳�򱣴�a,b,c,d

	A = coeff[0];
	B = coeff[1];
	C = coeff[2];
	D = coeff[3];

	//ȡ���ڵ�
	pcl::copyPointCloud(*strip_one_slice, inliers, *interior_point);
	return true;
}
/*���߹���*/
bool show_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice, pcl::PointCloud <pcl::Normal>::Ptr normals)
{

	cout << "������ʾ����������!" << endl;
	pcl::visualization::PCLVisualizer viewer("show_normals");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(slice, "slice");
	//��ʾ��ǰСƬ�ķ��ߣ����У�slice��ʾԭʼ����ģ�ͣ�normals��ʾ������Ϣ��1��ʾ��Ҫ��ʾ���ߵĵ��Ƽ���������ʾÿ���㶼��ʾ����0.5��ʾ���򳤶�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(slice, 255, 0, 0);  //���õ�����ɫ
	viewer.addPointCloud<pcl::PointXYZRGB>(slice, single_color, "sample cloud");  //�������ϵ
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(slice, normals, 5, 2.5, "normals");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");  //�޸ĵ��ƴ�С
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return true;
}
/*���ص���*/
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
		std::cout << "��ͷ���Ƽ��سɹ�!" << std::endl;
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
/*��ʾxyzrgbnormal���͵ĵ���*/
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
/*��ʾxyzrgbnor���͵ĵ���*/
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
/*��ʾxyz���͵ĵ���*/
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
/*��ʾxyzhsv���͵ĵ���*/
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
/*ransic���Բִ�к���*/
bool ransice_circular_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Circle &circle)
{
	//cout << "->���ڹ��ƿռ�Բ..." << endl;
	circle.m_center_x = circle.m_center_y = circle.m_center_z = circle.m_radius = 0;
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>(cloud_ptr));	//ѡ����ϵ����뼸��ģ��
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3D);	//�����������һ���Զ���
	ransac.setDistanceThreshold(0.1);	//���þ�����ֵ����ģ�;���С��0.1�ĵ���Ϊ�ڵ�
	ransac.setMaxIterations(15000);		//��������������
	ransac.computeModel();				//ִ��ģ�͹���
	std::vector<int> inliers;			//�洢�ڵ�����������
	ransac.getInliers(inliers);			//��ȡ�ڵ��Ӧ������

										// ����������ȡ�ڵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_ptr, inliers, *cloud_circle);
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	circle.m_center_x = coefficient[0];  //Ϊ�˷��ظ����ú�������Բ��������뾶�����ô���
	circle.m_center_y = coefficient[1];
	circle.m_center_z = coefficient[2];
	circle.m_radius = coefficient[3];

	/*�Ա�չʾ������ϵķ�Χ������Ͻ��*/

	//cout << "�ռ�Բ�ģ�"
	//	<< "(" << coefficient[0] << ","
	//	<< coefficient[1] << ","
	//	<< coefficient[2] << ")"
	//	<< endl;
	//cout << "�뾶��" << coefficient[3] << endl;
	//cout << "�ռ�Բ��������"
	//	<< "(" << coefficient[4] << ","
	//	<< coefficient[5] << ","
	//	<< coefficient[6] << ")"
	//	<< endl;

	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("��Ͻ��"));
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, "cloud");											//���ԭʼ����
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	//��ɫ
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");	//��Ĵ�С

	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_circle, "circle");										//���ģ�͵���,��Ͻ��
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "circle");	//��ɫ
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "circle");	//��Ĵ�С
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	return true;
}
/*��������ƽ����˲�����z������*/
bool Cable::filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_filtered)		//���ô���
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;									//�����뾶�˲���;
	outrem.setInputCloud(m_pointcloud_rgb);												//�����������;
	std::cout << "�����ͷ���Ƶ�����" << m_pointcloud_rgb->size() << std::endl;			//�˲�ǰ��������
	outrem.setRadiusSearch(1);															//���ð뾶Ϊ1�ķ�Χ�����ٽ���;
	outrem.setMinNeighborsInRadius(3);													//���ò�ѯ�������㼯��С��3��ɾ��;
	outrem.filter(*pointcloud_filtered);												//ִ�������˲�������洢��pointcloud_filtered��
																						//cout << "����ǰ��ǰ����z���ֵ = " << pointcloud_filtered->at(pointcloud_filtered->size() - 1).z << "��ǰ����z��Сֵ = " << pointcloud_filtered->at(0).z << endl;
	rankpointcloud_z(pointcloud_filtered);		   //�Ե��°�zֵ����
	cout << "�����ͷ����z�����ֵ = " << pointcloud_filtered->at(pointcloud_filtered->size() - 1).z << ",  " << "�����ͷ����z����Сֵ = " << pointcloud_filtered->at(0).z << endl;
	//std::cout << "�˲������������" << pointcloud_filtered->size() << std::endl;		//�˲�������������
	//showpointcloud_xyzrgb(pointcloud_filtered);
	return true;
}
/*��zֵ�����㷨*/
bool Cable::rankpointcloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank)
{
	int i;
	for (i = pointcloud_rank->size() / 2 - 1; i >= 0; --i)
	{
		heapify_z(pointcloud_rank, i, pointcloud_rank->size() - 1);  //����
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
/*zֵ�����㷨���Ӻ���*/
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
/*�Ե��ư��̶�zֵ����Բ���*/
bool Cable::z_value_circle_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double distance)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr once_circle_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	rankpointcloud_z(pointcloud_filtered);		//���˲���ĵ��ư�z���������,ȷ�����ڵ����ļ��������һ���ǰ�zֵ��С������ȡ��
	if (m_indexandcircles.empty())				//�ж�m_indexandcircles�Ƿ�Ϊ�գ��洢Բ������ȫ�ֱ��������Ϊ�վ�ִ��Բ��ϵĲ���
	{
		if (m_pointcloud_rgb->empty())			//�ж�����ĵ����Ƿ�Ϊ��
		{
			std::cout << "point cloud is empty!" << std::endl;
			return false;
		}

		pcl::PointXYZRGB point;
		pcl::PointXYZRGB point2;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_fit(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int> index;								//�������Բ�������

		point = pointcloud_filtered->at(0);					//�����ļ������ĵ�һ����
		double z_min = pointcloud_filtered->at(0).z;		//�����ļ������ĵ�z����͵��zֵ
		double z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;		//�������Ƶ���ߵ��zֵ
																						//cout <<"Բ��Ͽ�ʼ�� "<< "cable_size_min: " << z_min << "," << " cable_size_max��" << z_max << endl;
		cout << endl;
		cout << "���ڽ��н�ͷ����λ��У���� " << endl;


		m_indexandcircles.clear();
		//�ȼ�(distance / 2)��Ϊ�˱���Բ�ĺ��ֻ�������ط�Բ��һ��
		for (double k = z_min + (distance / 2); k < z_max; k = k + distance)    //���Բ�����ҳ��㣬Ȼ�����Բ���洢���Բ���������洢���Բ�����е��Ƶ��������ÿ��Բ��1mm
		{
			//ֱ��ָ��xy���任zֵ���Ӷ��ҳ���ͬz�߶ȵĵ���
			point2.x = point.x;
			point2.y = point.y;
			point2.z = k;

			index.clear();         //ÿ��ѭ����ʼ��
			pointcloud_fit->clear();
			once_circle_points->clear();
			for (int t = 0; t < pointcloud_filtered->size(); t++)  //һ��forѭ�����ҳ�һ����Χ�ĵ���д洢�������������Բ
			{
				//point2.z�����ӵ�ͨ���ٽ��������ҳ������ٽ��㣬�����˲�������ļ������ҳ������ٽ���zֵС��һ����Χ�ĵ�
				double theta = pointcloud_filtered->at(t).z - point2.z;
				if (theta < distance / 2 && theta > -distance / 2)
				{
					pointcloud_fit->push_back(pointcloud_filtered->at(t));  //pointcloud_fit�洢���ӵ�߶�һ����Χ�ڵĵ����ά��Ϣ
					index.push_back(t);			//index�洢���ӵ�߶�һ����Χ�ڵ�������������������˲���ĵ����ļ����ĸ�λ�ã������Ǵ洢�ĵ�����ݣ���ά�ռ�λ�ã������Ǵ洢�ĵ㴦��������λ�ã�
				}
			}
			if (pointcloud_fit->size())  //���ҳ����ĵ����Բ
			{
				once_circle_points = pointcloud_fit->makeShared();

				Circle circle;   //�ṹ��Բ������Բ�ģ�x��y��z���Ͱ뾶
				ransice_circular_fitting(pointcloud_fit, circle);		//�ٽ������Բ
				IndexofCircle indexofcircle;		//�ṹ�壬����һ���ṹ��Բ�ģ������������洢���Բ�����е��Ƶ��������
				indexofcircle.once_circle = circle;	//circleleastfit��ϵ�Բ��ֵ���ṹ��indexofcircle�����Բ
				indexofcircle.each_circle_index = index;		//ÿһ��Բ������Ƶ������������������˲�������������ļ���˵�ģ���Χ�������˲���ĵ��ƣ�
				indexofcircle.each_circle_points = once_circle_points->makeShared();
				//m_indexandcirclesȫ�ֱ����洢����ÿ�����Բ�Ľṹ����Ϣ��Բ��Բ�ġ��뾶����ÿ�����Բ�ϵ��ƹ��ɵ�vector���������棬ÿһ��ѭ���洢һ�����Բ�Ľṹ����Ϣ
				m_indexandcircles.push_back(indexofcircle);		//m_indexandcircles�洢����ÿ�����Բ�Ľṹ����Ϣ������
			}
			//std::cout << "Բ�����ɣ�" << k / z_max * 100 << "%";    //������
			printf("\r");
		}


	}
	/*��ʾһ��Բ�����ڲ鿴Բ�����*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pointcloud_temp->clear();
	//for (int i = 0; i < m_indexandcircles.at(0).each_circle_index.size();i++)
	//{
	//	pointcloud_temp->push_back(pointcloud_filtered->at(i));
	//}
	//showpointcloud_xyzrgb(pointcloud_temp);
	//cout << endl;
	//cout << "���Բ�ĸ��� = " << m_indexandcircles.size() << endl;
	cout << "��ͷ����λ��У����ɣ� " << endl;
	return true;

}
/*�ٽ�Բ�뾶��׼�����*/
bool Cable::calculation_standard_deviation(int index, int range, double &standard_fore, double &standard_back)
{
	double fore_sum, back_Sum;
	double average_value_fore, average_value_back;
	double variance_temp_fore, variance_temp_back;
	double variance_fore, variance_back;

	fore_sum = back_Sum = average_value_fore = average_value_back = standard_fore = standard_back
		= variance_temp_fore = variance_temp_back = variance_fore = variance_back = 0;

	for (int i = 1; i <= range; i++)    //�����ֵ
	{
		fore_sum = fore_sum + m_indexandcircles.at(index - i).once_circle.m_radius;
		back_Sum = back_Sum + m_indexandcircles.at(index + i).once_circle.m_radius;
	}
	average_value_fore = fore_sum / range;
	average_value_back = back_Sum / range;

	for (int j = 1; j <= range; j++)  //���㷽��ķ���
	{
		variance_temp_fore = variance_temp_fore + pow(((m_indexandcircles.at(index - j).once_circle.m_radius) - average_value_fore), 2);
		variance_temp_back = variance_temp_back + pow(((m_indexandcircles.at(index + j).once_circle.m_radius) - average_value_back), 2);
	}
	variance_fore = variance_temp_fore / range;    //����
	variance_back = variance_temp_back / range;

	standard_fore = sqrt(variance_fore);
	standard_back = sqrt(variance_back);
	//standard_fore = variance_fore;
	//standard_back = variance_back;
	return true;
}
/*��׼������ָ6_21֮ǰ�汾����*/
/*bool Cable::rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end)*/
//{
//	/*�����Բ�ϲ�ͬ����ɫ������۲�*/
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_circle_colour(new pcl::PointCloud<pcl::PointXYZRGB>);    //rgb�˲���ĵ���
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
//	/*�뾶����д��*/
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
//	//cout << "�뾶������!" << endl;
//	
//	std::cout << "���ڽ�������ָ�!" << endl;
//	region_start.clear();
//	region_end.clear();
//	//����µ���͵㣬ֻ��һ���㲻׼ȷ�����������m���㲢��ƽ�������õ���һ������ȷ����ԭ������Ϊ����ɨ�����
//	int cable_min_point_number = 20;
//	double sum_cable_min_point = 0;
//	for (int i = 1; i <= cable_min_point_number; i++)
//	{
//		sum_cable_min_point = sum_cable_min_point + pointcloud_filtered->at(i).z;
//	}
//	double cable_z_mini_value = sum_cable_min_point / cable_min_point_number;  //��о����͵㣬����о���
//
//	//����µ���ߵ㣬ֻ��һ���㲻׼ȷ�����������m���㲢��ƽ�����������һ�����Բ��Բ��ԭ������Ϊ����ɨ�����
//	int cable_max_point_number = 20;
//	double sum_cable_max_point = 0;
//	for (int i = 1; i <= cable_max_point_number; i++)
//	{
//		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
//	}
//	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //��о����ߵ㣬����о�յ�
//
//
//	double standard_fore, standard_back;    //Բ�뾶�ı�׼��
//	double standard_back_compare_fore, standard_fore_compare_back;
//	std::vector<double> back_compare_fore_all;
//	std::vector<double> fore_compare_back_all;
//	back_compare_fore_all.clear();
//	fore_compare_back_all.clear();
//
//	/*��׼����㣬������Ҫ����Ǳ�׼��Լ�����*/
//	int k = 15; //ÿ����ȡǰ��20��Բ�����׼�Ȼ������׼��֮��
//	for (int i = 0; i < k; i++)  //Ϊ�������Բ������ȣ�ǰ����k��0
//	{
//		back_compare_fore_all.push_back(0);
//		fore_compare_back_all.push_back(0);
//	}
//
//	for (int i = k; i < m_indexandcircles.size() - k; i++)
//	{
//		standard_fore = standard_back = standard_back_compare_fore = standard_fore_compare_back = 0;
//		calculation_standard_deviation(i, k, standard_fore, standard_back);    //����i��������i�����Բ
//
//		standard_back_compare_fore = standard_back / standard_fore;
//		standard_fore_compare_back = standard_fore / standard_back;
//		back_compare_fore_all.push_back(standard_back_compare_fore);
//		fore_compare_back_all.push_back(standard_fore_compare_back);
//	}
//	for (int i = 0; i < k; i++)	//Ϊ�������Բ������ȣ�������k��0
//	{
//		back_compare_fore_all.push_back(0);
//		fore_compare_back_all.push_back(0);
//	}
//
//	/*�뾶��׼������д��*/
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
//	//cout << "��׼���ֵ���ǰ������!" << endl;
//
//	/*�뾶��׼������д��*/
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
//	//cout << "��׼���ֵ���ǰ������!" << endl;
//
//
//	std::vector<Circle> region_circles; //������ÿ�������յ���о��ָ�������������Բ�ģ�ÿ�ε���ǰ�ǵó�ʼ��
//	/**************************---���������---****************************/
//	/**************************---���������---****************************/
//	/**************************---���������---****************************/
//	
//	/*^^^^^^^^^^^^^^^^^^^^^^^��뵼����ɴ����^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	//std::vector<double> back_compare_fore_1;
//	//back_compare_fore_1.clear();
//	//std::vector<int> start1_index_all;  start1_index_all.clear();
//
//	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
//	//{
//	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 370) &&
//	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 310))
//	//	{
//	//		start1_index_all.push_back(i);
//	//	}
//	//}
//	//if (!(start1_index_all.size()))
//	//{
//	//	cout << "��뵼����ɴ����������ȡ�쳣!" << endl;
//	//}
//
//	///***********************����600-680mm���Բ�뾶�ķ�ʽ����뵼����ɴ����***********************************/
//	//std::vector<int> cable_circle_index_size;   //�������Բ����
//	//cable_circle_index_size.clear();
//	//for (int i = 0; i < m_indexandcircles.size(); i++)
//	//{
//	//	cable_circle_index_size.push_back(i);
//	//}
//	//std::vector<int> Outer_semiconducting_circle_index;
//	//Outer_semiconducting_circle_index.clear();
//	//std::vector<int>::const_iterator region_index_start = cable_circle_index_size.begin() + start1_index_all.at(0);
//	//std::vector<int>::const_iterator region_index_end = cable_circle_index_size.begin() + start1_index_all.at(start1_index_all.size() - 1);
//	//Outer_semiconducting_circle_index.assign(region_index_start, region_index_end);   //�Ѵ��ھ��붥��600-680mmԲ�������ó����洢
//
//	///*��׼��ҳ���뵼����ɴ����*/
//	////sort(start_1_start.begin(), start_1_start.end());
//	//std::vector<double>::const_iterator first_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(0);
//	//std::vector<double>::const_iterator second_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(start1_index_all.size() - 1);
//	//back_compare_fore_1.assign(first_back_2_fore_1, second_back_2_fore_1);    //�Ѵ��ڸ÷�Χ�ڵı�׼���ֵ��ȡ����
//	//double goal_1 = back_compare_fore_1.at(0);
//	//int flag = 0;
//	//for (int i = 0; i < back_compare_fore_1.size(); i++)					  //�ҳ��÷�Χ�ڱ�׼���ֵ���ı��
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
//	//double first_start_z_value = m_indexandcircles.at(first_start_index).once_circle.m_center_z;    //��뵼����ɴ����
//
//	/*ֱ�Ӹ��ݾ���ָ����뵼����ɴ������*/
//	
//	double first_start_z_value = cable_z_max_value - 340 + ( 2  / (rand() % 6));    //��뵼����ɴ����
//
//	/*��뵼����ɴ�����׼�����*/
//	//region_circles.clear();
//	//fusion_iteration_detail_segmentation(first_start, 20/circle_distance, 1, region_circles);  //ȡ��������ǰ1cm�����Ը�С��zֵ����Բ��ϣ����½���ǰ���׼��֮�ȼ��㣬�����ҵ�����ֵ			
//	//double first_start_updatez_z_value = 0;
//	//update_segmentation_start(region_circles, k/2, cable_z_mini_value, first_start_updatez_z_value);
//	//double  first_start_update = first_start_updatez_z_value;
//	//cout << "update��뵼����ɴ����zֵ�������µ׶˾��� = " << first_start_updatez_z_value << " mm" << endl;
//	//double first_start_z_value = first_start_updatez_z_value + cable_z_mini_value;
//	//printf("%s", "\n");
//
//
//	/*��뵼����ɴ������������뾶������*/
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
//	/*����������������뵼����ɴ����*/
//	//double core_radius_parameter = 21;
//	//std::vector<int> outer_semi_conduct_start; outer_semi_conduct_start.clear(); //�����������ҳ���뵼���յ�
//	//for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
//	//{
//	//	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius_parameter > 18.9)
//	//	{
//	//		outer_semi_conduct_start.push_back(i);
//	//	}
//	//}
//	//if (!outer_semi_conduct_start.size())
//	//{
//	//	cout << "����������������뵼����ɴ�����쳣" << endl;
//	//}
//	//sort(outer_semi_conduct_start.begin(), outer_semi_conduct_start.end());
//	//int outer_semi_conduct_start_max_index = outer_semi_conduct_start.at(outer_semi_conduct_start.size() - 1) + 1;
//	//double first_start_z_value = m_indexandcircles.at(outer_semi_conduct_start_max_index).once_circle.m_center_z;
//
//
//	/*^^^^^^^^^^^^^^^^^^^^^^^��Ӧ��׶���^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	std::vector<double> back_compare_fore_3;
//	back_compare_fore_3.clear();
//	std::vector<int>start3_index_all;
//	start3_index_all.clear();
//	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
//	{
//		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 105 &&
//			cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 85) //�Ѿ�����о85-105mm��Բ�������ҳ���
//		{
//			start3_index_all.push_back(i);
//		}
//	}
//	if (!(start3_index_all.size()))
//	{
//		cout << "��Ӧ��׶���������ȡ�쳣��" << endl;
//	}
//	std::vector<double>::const_iterator first_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(0);
//	std::vector<double>::const_iterator second_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(start3_index_all.size() - 1);
//	back_compare_fore_3.assign(first_back_2_fore_3, second_back_2_fore_3);
//
//	double goal_3 = back_compare_fore_3.at(0);  //�ҳ�55-135mm��back��fore��������ֵ
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
//	/*�Ż�����*/
//	//region_circles.clear();  //ÿ��ʹ�üǵó�ʼ�������ߺ����������������
//	//fusion_iteration_detail_segmentation(third_start, 10 / circle_distance, 2, region_circles);  //ȡ��������ǰ1cm���½���Բ��ϣ����½���ǰ���׼��֮�ȼ��㣬�����ҵ�����ֵ			
//	//double thrid_start_updatez_z_value = 0;
//	//update_segmentation_start(region_circles, k / 3, cable_z_mini_value, thrid_start_updatez_z_value);
//	//double thrid_start_update = thrid_start_updatez_z_value;
//	//printf("%s", "\n");
//
//
//
//	/**************************---�������յ�---****************************/
//	/**************************---�������յ�---****************************/
//	/**************************---�������յ�---****************************/
//
//	/*^^^^^^^^^^^^^^^^^^^^^^^��뵼����ɴ��յ�^^^^^^^^^^^^^^^^^^^^^^^^^*/
//	//std::vector<double> fore_compare_back_1;
//	//fore_compare_back_1.clear();
//	//std::vector<int> region_circle_index_end_1;		region_circle_index_end_1.clear();
//	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
//	//{
//	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 310) &&
//	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 290))
//	//	{
//	//		region_circle_index_end_1.push_back(i);
//	//	}
//	//}
//	//if (!(region_circle_index_end_1.size()))
//	//{
//	//	cout << "��뵼����ɴ��յ�������ȡ�쳣��" << endl;
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
//	/*������뵼�������о�ĺ�����ò�������쵼����ɴ��յ������ȡ*/
//	//double core_radius = 21; //�õ��µ���о�뾶
//	//std::vector<int> outer_semi_conduct_end; outer_semi_conduct_end.clear(); //�����������ҳ���뵼���յ�
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
//	//	cout << "��뵼����ɴ��յ�������⣡" << endl;
//	//	return 0;
//	//}
//
//
//		/*^^^^^^^^^^^^^^^^^^^^^^^�ڰ뵼������о����^^^^^^^^^^^^^^^^^^^^^^^^^*/
//		 //�ڸ���������ڣ�����������Բ�İ뾶��������
//		std::vector<IndexofCircle> fore_compare_back_3;
//		fore_compare_back_3.clear();
//
//		std::vector<int> end_3_all_index;	end_3_all_index.clear();//20-60
//
//		for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
//		{
//
//			if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 25) &&
//				(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 40))
//			{
//				end_3_all_index.push_back(i);  //������������Բ�������洢����
//			}
//
//		}
//
//		if (!(end_3_all_index.size()))
//		{
//			cout << "�ڰ뵼������о�������ȡ�쳣��" << endl;
//		}
//		std::vector<IndexofCircle>::const_iterator first_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(0);
//		std::vector<IndexofCircle>::const_iterator second_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(end_3_all_index.size() - 1);
//		fore_compare_back_3.assign(first_fore_2_back_3, second_fore_2_back_3);
//
//		//double adjacent_radius_distance = fore_compare_back_3.at(0).once_circle.m_radius - fore_compare_back_3.at(1).once_circle.m_radius; 
//		//int flag_back_3 = 0;
//		//for (int i = 0; i < fore_compare_back_3.size() - 1; i++)  //�ҳ�������Χ�ڰ뾶��ֵ������ĵط�
//		//{
//		//	//cout << fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius << " " << i <<endl;
//		//	if (fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius > adjacent_radius_distance)
//		//	{
//		//		adjacent_radius_distance = fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius;
//		//		flag_back_3 = i;
//		//	}
//		//}
//		//int thrid_end = flag_back_3 + m_indexandcircles.size() - end_3.size();  //����������ڰ뵼������о����
//		//double thrid_end_disance = m_indexandcircles.at(thrid_end).once_circle.m_center_z - cable_z_mini_value; //�������µ׶˾���
//		//cout << "--�ڰ뵼������о�����������µ׶˾��� = " << thrid_end_disance << " mm" << endl;
//
//		double adjacent_radius_distance = 0;
//		int flag_back_3 = 0;
//		for (int i = end_3_all_index.at(0) ; i < end_3_all_index.at(end_3_all_index.size() - 1) - 2; i++)  //�ҳ�������Χ�ڰ뾶��ֵ������ĵط�
//		{
//
//			if ((m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius) > adjacent_radius_distance)
//			{
//				adjacent_radius_distance = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius;
//				flag_back_3 = i + 1;
//			}
//		}
//		int inner_semiconducting_end_index = flag_back_3 ;  //����������ڰ뵼������о������������ƫ��о��
//		double inner_semiconducting_end_z_value = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z ;   //�ڰ뵼������о�Ľ���
//		
//
//		/*���ݺ���ҳ��ڰ뵼�����*/
//		double cable_core_radius_average = 21;   //�趨��о����뾶Ϊ21mm
//		std::vector<int> inner_semiconducting_index;    //�洢���ݺ���ҳ������ڰ뵼��Բ������
//		inner_semiconducting_index.clear();
//
//		//for (int i = end_3_all_index.at(0); i<end_3_all_index.at(end_3_all_index.size() - 1); i++)
//		//{
//		//	if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
//		//		&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 1.95)    //����ֵ�����ڰ뵼������λ��
//		//		&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //������о��ɢ���³���
//		//	{
//		//		inner_semiconducting_index.push_back(i);
//		//	}
//		//}
//		for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1); i++)
//		{
//			if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
//				&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 2.2)    //����ֵ�����ڰ뵼������λ��
//				&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //������о��ɢ���³���
//			{
//				inner_semiconducting_index.push_back(i);
//			}
//		}
//
//		double inner_semiconducting_start_z_value_finally = 0;
//		if (inner_semiconducting_index.size())   //����д�ĥ�ڰ뵼�磬����ζ�Ÿ�������Ϊ��
//		{
//			sort(inner_semiconducting_index.begin(), inner_semiconducting_index.end());   //���ҳ������ڰ뵼��Բ�������������������i��С���Ǹ��ط�
//			int inner_semiconducting_start_i = inner_semiconducting_index.at(0);  //�ڰ뵼�����Բ������,�ҳ��ڰ뵼������о�Ľ���������ڰ뵼�����ҳ�����
//			double inner_semiconducting_start_z_value = m_indexandcircles.at(inner_semiconducting_index.at(0)).once_circle.m_center_z;
//			inner_semiconducting_start_z_value_finally = inner_semiconducting_start_z_value;
//		}
//		else   //���û�д�ĥ�ڰ뵼�磬���ڰ뵼�����������о���ڰ嵼��Ľ���㣬Ҳ�����ڰ뵼����Ϊ0
//		{
//			inner_semiconducting_start_z_value_finally = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;
//			std::cout << "�ڰ뵼���������о����غϣ�" << std::endl;
//		}
//
//		/*���·ָ����洢*/
//		region_start.push_back(first_start_z_value);  //��뵼����ɴ�����zֵ
//		region_start.push_back(second_start_z_value);  //��Ӧ��׶����zֵ
//		region_start.push_back(inner_semiconducting_start_z_value_finally);  //�ڰ뵼�����㣬Ҳ�ǵڶ���Ǧ��ͷ�յ��zֵ
//		region_start.push_back(inner_semiconducting_end_z_value);  //��о����zֵ��Ҳ���ڰ뵼���յ�zֵ
//		/*���·ָ��յ�洢*/
//		region_end.push_back(first_end_z_value);  //��뵼����ɴ��յ��zֵ,����һ��
//		region_end.push_back(inner_semiconducting_start_z_value_finally);  //��Ӧ��׶�յ��zֵ
//		region_end.push_back(inner_semiconducting_end_z_value);  //�ڰ뵼���յ��zֵ
//		region_end.push_back(cable_z_max_value);  //��о�յ��zֵ	
//
//		/*�Էָ����ֶ���ɫ��ʾ*/
//		for (int n = 0; n < pointcloud_filtered->size(); n++)
//		{
//			if (pointcloud_filtered->at(n).z < region_start.at(0))  //��뵼��
//			{
//				pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(0) && pointcloud_filtered->at(n).z < region_end.at(0))  //��뵼����ɴ�
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 255;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_end.at(0) && pointcloud_filtered->at(n).z < region_start.at(1))  //��һ��ƽ������
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(1) && pointcloud_filtered->at(n).z < region_end.at(1))  //Ǧ��ͷ
//			{
//				pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
//			}
//			else if (pointcloud_filtered->at(n).z > region_end.at(1) && pointcloud_filtered->at(n).z < region_start.at(2))  //�ڰ뵼��
//			{
//				pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
//			}
//			else if (pointcloud_filtered->at(n).z > region_start.at(3) && pointcloud_filtered->at(n).z < region_end.at(3))  //��о
//			{
//				pointcloud_filtered->at(n).r = 198;			pointcloud_filtered->at(n).g = 145;			pointcloud_filtered->at(n).b = 69;
//			}
//		}
//
//		showpointcloud_xyzrgb(pointcloud_filtered);		//�ֶ���ʾ
//
//		std::vector<int> inner_semiconducting_circle_index; inner_semiconducting_circle_index.clear();
//		std::vector<int> XLPE_insulation_circle_index; XLPE_insulation_circle_index.clear();
//
//		for (int i = 0; i<m_indexandcircles.size();i++)
//		{
//			if (region_start.at(2)< m_indexandcircles.at(i).once_circle.m_center_z && 
//				m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(3))  //�ڰ뵼��Բ�����ó���
//			{
//				inner_semiconducting_circle_index.push_back(i);
//			}
//
//			if (region_end.at(0) < m_indexandcircles.at(i).once_circle.m_center_z &&
//				m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(1))  //XLPE��Ե��Բ�����ó���
//			{
//				XLPE_insulation_circle_index.push_back(i);
//			}
//		}
//
//		std::vector<double> inner_semiconducting_circle_radius; inner_semiconducting_circle_radius.clear();
//		std::vector<double> XLPE_insulation_circle_radius; XLPE_insulation_circle_radius.clear();
//		for (int i = 2; i<inner_semiconducting_circle_index.size(); i++)
//		{
//			inner_semiconducting_circle_radius.push_back(m_indexandcircles.at(inner_semiconducting_circle_index.at(i)).once_circle.m_radius);  ////�ڰ뵼��Բ�뾶�ó���
//		}
//		sort(inner_semiconducting_circle_radius.begin(), inner_semiconducting_circle_radius.end());
//
//		for (int i = 0; i < XLPE_insulation_circle_index.size(); i++)
//		{
//			XLPE_insulation_circle_radius.push_back(m_indexandcircles.at(XLPE_insulation_circle_index.at(i)).once_circle.m_radius);		//XLPE��Ե��ԲԲ�뾶�ó���
//		}
//		sort(XLPE_insulation_circle_radius.begin(), XLPE_insulation_circle_radius.end());
//
//		/*���㷴Ӧ��׶б�³���*/
//		double xiepo = pow((2 * (XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))) - (2 * (inner_semiconducting_circle_radius.at(0))), 2) +
//			pow((region_end.at(1) - region_start.at(1)), 2);
//		double Length_S = sqrt(xiepo);
//
//		cout << "\n" << endl;
//		cout << "���峤�ȣ�" << region_end.at(3) - region_start.at(3) << " mm" << endl;
//		cout << "�ڰ뵼��㳤�ȣ�" << region_start.at(3) - region_start.at(2) << endl;
//		cout << "�ڰ뵼����⾶��" << 2 * inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
//		cout << "XLPE��Ե���ȣ�" << region_start.at(1) - region_end.at(0) << " mm" << endl;
//		cout << "XLPE�⾶��" << 2 * XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2)) << " mm" << endl;
//		cout << "��뵼����ɴ����ȣ�" << region_end.at(0) - region_start.at(0) << " mm" << endl;
//		//cout << "��Ӧ��׶�����յ��⾶:Di:" <<2*(inner_semiconducting_circle_radius.at(0)) << endl;
//		//cout << "��Ӧ��׶��������Ե�⾶:Do:" << 2*(XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))<< endl;
//		//cout << "��Ӧ��׶б�³��ȣ�S:" << Length_S << endl;
//		cout << "��Ӧ��׶����߶ȣ�H:" << region_end.at(1) - region_start.at(1) << " mm" << endl;
//		cout << "��ͷ���߸߶ȣ�" << cable_z_max_value - region_end.at(0) << " mm" << endl;
//
//		//return 0;
//
//		/*��������Ƭ������СƬ���߼н�*/
//	
//		/*ȡ���ֲ�����*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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
//		/*���㷨�ߣ����߹���*/
//		//regionSegmentation(pointcloud_filtered_local);   //�Բ��ֵ��ƽ��г���
//		//pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//		//pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//		//pcl::PointCloud <pcl::Normal>::Ptr normals_resit(new pcl::PointCloud <pcl::Normal>);
//		//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;			   //�������߹��ƶ���
//		//normal_estimator.setSearchMethod(tree);				//������������
//		//normal_estimator.setInputCloud(m_index_strip.at(0).each_strip);				//���÷��߹��ƶ�������㼯
//		//normal_estimator.setKSearch(8);						// �������ڷ��������Ƶ�k������Ŀ  ���������ڶ��ٵ���ģ��ƽ����㷨��
//		//normal_estimator.compute(*normals);					//���㲢���������
//		//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//		//show_normals(m_index_strip.at(0).each_strip, normals);
//		//for (int i =0;i<normals->size();i++)
//		//{
//		//	normals->at(i).normal_x = (normals->at(i).normal_x) * (-1);
//		//	normals->at(i).normal_y = (normals->at(i).normal_y) * (-1);
//		//	normals->at(i).normal_z = (normals->at(i).normal_z) * (-1);
//		//	normals_resit->push_back(normals->at(i));
//		//}
//		//show_normals(m_index_strip.at(0).each_strip, normals_resit);  //���߷����ض���
//		//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
//
//		/*���Բ��ɫ�ֲ������ó���ʾ*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local1(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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
//		/***************************************************************ƽ���������***************************************************************************************************/
//		/*���������ȡ�������Ͼֲ�����Ȼ����Ͽռ�ƽ������*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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
//		/*������Ƭ�ľֲ�������*/
//		showpointcloud_xyzrgb(pointcloud_filtered_local);
//
//		regionSegmentation(pointcloud_filtered_local);   //�Բ��ֵ��ƽ��г���
//		/*����������ӻ�*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		for (int i = 0; i < m_index_strip.size(); i++)
//		{
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//			{
//				pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//			}
//		}
//
//		/*������ͬ��ɫ��ʾ*/
//		for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
//		{
//			m_index_strip.at(0).each_strip->at(i).r = 255;
//			m_index_strip.at(0).each_strip->at(i).g = 255;
//			m_index_strip.at(0).each_strip->at(i).b = 255;
//		}
//		/*��ʾĳһ��*/
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
//		Strip_2_slice(pointcloud_filtered_local);	//����Ƭ
//		/*��Ƭ������ӻ�*/
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		for (int i = 0; i < m_index_strip.size(); i++)
//		{
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//			{
//				pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//			}
//		}
//		/*��ʾĳһƬ*/
//		for (int i = 0;i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size();i++)
//		{
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
//			m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;
//
//		}
//		showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
//		/*��Ƭ��ͬ��ɫ��ʾ*/
//		showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);
//
//		/*ĳ���Ͽ�Ԫ����д��*/
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
//		//cout << "��Ԫ����������!" << endl;
//
//		//regionSegmentation(pointcloud_filtered);   //�Բ��ֵ��ƽ��г���
//		//Strip_2_slice(pointcloud_filtered);	//����Ƭ
//
//		/*ƽ���������*/
//		for (int i = 0; i<m_index_strip.size();i++ )	//����ÿһ��
//		{
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ�����
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ����ϣ���������ƽ�潻�ߣ�
//			for (int j = 0; j < m_index_strip.at(i).each_strip->size();j++)  //�ڵ�ǰ��������������������Ƭ״�ֲ����Ƴ�����Ȼ����ƽ���������
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
//			for (int m = 0;m<piece_local_point_cloud_1->size();m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
//			{
//				pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
//				m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
//			}
//			for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
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
//			ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //һ�������ھֲ����ƽ�����
//			float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
//			ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //һ�������ھֲ����ƽ�����
//
//			Eigen::Vector4f plane_a, plane_b;       //������ƽ�潻��
//			Eigen::VectorXf line;
//			plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
//			plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
//			pcl::planeWithPlaneIntersection(plane_a, plane_b, line,1e-6);
//			std::cout << "�ֱཻ��Ϊ��\n" << line << endl;   //(line[0],line[1],line[2])Ϊ����ֱ������һ�㣬��line[3],line[4],line[5]��Ϊֱ�ߵķ�����
//			//cout << "��֤����������ƽ���ϣ� " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;
//
//		/*���ӻ�*/
//		//	////���ӻ�
//		//	//pcl::visualization::PCLVisualizer viewer;
//		//	////���ӻ�ƽ��
//		//	//pcl::ModelCoefficients plane, plane1;
//		//	//for (size_t k = 0; k < 4; ++k)
//		//	//{
//		//	//	plane.values.push_back(plane_a[k]);
//		//	//	plane1.values.push_back(plane_b[k]);
//		//	//}
//		//	//viewer.addPlane(plane, "plane", 0);
//		//	//viewer.addPlane(plane1, "plane1", 0);
//		//	////���ӻ�ƽ�潻��
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
//		/*����ռ�һ�㵽ֱ�ߵľ���*/
//			Eigen::Vector4f point;
//			//point << line[0], line[1], line[2], 0;     //ֱ����һ��
//			point << line[0], line[1], line[2];     //ֱ����һ���룬x,y,z
//			Eigen::Vector4f normal;
//			normal << line[3], line[4], line[5];		//ֱ�ߵķ�������
//			std::vector<float> point_2_line_distance; point_2_line_distance.clear();
//			for (int k = 0;k < m_index_strip.at(i).each_strip->size();k++)
//			{
//				float distance = 0;
//				Eigen::Vector4f point_cloud_point;
//				point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
//				distance =  pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// �ռ�㵽�ռ�ֱ�߾��룬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
//				point_2_line_distance.push_back(distance);
//			}
//			float small_distance = point_2_line_distance.at(0);
//			int index_of_small_distance = 0;
//			
//			for (int k =0;k<point_2_line_distance.size();k++)//�ҳ���ǰ������һ����������ֱ�����
//			{
//				if (point_2_line_distance.at(k) < small_distance)
//				{
//					small_distance = point_2_line_distance.at(k);
//					index_of_small_distance = k;
//				}
//			}
//			std::cout << "У�����С������zֵ��" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
//			std::cout << "У��ǰ����ֵ = " << cable_z_max_value - line[2] << endl;
//			std::cout << "У�������ֵ = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
//		}
//		/***************************************************************ƽ���������***************************************************************************************************/
//
//		/***************************************************************ƽ��������ߣ���뵼����ɵ����***************************************************************************************************/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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
//		///*������Ƭ�ľֲ�������*/
//		//showpointcloud_xyzrgb(pointcloud_filtered_local);
//
//		//regionSegmentation(pointcloud_filtered_local);   //�Բ��ֵ��ƽ��г���
//		///*����������ӻ�*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		//for (int i = 0; i < m_index_strip.size(); i++)
//		//{
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//		//	{
//		//		pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//		//	}
//		//}
//
//		///*������ͬ��ɫ��ʾ*/
//		//for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
//		//{
//		//	m_index_strip.at(0).each_strip->at(i).r = 255;
//		//	m_index_strip.at(0).each_strip->at(i).g = 255;
//		//	m_index_strip.at(0).each_strip->at(i).b = 255;
//		//}
//		///*��ʾĳһ��*/
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
//		//Strip_2_slice(pointcloud_filtered_local);	//����Ƭ
//		///*��Ƭ������ӻ�*/
//		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
//		//for (int i = 0; i < m_index_strip.size(); i++)
//		//{
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
//		//	{
//		//		pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
//		//	}
//		//}
//		///*��ʾĳһƬ*/
//		//for (int i = 0;i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size();i++)
//		//{
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
//		//	m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;
//
//		//}
//		//showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
//		///*��Ƭ��ͬ��ɫ��ʾ*/
//		//showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);
//
//		///*ĳ���Ͽ�Ԫ����д��*/
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
//		////cout << "��Ԫ����������!" << endl;
//
//		////regionSegmentation(pointcloud_filtered);   //�Բ��ֵ��ƽ��г���
//		////Strip_2_slice(pointcloud_filtered);	//����Ƭ
//
//		///*ƽ���������*/
//		//for (int i = 0; i<m_index_strip.size();i++ )	//����ÿһ��
//		//{
//		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ�����
//		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ����ϣ���������ƽ�潻�ߣ�
//		//	for (int j = 0; j < m_index_strip.at(i).each_strip->size();j++)  //�ڵ�ǰ��������������������Ƭ״�ֲ����Ƴ�����Ȼ����ƽ���������
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
//		//	for (int m = 0;m<piece_local_point_cloud_1->size();m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
//		//	{
//		//		pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
//		//		m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
//		//	}
//		//	for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
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
//		//	ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //һ�������ھֲ����ƽ�����
//		//	float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
//		//	ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //һ�������ھֲ����ƽ�����
//
//		//	Eigen::Vector4f plane_a, plane_b;       //������ƽ�潻��
//		//	Eigen::VectorXf line;
//		//	plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
//		//	plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
//		//	pcl::planeWithPlaneIntersection(plane_a, plane_b, line,1e-6);
//		//	std::cout << "�ֱཻ��Ϊ��\n" << line << endl;   //(line[0],line[1],line[2])Ϊ����ֱ������һ�㣬��line[3],line[4],line[5]��Ϊֱ�ߵķ�����
//		//	//cout << "��֤����������ƽ���ϣ� " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;
//
//		///*���ӻ�*/
//		////	////���ӻ�
//		////	//pcl::visualization::PCLVisualizer viewer;
//		////	////���ӻ�ƽ��
//		////	//pcl::ModelCoefficients plane, plane1;
//		////	//for (size_t k = 0; k < 4; ++k)
//		////	//{
//		////	//	plane.values.push_back(plane_a[k]);
//		////	//	plane1.values.push_back(plane_b[k]);
//		////	//}
//		////	//viewer.addPlane(plane, "plane", 0);
//		////	//viewer.addPlane(plane1, "plane1", 0);
//		////	////���ӻ�ƽ�潻��
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
//		///*����ռ�һ�㵽ֱ�ߵľ���*/
//		//	Eigen::Vector4f point;
//		//	//point << line[0], line[1], line[2], 0;     //ֱ����һ��
//		//	point << line[0], line[1], line[2];     //ֱ����һ���룬x,y,z
//		//	Eigen::Vector4f normal;
//		//	normal << line[3], line[4], line[5];		//ֱ�ߵķ�������
//		//	std::vector<float> point_2_line_distance; point_2_line_distance.clear();
//		//	for (int k = 0;k < m_index_strip.at(i).each_strip->size();k++)
//		//	{
//		//		float distance = 0;
//		//		Eigen::Vector4f point_cloud_point;
//		//		point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
//		//		distance =  pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// �ռ�㵽�ռ�ֱ�߾��룬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
//		//		point_2_line_distance.push_back(distance);
//		//	}
//		//	float small_distance = point_2_line_distance.at(0);
//		//	int index_of_small_distance = 0;
//		//	
//		//	for (int k =0;k<point_2_line_distance.size();k++)//�ҳ���ǰ������һ����������ֱ�����
//		//	{
//		//		if (point_2_line_distance.at(k) < small_distance)
//		//		{
//		//			small_distance = point_2_line_distance.at(k);
//		//			index_of_small_distance = k;
//		//		}
//		//	}
//		//	std::cout << "У�����С������zֵ��" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
//		//	std::cout << "У��ǰ����ֵ = " << cable_z_max_value - line[2] << endl;
//		//	std::cout << "У�������ֵ = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
//		//}
//		/***************************************************************ƽ��������ߣ���뵼����ɵ����***************************************************************************************************/
//
//
//		/*ȡһ��������ÿСƬ����ƽ����ƣ�Ȼ��������߼н�*/
//		pcl::Normal axis_normal; 
//		axis_normal.normal_x = 0;  //ָ������Ϊz��
//		axis_normal.normal_y = 0;
//		axis_normal.normal_z = 1;
//		std::vector<float> one_strip_slice_angle; one_strip_slice_angle.clear();
//		for (int i = 0; i < m_cable_strip_slice_all.at(0).size(); i++)  //��ĳһ���е�СƬ���߼нǽ��м���
//		{
//			float A, B, C, D; A = B = C = D = 0;
//			ransac_plane_estimation(m_cable_strip_slice_all.at(0).at(i).pointcloud_strip_slice, A, B, C, D);  //��СƬ����ƽ����ƣ��Ӷ��ó�ƽ�淨��
//			pcl::Normal once_slice_normal;
//			once_slice_normal.normal_x = A; once_slice_normal.normal_y = B; once_slice_normal.normal_z = C;
//			//double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z);  //����н�����ֵ
//			double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z) /
//				sqrt(pow(once_slice_normal.normal_x,2) + pow(once_slice_normal.normal_y, 2) + pow(once_slice_normal.normal_x, 2) );  //����н�����ֵ
//			double inverse_cosine_value = acos(cos_theta) * 180 / M_PI;  //�нǷ����ң�Ҳ����������ڷ���֮��ļн�
//			one_strip_slice_angle.push_back( 180 -  inverse_cosine_value);  //�������߼н�
//		}
//
//		/*СƬ���߼н�����д��*/
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
//		//cout << "СƬ���߼н�������!" << endl;
//		
//		
//
//	return true;
//}
/*6��21�޸İ汾*/
bool Cable::rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end)
{
	/*�����Բ�ϲ�ͬ����ɫ������۲�*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_circle_colour(new pcl::PointCloud<pcl::PointXYZRGB>);    //rgb�˲���ĵ���
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

	/*�뾶����д��*/
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
	//cout << "�뾶������!" << endl;

	std::cout << endl << "���ڽ��н�ͷ��������ָ��������������!" << endl;
	region_start.clear();
	region_end.clear();
	//����µ���͵㣬ֻ��һ���㲻׼ȷ�����������m���㲢��ƽ�������õ���һ������ȷ����ԭ������Ϊ����ɨ�����
	int cable_min_point_number = 20;
	double sum_cable_min_point = 0;
	for (int i = 1; i <= cable_min_point_number; i++)
	{
		sum_cable_min_point = sum_cable_min_point + pointcloud_filtered->at(i).z;
	}
	double cable_z_mini_value = sum_cable_min_point / cable_min_point_number;  //��о����͵㣬����о���

																			   //����µ���ߵ㣬ֻ��һ���㲻׼ȷ�����������m���㲢��ƽ�����������һ�����Բ��Բ��ԭ������Ϊ����ɨ�����
	int cable_max_point_number = 20;
	double sum_cable_max_point = 0;
	for (int i = 1; i <= cable_max_point_number; i++)
	{
		sum_cable_max_point = sum_cable_max_point + pointcloud_filtered->at(pointcloud_filtered->size() - i).z;
	}
	double cable_z_max_value = sum_cable_max_point / cable_max_point_number;  //��о����ߵ㣬����о�յ�


	double standard_fore, standard_back;    //Բ�뾶�ı�׼��
	double standard_back_compare_fore, standard_fore_compare_back;
	std::vector<double> back_compare_fore_all;
	std::vector<double> fore_compare_back_all;
	back_compare_fore_all.clear();
	fore_compare_back_all.clear();

	/*��׼����㣬������Ҫ����Ǳ�׼��Լ�����*/
	int k = 15; //ÿ����ȡǰ��20��Բ�����׼�Ȼ������׼��֮��
	for (int i = 0; i < k; i++)  //Ϊ�������Բ������ȣ�ǰ����k��0
	{
		back_compare_fore_all.push_back(0);
		fore_compare_back_all.push_back(0);
	}

	for (int i = k; i < m_indexandcircles.size() - k; i++)
	{
		standard_fore = standard_back = standard_back_compare_fore = standard_fore_compare_back = 0;
		calculation_standard_deviation(i, k, standard_fore, standard_back);    //����i��������i�����Բ

		standard_back_compare_fore = standard_back / standard_fore;
		standard_fore_compare_back = standard_fore / standard_back;
		back_compare_fore_all.push_back(standard_back_compare_fore);
		fore_compare_back_all.push_back(standard_fore_compare_back);
	}
	for (int i = 0; i < k; i++)	//Ϊ�������Բ������ȣ�������k��0
	{
		back_compare_fore_all.push_back(0);
		fore_compare_back_all.push_back(0);
	}

	/*�뾶��׼������д��*/
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
	//cout << "��׼���ֵ���ǰ������!" << endl;

	/*�뾶��׼������д��*/
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
	//cout << "��׼���ֵ���ǰ������!" << endl;


	std::vector<Circle> region_circles; //������ÿ�������յ���о��ָ�������������Բ�ģ�ÿ�ε���ǰ�ǵó�ʼ��
										/**************************---���������---****************************/
										/**************************---���������---****************************/
										/**************************---���������---****************************/

										/*^^^^^^^^^^^^^^^^^^^^^^^��뵼����ɴ����^^^^^^^^^^^^^^^^^^^^^^^^^*/
										//std::vector<double> back_compare_fore_1;
										//back_compare_fore_1.clear();
										//std::vector<int> start1_index_all;  start1_index_all.clear();
										//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
										//{
										//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 370) &&
										//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 310))
										//	{
										//		start1_index_all.push_back(i);
										//	}
										//}
										//if (!(start1_index_all.size()))
										//{
										//	cout << "��뵼����ɴ����������ȡ�쳣!" << endl;
										//}

										///***********************����600-680mm���Բ�뾶�ķ�ʽ����뵼����ɴ����***********************************/
										//std::vector<int> cable_circle_index_size;   //�������Բ����
										//cable_circle_index_size.clear();
										//for (int i = 0; i < m_indexandcircles.size(); i++)
										//{
										//	cable_circle_index_size.push_back(i);
										//}
										//std::vector<int> Outer_semiconducting_circle_index;
										//Outer_semiconducting_circle_index.clear();
										//std::vector<int>::const_iterator region_index_start = cable_circle_index_size.begin() + start1_index_all.at(0);
										//std::vector<int>::const_iterator region_index_end = cable_circle_index_size.begin() + start1_index_all.at(start1_index_all.size() - 1);
										//Outer_semiconducting_circle_index.assign(region_index_start, region_index_end);   //�Ѵ��ھ��붥��600-680mmԲ�������ó����洢

										///*��׼��ҳ���뵼����ɴ����*/
										////sort(start_1_start.begin(), start_1_start.end());
										//std::vector<double>::const_iterator first_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(0);
										//std::vector<double>::const_iterator second_back_2_fore_1 = back_compare_fore_all.begin() + start1_index_all.at(start1_index_all.size() - 1);
										//back_compare_fore_1.assign(first_back_2_fore_1, second_back_2_fore_1);    //�Ѵ��ڸ÷�Χ�ڵı�׼���ֵ��ȡ����
										//double goal_1 = back_compare_fore_1.at(0);
										//int flag = 0;
										//for (int i = 0; i < back_compare_fore_1.size(); i++)					  //�ҳ��÷�Χ�ڱ�׼���ֵ���ı��
										//{
										//	if (goal_1 < back_compare_fore_1.at(i))
										//	{
										//		goal_1 = back_compare_fore_1.at(i);
										//		flag = i;
										//	}
										//}
										//int first_start_index = start1_index_all.at(flag + 1);
										//printf("%s", "\n");
										//double first_start_z_value = m_indexandcircles.at(first_start_index).once_circle.m_center_z;    //��뵼����ɴ����

										/*ֱ�Ӹ��ݾ���ָ����뵼����ɴ������*/

	double first_start_z_value = cable_z_max_value - 340 + (2 / (rand() % 6));    //��뵼����ɴ����

																				  /*��뵼����ɴ�����׼�����*/
																				  //region_circles.clear();
																				  //fusion_iteration_detail_segmentation(first_start, 20/circle_distance, 1, region_circles);  //ȡ��������ǰ1cm�����Ը�С��zֵ����Բ��ϣ����½���ǰ���׼��֮�ȼ��㣬�����ҵ�����ֵ			
																				  //double first_start_updatez_z_value = 0;
																				  //update_segmentation_start(region_circles, k/2, cable_z_mini_value, first_start_updatez_z_value);
																				  //double  first_start_update = first_start_updatez_z_value;
																				  //cout << "update��뵼����ɴ����zֵ�������µ׶˾��� = " << first_start_updatez_z_value << " mm" << endl;
																				  //double first_start_z_value = first_start_updatez_z_value + cable_z_mini_value;
																				  //printf("%s", "\n");


																				  /*��뵼����ɴ������������뾶������*/
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


																				  /*����������������뵼����ɴ����*/
																				  //double core_radius_parameter = 21;
																				  //std::vector<int> outer_semi_conduct_start; outer_semi_conduct_start.clear(); //�����������ҳ���뵼���յ�
																				  //for (int i = start1_index_all.at(0); i < start1_index_all.at(start1_index_all.size() - 1); i++)
																				  //{
																				  //	if (m_indexandcircles.at(i).once_circle.m_radius - core_radius_parameter > 18.9)
																				  //	{
																				  //		outer_semi_conduct_start.push_back(i);
																				  //	}
																				  //}
																				  //if (!outer_semi_conduct_start.size())
																				  //{
																				  //	cout << "����������������뵼����ɴ�����쳣" << endl;
																				  //}
																				  //sort(outer_semi_conduct_start.begin(), outer_semi_conduct_start.end());
																				  //int outer_semi_conduct_start_max_index = outer_semi_conduct_start.at(outer_semi_conduct_start.size() - 1) + 1;
																				  //double first_start_z_value = m_indexandcircles.at(outer_semi_conduct_start_max_index).once_circle.m_center_z;


																				  /*^^^^^^^^^^^^^^^^^^^^^^^��Ӧ��׶���^^^^^^^^^^^^^^^^^^^^^^^^^*/
	std::vector<double> back_compare_fore_3;
	back_compare_fore_3.clear();
	std::vector<int>start3_index_all;
	start3_index_all.clear();
	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
	{
		if (cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 105 &&
			cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 85) //�Ѿ�����о85-105mm��Բ�������ҳ���
		{
			start3_index_all.push_back(i);
		}
	}
	if (!(start3_index_all.size()))
	{
		cout << "��Ӧ��׶���������ȡ�쳣��" << endl;
	}
	std::vector<double>::const_iterator first_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(0);
	std::vector<double>::const_iterator second_back_2_fore_3 = back_compare_fore_all.begin() + start3_index_all.at(start3_index_all.size() - 1);
	back_compare_fore_3.assign(first_back_2_fore_3, second_back_2_fore_3);

	double goal_3 = back_compare_fore_3.at(0);  //�ҳ�55-135mm��back��fore��������ֵ
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

	/*�Ż�����*/
	//region_circles.clear();  //ÿ��ʹ�üǵó�ʼ�������ߺ����������������
	//fusion_iteration_detail_segmentation(third_start, 10 / circle_distance, 2, region_circles);  //ȡ��������ǰ1cm���½���Բ��ϣ����½���ǰ���׼��֮�ȼ��㣬�����ҵ�����ֵ			
	//double thrid_start_updatez_z_value = 0;
	//update_segmentation_start(region_circles, k / 3, cable_z_mini_value, thrid_start_updatez_z_value);
	//double thrid_start_update = thrid_start_updatez_z_value;
	//printf("%s", "\n");



	/**************************---�������յ�---****************************/
	/**************************---�������յ�---****************************/
	/**************************---�������յ�---****************************/

	/*^^^^^^^^^^^^^^^^^^^^^^^��뵼����ɴ��յ�^^^^^^^^^^^^^^^^^^^^^^^^^*/
	//std::vector<double> fore_compare_back_1;
	//fore_compare_back_1.clear();
	//std::vector<int> region_circle_index_end_1;		region_circle_index_end_1.clear();
	//for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
	//{
	//	if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 310) &&
	//		(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 290))
	//	{
	//		region_circle_index_end_1.push_back(i);
	//	}
	//}
	//if (!(region_circle_index_end_1.size()))
	//{
	//	cout << "��뵼����ɴ��յ�������ȡ�쳣��" << endl;
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

	/*������뵼�������о�ĺ�����ò�������쵼����ɴ��յ������ȡ*/
	//double core_radius = 21; //�õ��µ���о�뾶
	//std::vector<int> outer_semi_conduct_end; outer_semi_conduct_end.clear(); //�����������ҳ���뵼���յ�
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
	//	cout << "��뵼����ɴ��յ�������⣡" << endl;
	//	return 0;
	//}

	/*^^^^^^^^^^^^^^^^^^^^^^^�ڰ뵼������о����^^^^^^^^^^^^^^^^^^^^^^^^^*/
	//�ڸ���������ڣ�����������Բ�İ뾶��������
	std::vector<IndexofCircle> fore_compare_back_3;
	fore_compare_back_3.clear();

	std::vector<int> end_3_all_index;	end_3_all_index.clear();//20-60

	for (int i = 0; i < m_indexandcircles.size() - 1; i++)  //����������ƾ���
	{

		if ((cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z > 25) &&
			(cable_z_max_value - m_indexandcircles.at(i).once_circle.m_center_z < 40))
		{
			end_3_all_index.push_back(i);  //������������Բ�������洢����
		}

	}

	if (!(end_3_all_index.size()))
	{
		cout << "�ڰ뵼������о�������ȡ�쳣��" << endl;
	}
	std::vector<IndexofCircle>::const_iterator first_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(0);
	std::vector<IndexofCircle>::const_iterator second_fore_2_back_3 = m_indexandcircles.begin() + end_3_all_index.at(end_3_all_index.size() - 1);
	fore_compare_back_3.assign(first_fore_2_back_3, second_fore_2_back_3);

	//double adjacent_radius_distance = fore_compare_back_3.at(0).once_circle.m_radius - fore_compare_back_3.at(1).once_circle.m_radius; 
	//int flag_back_3 = 0;
	//for (int i = 0; i < fore_compare_back_3.size() - 1; i++)  //�ҳ�������Χ�ڰ뾶��ֵ������ĵط�
	//{
	//	//cout << fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius << " " << i <<endl;
	//	if (fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius > adjacent_radius_distance)
	//	{
	//		adjacent_radius_distance = fore_compare_back_3.at(i).once_circle.m_radius - fore_compare_back_3.at(i + 1).once_circle.m_radius;
	//		flag_back_3 = i;
	//	}
	//}
	//int thrid_end = flag_back_3 + m_indexandcircles.size() - end_3.size();  //����������ڰ뵼������о����
	//double thrid_end_disance = m_indexandcircles.at(thrid_end).once_circle.m_center_z - cable_z_mini_value; //�������µ׶˾���
	//cout << "--�ڰ뵼������о�����������µ׶˾��� = " << thrid_end_disance << " mm" << endl;

	double adjacent_radius_distance = 0;
	int flag_back_3 = 0;
	for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1) - 2; i++)  //�ҳ�������Χ�ڰ뾶��ֵ������ĵط�
	{

		if ((m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius) > adjacent_radius_distance)
		{
			adjacent_radius_distance = m_indexandcircles.at(i).once_circle.m_radius - m_indexandcircles.at(i + 2).once_circle.m_radius;
			flag_back_3 = i + 1;
		}
	}
	int inner_semiconducting_end_index = flag_back_3;  //����������ڰ뵼������о������������ƫ��о��
	double inner_semiconducting_end_z_value = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;   //�ڰ뵼������о�Ľ���


																															 /*���ݺ���ҳ��ڰ뵼�����*/
	double cable_core_radius_average = 21;   //�趨��о����뾶Ϊ21mm
	std::vector<int> inner_semiconducting_index;    //�洢���ݺ���ҳ������ڰ뵼��Բ������
	inner_semiconducting_index.clear();

	//for (int i = end_3_all_index.at(0); i<end_3_all_index.at(end_3_all_index.size() - 1); i++)
	//{
	//	if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
	//		&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 1.95)    //����ֵ�����ڰ뵼������λ��
	//		&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //������о��ɢ���³���
	//	{
	//		inner_semiconducting_index.push_back(i);
	//	}
	//}
	for (int i = end_3_all_index.at(0); i < end_3_all_index.at(end_3_all_index.size() - 1); i++)
	{
		if ((m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average > 1.42)
			&& (m_indexandcircles.at(i).once_circle.m_radius - cable_core_radius_average < 2.2)    //����ֵ�����ڰ뵼������λ��
			&& (m_indexandcircles.at(i).once_circle.m_center_z < m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z))  //������о��ɢ���³���
		{
			inner_semiconducting_index.push_back(i);
		}
	}

	double inner_semiconducting_start_z_value_finally = 0;
	if (inner_semiconducting_index.size())   //����д�ĥ�ڰ뵼�磬����ζ�Ÿ�������Ϊ��
	{
		sort(inner_semiconducting_index.begin(), inner_semiconducting_index.end());   //���ҳ������ڰ뵼��Բ�������������������i��С���Ǹ��ط�
		int inner_semiconducting_start_i = inner_semiconducting_index.at(0);  //�ڰ뵼�����Բ������,�ҳ��ڰ뵼������о�Ľ���������ڰ뵼�����ҳ�����
		double inner_semiconducting_start_z_value = m_indexandcircles.at(inner_semiconducting_index.at(0)).once_circle.m_center_z;
		inner_semiconducting_start_z_value_finally = inner_semiconducting_start_z_value;
	}
	else   //���û�д�ĥ�ڰ뵼�磬���ڰ뵼�����������о���ڰ嵼��Ľ���㣬Ҳ�����ڰ뵼����Ϊ0
	{
		inner_semiconducting_start_z_value_finally = m_indexandcircles.at(inner_semiconducting_end_index).once_circle.m_center_z;
		std::cout << "�ڰ뵼���������о����غϣ�" << std::endl;
	}

	/*���·ָ����洢*/
	region_start.push_back(first_start_z_value);  //��뵼����ɴ�����zֵ
	region_start.push_back(second_start_z_value);  //��Ӧ��׶����zֵ
	region_start.push_back(inner_semiconducting_start_z_value_finally);  //�ڰ뵼�����㣬Ҳ�ǵڶ���Ǧ��ͷ�յ��zֵ
	region_start.push_back(inner_semiconducting_end_z_value);  //��о����zֵ��Ҳ���ڰ뵼���յ�zֵ
															   /*���·ָ��յ�洢*/
	region_end.push_back(first_end_z_value);  //��뵼����ɴ��յ��zֵ,����һ��
	region_end.push_back(inner_semiconducting_start_z_value_finally);  //��Ӧ��׶�յ��zֵ
	region_end.push_back(inner_semiconducting_end_z_value);  //�ڰ뵼���յ��zֵ
	region_end.push_back(cable_z_max_value);  //��о�յ��zֵ	

											  /*�Էָ����ֶ���ɫ��ʾ*/
	for (int n = 0; n < pointcloud_filtered->size(); n++)
	{
		if (pointcloud_filtered->at(n).z < region_start.at(0))  //��뵼��
		{
			pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(0) && pointcloud_filtered->at(n).z < region_end.at(0))  //��뵼����ɴ�
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 255;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_end.at(0) && pointcloud_filtered->at(n).z < region_start.at(1))  //����Ե����
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(1) && pointcloud_filtered->at(n).z < region_end.at(1))  //��Ӧ��׶
		{
			pointcloud_filtered->at(n).r = 255;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 255;
		}
		else if (pointcloud_filtered->at(n).z > region_end.at(1) && pointcloud_filtered->at(n).z < region_start.at(2))  //�ڰ뵼��
		{
			pointcloud_filtered->at(n).r = 0;			pointcloud_filtered->at(n).g = 0;			pointcloud_filtered->at(n).b = 0;
		}
		else if (pointcloud_filtered->at(n).z > region_start.at(3) && pointcloud_filtered->at(n).z < region_end.at(3))  //��о
		{
			pointcloud_filtered->at(n).r = 198;			pointcloud_filtered->at(n).g = 145;			pointcloud_filtered->at(n).b = 69;
		}
	}

	//showpointcloud_xyzrgb(pointcloud_filtered);		//�ֶ���ʾ

	std::vector<int> inner_semiconducting_circle_index; inner_semiconducting_circle_index.clear();
	std::vector<int> XLPE_insulation_circle_index; XLPE_insulation_circle_index.clear();
	std::vector<int> conduct_circle_index; conduct_circle_index.clear();
	std::vector<int> outer_semiconducting_circle_index; outer_semiconducting_circle_index.clear();

	for (int i = 0; i<m_indexandcircles.size(); i++)
	{
		if (region_start.at(2)< m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(3))  //�ڰ뵼��Բ�����ó���
		{
			inner_semiconducting_circle_index.push_back(i);
		}

		if (region_end.at(0) < m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(1))  //XLPE��Ե��Բ�����ó���
		{
			XLPE_insulation_circle_index.push_back(i);
		}
		if (region_start.at(3) < m_indexandcircles.at(i).once_circle.m_center_z &&
			m_indexandcircles.at(i).once_circle.m_center_z <  region_end.at(3))  //�����Բ�����ó���
		{
			conduct_circle_index.push_back(i);
		}
		if (m_indexandcircles.at(i).once_circle.m_center_z < region_start.at(0))  //�����Բ�����ó���
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
		inner_semiconducting_circle_radius.push_back(m_indexandcircles.at(inner_semiconducting_circle_index.at(i)).once_circle.m_radius);  ////�ڰ뵼��Բ�뾶�ó���
	}
	sort(inner_semiconducting_circle_radius.begin(), inner_semiconducting_circle_radius.end());

	for (int i = 0; i < XLPE_insulation_circle_index.size(); i++)
	{
		XLPE_insulation_circle_radius.push_back(m_indexandcircles.at(XLPE_insulation_circle_index.at(i)).once_circle.m_radius);		//XLPE��Ե��ԲԲ�뾶�ó���
	}
	sort(XLPE_insulation_circle_radius.begin(), XLPE_insulation_circle_radius.end());

	for (int i = 0; i < conduct_circle_index.size(); i++)
	{
		conduct_circle_radius.push_back(m_indexandcircles.at(conduct_circle_index.at(i)).once_circle.m_radius);		//�����Բ�뾶�ó���
	}
	sort(conduct_circle_radius.begin(), conduct_circle_radius.end());

	for (int i = 0; i < outer_semiconducting_circle_index.size(); i++)
	{
		outer_semiconducting_circle_radius.push_back(m_indexandcircles.at(outer_semiconducting_circle_index.at(i)).once_circle.m_radius);		//��뵼���Բ�뾶�ó���
	}
	sort(outer_semiconducting_circle_radius.begin(), outer_semiconducting_circle_radius.end());


	/*���㷴Ӧ��׶б�³���*/
	double xiepo = pow((2 * (XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))) - (2 * (inner_semiconducting_circle_radius.at(0))), 2) +
		pow((region_end.at(1) - region_start.at(1)), 2);
	double Length_S = sqrt(xiepo);
	//cout << "\n" << endl;	
	//cout << "��Ӧ��׶�����յ��⾶:Di:" <<2*(inner_semiconducting_circle_radius.at(0)) << endl;
	//cout << "��Ӧ��׶��������Ե�⾶:Do:" << 2*(XLPE_insulation_circle_radius.at(XLPE_insulation_circle_radius.size() - 1))<< endl;
	//cout << "��Ӧ��׶б�³��ȣ�S:" << Length_S << endl;

	para.inner_semiconducting_circle_radius = inner_semiconducting_circle_radius;
	para.XLPE_insulation_circle_radius = XLPE_insulation_circle_radius;
	para.outer_semiconducting_circle_radius = outer_semiconducting_circle_radius;
	para.conduct_circle_radius = conduct_circle_radius;
	para.cable_z_max_value = cable_z_max_value;

	cout << "1.��뵼����ɴ�����߶ȣ�" << region_end.at(0) - region_start.at(0) << " mm" << endl;
	cout << "2.XLPE����Ե����߶ȣ�" << region_start.at(1) - region_end.at(0) << " mm" << endl;
	cout << "3.��Ӧ��׶����߶ȣ�" << region_end.at(1) - region_start.at(1) << " mm" << endl;
	cout << "4.�ڰ뵼�������߶ȣ�" << region_start.at(3) - region_start.at(2) << endl;
	cout << "5.�������򳤶ȣ�" << region_end.at(3) - region_start.at(3) << " mm" << endl;
	cout << "6.��ͷ��������߶ȣ�" << cable_z_max_value - region_end.at(0) << " mm" << endl;
	cout << "7.��뵼�������⾶��" << 2 * outer_semiconducting_circle_radius.at((int)(outer_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
	cout << "8.��������Ե�����⾶��" << 2 * XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2)) << " mm" << endl;
	cout << "9.�ڰ뵼��������⾶��" << 2 * inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2)) << " mm" << endl;
	cout << "10.���������⾶��" << 2 * conduct_circle_radius.at((int)(conduct_circle_radius.size() / 2)) << " ��" << endl;
	cout << "11.��뵼����ɴ���б�Ƕȣ�" << (atan2((outer_semiconducting_circle_radius.at((int)(outer_semiconducting_circle_radius.size() / 2)) -
		XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2))), region_end.at(0) - region_start.at(0))) * 180 / M_PI << " mm" << endl;
	cout << "12.��Ӧ��׶��б�Ƕȣ�" << (atan2((XLPE_insulation_circle_radius.at((int)(XLPE_insulation_circle_radius.size() / 2))
		- inner_semiconducting_circle_radius.at((int)(inner_semiconducting_circle_radius.size() / 2))), region_end.at(1) - region_start.at(1))) * 180 / M_PI << " ��" << endl;

	std::cout << "��ͷ��������ָ�����������������!" << endl;


	return true;

	/*��������Ƭ������СƬ���߼н�*/

	/*ȡ���ֲ�����*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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

	/*���㷨�ߣ����߹���*/
	//regionSegmentation(pointcloud_filtered_local);   //�Բ��ֵ��ƽ��г���
	//pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	//pcl::PointCloud <pcl::Normal>::Ptr normals_resit(new pcl::PointCloud <pcl::Normal>);
	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;			   //�������߹��ƶ���
	//normal_estimator.setSearchMethod(tree);				//������������
	//normal_estimator.setInputCloud(m_index_strip.at(0).each_strip);				//���÷��߹��ƶ�������㼯
	//normal_estimator.setKSearch(8);						// �������ڷ��������Ƶ�k������Ŀ  ���������ڶ��ٵ���ģ��ƽ����㷨��
	//normal_estimator.compute(*normals);					//���㲢���������
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);
	//show_normals(m_index_strip.at(0).each_strip, normals);
	//for (int i =0;i<normals->size();i++)
	//{
	//	normals->at(i).normal_x = (normals->at(i).normal_x) * (-1);
	//	normals->at(i).normal_y = (normals->at(i).normal_y) * (-1);
	//	normals->at(i).normal_z = (normals->at(i).normal_z) * (-1);
	//	normals_resit->push_back(normals->at(i));
	//}
	//show_normals(m_index_strip.at(0).each_strip, normals_resit);  //���߷����ض���
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);

	/*���Բ��ɫ�ֲ������ó���ʾ*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local1(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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

	/***************************************************************ƽ���������***************************************************************************************************/
	/*���������ȡ�������Ͼֲ�����Ȼ����Ͽռ�ƽ������*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local(new pcl::PointCloud<pcl::PointXYZRGB>);    //ȡ�����򽻽�����ҵľֲ�����
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
	/*������Ƭ�ľֲ�������*/
	showpointcloud_xyzrgb(pointcloud_filtered_local);

	regionSegmentation(pointcloud_filtered_local);   //�Բ��ֵ��ƽ��г���
													 /*����������ӻ�*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_strip_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < m_index_strip.size(); i++)
	{
		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
		{
			pointcloud_filtered_local_strip_clor->push_back(m_index_strip.at(i).each_strip->at(j));
		}
	}

	/*������ͬ��ɫ��ʾ*/
	for (int i = 0; i < m_index_strip.at(0).each_strip->size(); i++)
	{
		m_index_strip.at(0).each_strip->at(i).r = 255;
		m_index_strip.at(0).each_strip->at(i).g = 255;
		m_index_strip.at(0).each_strip->at(i).b = 255;
	}
	/*��ʾĳһ��*/
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);

	rankpointcloud_z(m_index_strip.at(0).each_strip);
	/*��һ���ϲ�ͬ����ľֲ����ϲ�ͬ��ɫ*/
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
	//showpointcloud_xyzrgb(m_index_strip.at(0).each_strip);   //һ���ϲ�ͬ������ʾ��ͬ��ɫ
	showpointcloud_xyzrgb(pointcloud_filtered_local_strip_clor);	//�ֲ����Ƹ����Բ�ͬ��ɫ��ʾ��չʾ����Ч��

	Strip_2_slice(pointcloud_filtered_local);	//����Ƭ
												/*��Ƭ������ӻ�*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered_local_slice_clor(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < m_index_strip.size(); i++)
	{
		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)
		{
			pointcloud_filtered_local_slice_clor->push_back(m_index_strip.at(i).each_strip->at(j));
		}
	}
	/*��ʾĳһƬ*/
	for (int i = 0; i<m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->size(); i++)
	{
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).r = 255;
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).g = 0;
		m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice->at(i).b = 0;

	}
	//showpointcloud_xyzrgb(m_cable_strip_slice_all.at(9).at(9).pointcloud_strip_slice);
	/*��Ƭ��ͬ��ɫ��ʾ*/
	showpointcloud_xyzrgb(pointcloud_filtered_local_slice_clor);   //����Ԫ��ͬ��ɫ��ʾ����չʾ��Ƭ����Ч��

																   /*ĳ���Ͽ�Ԫ����д��*/
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
																   //cout << "��Ԫ����������!" << endl;

																   //regionSegmentation(pointcloud_filtered);   //�Բ��ֵ��ƽ��г���
																   //Strip_2_slice(pointcloud_filtered);	//����Ƭ

																   /*ƽ���������*/
	for (int i = 0; i<m_index_strip.size(); i++)	//����ÿһ��
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ�����
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ����ϣ���������ƽ�潻�ߣ�
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr piece_local_point_cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);  //ȡ����ǰС��һ���ֳ�����ƽ�����

		for (int j = 0; j < m_index_strip.at(i).each_strip->size(); j++)  //�ڵ�ǰ��������������������Ƭ״�ֲ����Ƴ�����Ȼ����ƽ���������
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
		for (int m = 0; m<piece_local_point_cloud_1->size(); m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
		{
			pointcloud_filtered_local->push_back(piece_local_point_cloud_1->at(m));
			m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_1->at(m));
		}
		for (int m = 0; m < piece_local_point_cloud_2->size(); m++)  //���ҳ�����СƬ������ɫ���ٷŻ����У��۲�λ��
		{
			pointcloud_filtered_local->push_back(piece_local_point_cloud_2->at(m));
			m_index_strip.at(i).each_strip->push_back(piece_local_point_cloud_2->at(m));
		}
		//showpointcloud_xyzrgb(pointcloud_filtered_local);  //�������ֲ���������ʾ���ƽ��ĵ���
		//showpointcloud_xyzrgb(m_index_strip.at(i).each_strip);	//��ʾĳһ����һ����Ƭ��ɣ�Ƭ����ɫ��ͬ
		//showpointcloud_xyzrgb(piece_local_point_cloud_1);	//����ʾ���ƽ��ľֲ�����1
		//showpointcloud_xyzrgb(piece_local_point_cloud_2);	//����ʾ���ƽ��ľֲ�����2

		float plane1_A, plane1_B, plane1_C, plane1_D; plane1_A = plane1_B = plane1_C = plane1_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_1, plane1_A, plane1_B, plane1_C, plane1_D);  //XLPE�����ھֲ����ƽ�����
		float plane2_A, plane2_B, plane2_C, plane2_D; plane2_A = plane2_B = plane2_C = plane2_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_2, plane2_A, plane2_B, plane2_C, plane2_D);  //��Ӧ��׶�����ھֲ����ƽ�����
		float plane3_A, plane3_B, plane3_C, plane3_D; plane3_A = plane3_B = plane3_C = plane3_D = 0;
		ransac_plane_estimation(piece_local_point_cloud_3, plane3_A, plane3_B, plane3_C, plane3_D);  //�ڰ뵼�������ھֲ����ƽ�����


		Eigen::Vector4f plane_a, plane_b, plane_c;       //������ƽ�潻��
		Eigen::VectorXf line, line1;
		plane_a << plane1_A, plane1_B, plane1_C, plane1_D;
		plane_b << plane2_A, plane2_B, plane2_C, plane2_D;
		plane_c << plane3_A, plane3_B, plane3_C, plane3_D;
		pcl::planeWithPlaneIntersection(plane_a, plane_b, line, 1e-6);
		pcl::planeWithPlaneIntersection(plane_b, plane_c, line1, 1e-6);

		std::cout << "XLPE�뷴Ӧ��׶�ֱཻ��Ϊ��\n" << line << endl;   //(line[0],line[1],line[2])Ϊ����ֱ������һ�㣬��line[3],line[4],line[5]��Ϊֱ�ߵķ�����
		std::cout << "��Ӧ��׶���ڰ뵼���ֱཻ��Ϊ��\n" << line1 << endl;
		//cout << "��֤����������ƽ���ϣ� " << plane1_A*line[0] + plane1_B*line[1] + plane1_C*line[2] + plane1_D << endl;

		/*ƽ�漰ƽ�潻�߿��ӻ�*/
		//	////���ӻ�
		//	//pcl::visualization::PCLVisualizer viewer;
		//	////���ӻ�ƽ��
		//	//pcl::ModelCoefficients plane, plane1;
		//	//for (size_t k = 0; k < 4; ++k)
		//	//{
		//	//	plane.values.push_back(plane_a[k]);
		//	//	plane1.values.push_back(plane_b[k]);
		//	//}
		//	//viewer.addPlane(plane, "plane", 0);
		//	//viewer.addPlane(plane1, "plane1", 0);
		//	////���ӻ�ƽ�潻��
		//	//pcl::PointXYZ p1, p2, p3, p4;
		//	//p1.x = line[0]; p1.y = line[1]; p1.z = line[2];
		//	//p2.x = line[3]; p2.y = line[4]; p2.z = line[5];
		//	//viewer.addLine(p1, p2, 1, 0, 0, "line", 0);
		//	//while (!viewer.wasStopped())
		//	//{
		//	//	viewer.spinOnce(100);
		//	//}
		//	

		/*����ռ�һ�㵽ֱ�ߵľ���*/
		Eigen::Vector4f point, point1;
		//point << line[0], line[1], line[2], 0;     //ֱ����һ��
		point << line[0], line[1], line[2];     //ֱ����һ���룬x,y,z
		point1 << line1[0], line1[1], line1[2];     //ֱ����һ���룬x,y,z

		Eigen::Vector4f normal, normal1;
		normal << line[3], line[4], line[5];		//ֱ�ߵķ�������
		normal1 << line1[3], line1[4], line1[5];		//ֱ�ߵķ�������

		std::vector<float> point_2_line_distance; point_2_line_distance.clear();
		std::vector<float> point_2_line1_distance; point_2_line1_distance.clear();

		for (int k = 0; k < m_index_strip.at(i).each_strip->size(); k++)
		{
			float distance = 0;
			float distance1 = 0;
			Eigen::Vector4f point_cloud_point;
			point_cloud_point << m_index_strip.at(i).each_strip->at(k).x, m_index_strip.at(i).each_strip->at(k).y, m_index_strip.at(i).each_strip->at(k).z;
			distance = pcl::sqrPointToLineDistance(point_cloud_point, point, normal);// �ռ�㵽�ռ�ֱ�߾��룬�����ֱ�Ϊ������һ�㣬ֱ����һ�㣬ֱ�ߵķ�����
			distance1 = pcl::sqrPointToLineDistance(point_cloud_point, point1, normal1);
			point_2_line_distance.push_back(distance);
			point_2_line1_distance.push_back(distance1);

		}
		float small_distance = point_2_line_distance.at(0);
		int index_of_small_distance = 0;
		for (int k = 0; k<point_2_line_distance.size(); k++)//�ҳ���ǰ������һ����������ֱ�����
		{
			if (point_2_line_distance.at(k) < small_distance)
			{
				small_distance = point_2_line_distance.at(k);
				index_of_small_distance = k;
			}
		}

		float small_distance1 = point_2_line1_distance.at(0);
		int index_of_small_distance1 = 0;
		for (int k = 0; k < point_2_line1_distance.size(); k++)//�ҳ���ǰ������һ����������ֱ�����
		{
			if (point_2_line1_distance.at(k) < small_distance1)
			{
				small_distance1 = point_2_line1_distance.at(k);
				index_of_small_distance1 = k;
			}
		}

		std::cout << "У�����С������95mm��λzֵ��" << m_index_strip.at(i).each_strip->at(index_of_small_distance).z << std::endl;
		std::cout << "��Ӧ��׶У��ǰ����ֵ = " << cable_z_max_value - line[2] << endl;
		std::cout << "��Ӧ��׶У�������ֵ = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance).z << endl;
		std::cout << "У�����С������40mm��λzֵ��" << m_index_strip.at(i).each_strip->at(index_of_small_distance1).z << std::endl;
		std::cout << "�ڰ뵼�����У��ǰ����ֵ = " << cable_z_max_value - line1[2] << endl;
		std::cout << "�ڰ뵼�����У�������ֵ = " << cable_z_max_value - m_index_strip.at(i).each_strip->at(index_of_small_distance1).z << endl;
	}
	/***************************************************************ƽ���������***************************************************************************************************/

	/*ȡһ��������ÿСƬ����ƽ����ƣ�Ȼ��������߼н�*/
	pcl::Normal axis_normal;
	axis_normal.normal_x = 0;  //ָ������Ϊz��
	axis_normal.normal_y = 0;
	axis_normal.normal_z = 1;
	std::vector<float> one_strip_slice_angle; one_strip_slice_angle.clear();
	for (int i = 0; i < m_cable_strip_slice_all.at(0).size(); i++)  //��ĳһ���е�СƬ���߼нǽ��м���
	{
		float A, B, C, D; A = B = C = D = 0;
		ransac_plane_estimation(m_cable_strip_slice_all.at(0).at(i).pointcloud_strip_slice, A, B, C, D);  //��СƬ����ƽ����ƣ��Ӷ��ó�ƽ�淨��
		pcl::Normal once_slice_normal;
		once_slice_normal.normal_x = A; once_slice_normal.normal_y = B; once_slice_normal.normal_z = C;
		//double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z);  //����н�����ֵ
		double cos_theta = (axis_normal.normal_x)*(once_slice_normal.normal_x) + (axis_normal.normal_y)*(once_slice_normal.normal_y) + (axis_normal.normal_z)*(once_slice_normal.normal_z) /
			sqrt(pow(once_slice_normal.normal_x, 2) + pow(once_slice_normal.normal_y, 2) + pow(once_slice_normal.normal_x, 2));  //����н�����ֵ
		double inverse_cosine_value = acos(cos_theta) * 180 / M_PI;  //�нǷ����ң�Ҳ����������ڷ���֮��ļн�
		one_strip_slice_angle.push_back(180 - inverse_cosine_value);  //�������߼н�
	}

	/*СƬ���߼н�����д��*/
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
	//cout << "СƬ���߼н�������!" << endl;



	return true;
}
/*�ַָ�����õ������������յ�󣬽��յ�ǰ��k/2��Բ�ó����ںϣ�Ȼ���Ը�Сzֵ�������Բ*/
//circle_number�����򽻽ӵ�ı�� ��
//��һ����׼��ָ�ʹ�õ�ǰ��Բ�ĸ���field
//flag  1�� ���µ�circle_number 2���������� 3����circle_number����
bool Cable::fusion_iteration_detail_segmentation(int circle_number, int field, int flag, std::vector<Circle> &region_circles)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_circle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  //��������һ�α�׼��֮���ҳ��ֽ��ǰ���󣩶��ٸ�Բ�ĵ�
	region_circle_cloud->clear();

	if (flag == 1)		//ȡ�ַָ���㼰���ǰ���Բ�������ڰ뵼�������о���磩
	{
		for (int i = field; i >= 0; i--)    //�����ٽ�field��Բ
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number - i).each_circle_index.size(); j++) //�ٽ�ǰiĳ��Բ�ڵĵ�
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number - i).each_circle_points->at(j));   //��ָ�����ٽ�Բ�ڵ�ÿ������д洢
			}
		}
	}


	if (flag == 2)   //ȡ�ַָ�ǰ���Բ����
	{
		for (int i = field; i >= 0; i--)     //ȡcircle_numberǰ���Բ
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number - i).each_circle_index.size(); j++)  //�ٽ�ǰiĳ��Բ�ڵĵ�
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number - i).each_circle_points->at(j));   //��ָ�����ٽ�Բ�ڵ�ÿ������д洢
			}
		}

		for (int i = 1; i <= field; i++)    //ȡcircle_number�����Բ
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number + i).each_circle_index.size(); j++)  //�ٽ�ĳ��Բ�ڵĵ�
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number + i).each_circle_points->at(j));   //��ָ�����ٽ�Բ�ڵ�ÿ������д洢
			}
		}
	}

	if (flag == 3)		//ȡ�ַָ�����Բ����
	{
		for (int i = 1; i <= field; i++)    //ȡcircle_number�����Բ
		{
			for (int j = 0; j < m_indexandcircles.at(circle_number + i).each_circle_index.size(); j++)  //�ٽ�ĳ��Բ�ڵĵ�
			{
				region_circle_cloud->push_back(m_indexandcircles.at(circle_number + i).each_circle_points->at(j));   //��ָ�����ٽ�Բ�ڵ�ÿ������д洢
			}
		}
	}


	//�����»�ȡ���ĵ��¾ֲ����ƽ���Բ���
	if (region_circle_cloud->empty())			//�ж�����ĵ����Ƿ�Ϊ��
	{
		std::cout << "region point cloud is empty!" << std::endl;
		return false;
	}

	rankpointcloud_z(region_circle_cloud);		//�����������
	pcl::PointXYZRGB srart_point;
	pcl::PointXYZRGB update_point;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_circle_point(new pcl::PointCloud<pcl::PointXYZRGB>);
	srart_point = region_circle_cloud->at(0);
	double region_point_z_min = region_circle_cloud->at(0).z;		//�����ļ������ĵ�z����͵�����ļ���zֵ
	double region_point_z_max = region_circle_cloud->at(region_circle_cloud->size() - 1).z;		//�������Ƶ���ߵ��zֵ
	region_circles.clear();

	for (double circle_height = region_point_z_min; circle_height < region_point_z_max; circle_height += 0.2)   //����zֵ��������С����ÿk��zֵ����һ��Բ
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
				region_circle_point->push_back(region_circle_cloud->at(i));//�洢��Χ�ڵĵ���
			}
		}
		if (region_circle_point->size())
		{
			Circle circle_temp;
			ransice_circular_fitting(region_circle_point, circle_temp);  //ransicԲ���
			Circle once_circle;
			once_circle.m_center_x = circle_temp.m_center_x;
			once_circle.m_center_y = circle_temp.m_center_y;
			once_circle.m_center_z = circle_temp.m_center_z;
			once_circle.m_radius = circle_temp.m_radius;
			region_circles.push_back(once_circle);  //��ÿһ�����Բ������
		}
	}
	return true;
}
/*��㾫�ָ�zֵ��ȡ back/fore*/
bool Cable::update_segmentation_start(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value)
{
	std::vector<double> start_back_compare_fore_all;
	double standard_fore_start, standard_back_start;
	double standard_back_compare_fore_start;
	start_back_compare_fore_all.clear();
	for (int i = 0; i < k; i++)  //Ϊ�������Բ������ȣ�ǰ����k��0
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

	for (int i = 0; i < k; i++)	//Ϊ�������Բ������ȣ�������k��0
	{
		start_back_compare_fore_all.push_back(0);
	}

	double max_variance = start_back_compare_fore_all.at(0);   //�ҳ�����׼��
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
/*�㾫�ָ�zֵ��ȡ  fore/back*/
bool Cable::update_segmentation_end(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value)  //k�ǵ�һ�ηָ���б�׼��֮��ʱʹ�õ�ǰ��Բ����
{
	std::vector<double> fore_compare_back_region_end;
	double Variance_fore_update, Variance_back_update;
	double  Variance_fore_compare_back_region_end_update;
	fore_compare_back_region_end.clear();
	for (int i = 0; i < k; i++)  //Ϊ�������Բ������ȣ�ǰ����k��0
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
	for (int i = 0; i < k; i++)	//Ϊ�������Բ������ȣ�������k��0
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
/*�Ե��¸����ֽ��гߴ����*/
/*���·ָ����洢*/
//region_start.push_back(first_start_z_value);  //��뵼����ɴ�����������zֵ
//region_start.push_back(second_start_update + cable_z_mini_value);  //��һ��Ǧ��ͷ����zֵ
//region_start.push_back(thrid_start_update + cable_z_mini_value);  //�ڶ���Ǧ��ͷ����zֵ
//region_start.push_back(inner_semiconducting_start_z_value_finally);  //�ڰ뵼�����㣬Ҳ�ǵڶ���Ǧ��ͷ�յ��zֵ
//region_start.push_back(third_end_disance + cable_z_mini_value);  //��о����zֵ��Ҳ���ڰ뵼���յ�zֵ
/*���·ָ��յ�洢*/
//region_end.push_back(first_end_update);  //��뵼����ɴ��յ��zֵ,����һ��
//region_end.push_back(second_end_update + cable_z_mini_value);  //��һ��Ǧ��ͷ�յ����������͵��zֵ
//region_end.push_back(inner_semiconducting_start_z_value_finally);  //�ڶ���Ǧ��ͷ�յ��zֵ
//region_end.push_back(third_end_disance + cable_z_mini_value);  //�ڰ뵼���յ��zֵ
//region_end.push_back(cable_z_max_value);  //��о�յ��zֵ	
Parameter_measurement Cable::segmentation_region_size_measurement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<double> region_start, std::vector<double> region_end)
{
	std::cout << "���ڸ��ݷָ����Ե��²������в�����..." << endl;
	double cable_z_max_value = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z; //�����һ�����Բ��zֵ��Ϊ���µ���ߵ�

	Parameter_measurement cable_region_sizz_measurement;
	cable_region_sizz_measurement.conductor_size = region_end.at(region_end.size() - 1) - region_start.at(region_start.size() - 1);		//1.��о����ĳ��ȣ���׼Ϊ30mm
	cable_region_sizz_measurement.core_pencil_head_size = region_end.at(region_end.size() - 1) - region_start.at(2);					//2.������оǦ��ͷ�ĳ��ȣ���׼Ϊ95mm
	cable_region_sizz_measurement.inner_semiconducting_size = region_end.at(3) - region_start.at(3);									//3.�ڰ뵼���Ŀ�ȣ���׼Ϊ10mm
	cable_region_sizz_measurement.outside_pencil_head_size = region_end.at(1) - region_start.at(1);										//4.Զ����о��Ǧ��ͷ������µĳߴ磬��׼Ϊ30mm
	cable_region_sizz_measurement.core_top_2_outside_pencil_head_distance = region_end.at(region_end.size() - 1) - region_start.at(1);	//5.Զ����оǦ��ͷ��������о���˵ľ��룬��׼Ϊ300mm
	cable_region_sizz_measurement.core_outside_semiconducting_end = region_end.at(region_end.size() - 1) - region_end.at(0);			//6.��뵼����ɴ��յ������о����600mm
	cable_region_sizz_measurement.outside_semiconducting_size = region_end.at(0) - region_start.at(0);									//7.��뵼����ɴ��ĳߴ�,��׼Ϊ40mm

																																		/*����ԣ�8.����Ե��Ե�⾶ƫ�9.��оĩ�˲���18cmλ�ô��ľ�Ե�⾶���в�����10.������ɢ����*/

																																		//cable_region_sizz_measurement.main_insulation_deviation = main_insulation_difference; //8.��뵼����ɴ��յ����ȡ4��Բ���������ֵ����Сֵ֮��İ뾶��
	std::vector<int> cable_core_region_radius;   //�洢��о����Բ������
	cable_core_region_radius.clear();
	std::vector<int> region_circle_index_15_2_22; //�洢������о����16cm��20cmԲ������
	region_circle_index_15_2_22.clear();
	double thin_insulation_circle;				 //�洢������о����18cm���İ뾶
	std::vector<int> main_insulation_circle_index; //�洢����Ե��Բ������350-580
	main_insulation_circle_index.clear();
	for (int i = 0; i < m_indexandcircles.size(); i++)
	{
		//�ҳ�������о����15cm-22��Բ������
		if (region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z > 150 && region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z < 220)
		{
			region_circle_index_15_2_22.push_back(i);
		}
		//�ҳ���о����Բ������
		if (m_indexandcircles.at(i).once_circle.m_center_z <= region_end.at(region_end.size() - 1) && m_indexandcircles.at(i).once_circle.m_center_z > region_start.at(region_start.size() - 1))
		{
			cable_core_region_radius.push_back(i);
		}
		//�ҳ���������Ե������о����350-580����Բ������
		if (region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z > 350 && region_end.at(region_end.size() - 1) - m_indexandcircles.at(i).once_circle.m_center_z < 580)
		{
			main_insulation_circle_index.push_back(i);
		}

	}

	/*****************************************8.���������о����35cm-58cm�����µľ�Ե�⾶ƫ��**************************************************/
	//std::vector<double> insulation_outer_diameter_deviation;
	//insulation_outer_diameter_deviation.clear();
	//std::vector<double> once_circle_deviation;
	//double max_outer_diameter_difference;
	//for (int i = 0; i < main_insulation_circle_index.size(); i += 10)  //����ÿһ��Բ�ľ�Ե�⾶��жϣ���10���ж�һ��
	//{
	//	max_outer_diameter_difference = 0;
	//	once_circle_deviation.clear();
	//	std::vector<Index_Strip> pointcloud_2_strip_all_main_insulation_one_circle;
	//	pointcloud_2_strip(m_indexandcircles.at(main_insulation_circle_index.at(i)).each_circle_points, pointcloud_2_strip_all_main_insulation_one_circle,90);  //��45�Ȱ�Բ���4���֣��ֱ���ϳ�Բ��ͨ���Ƚϰ뾶�Ĳ�����ҳ�����Բ��
	//	for (int j = 0;j<pointcloud_2_strip_all_main_insulation_one_circle.size();j++)
	//	{
	//		Circle circle;
	//		ransic_circle_estimation(pointcloud_2_strip_all_main_insulation_one_circle.at(j).each_strip, circle);
	//		once_circle_deviation.push_back(circle.m_radius);
	//	}
	//	sort(once_circle_deviation.begin(), once_circle_deviation.end());
	//	max_outer_diameter_difference = once_circle_deviation.at(once_circle_deviation.size() - 1) - once_circle_deviation.at(0);  //��4������ϵ�Բ�������뾶����С�뾶����
	//	insulation_outer_diameter_deviation.push_back(max_outer_diameter_difference);  //�Ѳ��ÿһ��Բ�����ƫ�������
	//}
	//sort(insulation_outer_diameter_deviation.begin(), insulation_outer_diameter_deviation.end());
	//cable_region_sizz_measurement.main_insulation_deviation = insulation_outer_diameter_deviation.at(insulation_outer_diameter_deviation.size() - 1 );  //8.����ͬһ����ľ�Ե�⾶���������ֵ����Сֵ֮��İ뾶���һ��Բ�ĵ���4�֣�Ȼ��ֱ���ϣ�,�����10��Բ����һ��

	/*ͨ�������Բ*/
	std::vector<double> insulation_outer_diameter_deviation;//����Եֱ��
	insulation_outer_diameter_deviation.clear();
	std::vector<double> once_circle_deviation;
	double max_outer_diameter_difference;
	for (int i = 0; i < main_insulation_circle_index.size(); i += 10)  //����ÿһ��Բ�ľ�Ե�⾶��жϣ���10���ж�һ��
	{
		std::vector<double> ellipse;
		ellipse = getEllipseparGauss(m_indexandcircles.at(main_insulation_circle_index.at(i)).each_circle_points);
		double  ellipse_a = ellipse.at(2);  //��Բ����
		double ellipse_b = ellipse.at(3);	//��Բ����
		double a_b_dfference = (2 * ellipse_a) - (2 * ellipse_b);

		insulation_outer_diameter_deviation.push_back(a_b_dfference);  //�Ѳ��ÿһ����Բ�����ƫ�������
	}
	sort(insulation_outer_diameter_deviation.begin(), insulation_outer_diameter_deviation.end());
	//8.����ͬһ����ľ�Ե�⾶���������ֵ����Сֵ֮��İ뾶��,�����10��Բ����һ��
	cable_region_sizz_measurement.main_insulation_deviation = insulation_outer_diameter_deviation.at(insulation_outer_diameter_deviation.size() - 1);

	/*****************************************************9.����������о����15cm-22cm��Բ�İ뾶,��׼Ϊ65mm����Ե�⾶���ڰ뾶��2********************************************/
	//�����Ե�⾶��Χ
	std::vector<double> main_insulation_radius_15_2_22;
	main_insulation_radius_15_2_22.clear();
	for (int i = 0; i <region_circle_index_15_2_22.size(); i++)
	{
		main_insulation_radius_15_2_22.push_back(m_indexandcircles.at(region_circle_index_15_2_22.at(i)).once_circle.m_radius);
	}
	sort(main_insulation_radius_15_2_22.begin(), main_insulation_radius_15_2_22.end());
	cable_region_sizz_measurement.insulation_core_size_18_min = main_insulation_radius_15_2_22.at(0) * 2;
	cable_region_sizz_measurement.insulation_core_size_18_max = main_insulation_radius_15_2_22.at(main_insulation_radius_15_2_22.size() - 1) * 2;

	/*****************************************************10.������1��������ɢ����,*********************************************************************************/
	//double standard_parts_core_standard_value = 5.9122980166409684;   //�ñ�׼�����Է���normal����������о�������а뾶�ı�׼�ֻҪ��׼����ڸ�ֵ��˵��������Ѿ�������ɢ����
	//double cable_core_region_radius_value_sum = 0;
	//double cable_core_region_radius_value_average = 0;
	//double variance_deviation_molecule = 0;
	//double variance_deviation = 0;
	//double standard_deviation = 0;
	//for (int i = 0;i < cable_core_region_radius.size();i++)  //������о��������Բ�İ뾶֮��
	//{
	//	cable_core_region_radius_value_sum = cable_core_region_radius_value_sum + m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius;
	//}
	//cable_core_region_radius_value_average = cable_core_region_radius_value_sum / cable_core_region_radius.size() ;   //���������о�������а뾶�͵ľ�ֵ
	//for (int j = 0; j < cable_core_region_radius.size(); j++)  //���㷽��ķ���
	//{
	//	double variance_temp = pow(((m_indexandcircles.at(cable_core_region_radius.at(j)).once_circle.m_radius) - cable_core_region_radius_value_average), 2);
	//	variance_deviation_molecule = variance_deviation_molecule + variance_temp;
	//}
	//variance_deviation = variance_deviation_molecule / cable_core_region_radius.size();  //��о����Բ�뾶�ķ���
	//standard_deviation = sqrt(variance_deviation);   //��о����Բ�뾶�ı�׼��
	//int loose_flag = 0;
	//if (standard_deviation > (standard_parts_core_standard_value ) )  //��ʵ�ʴ�ĥ���µ���о�����Բ�ó����������׼������׼����ڱ�׼����о�����׼�����0.5������������֤����о��������ɢ
	//{
	//	loose_flag = 1;	
	//}
	//cable_region_sizz_measurement.conductor_deformation = loose_flag;   //10.ͨ���Դ�ĥ������о��������Բ�뾶�����׼�Ȼ�����׼���ⲿ��Բ�ı�׼�����Ƚϣ��ж���о�Ƿ�����ɢ���ε����ƣ�0����ɢ��1��ɢ	

	//**************************************************10.������ɢ���� 11.����о������б***********************************************************************************
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_pointcloud->clear();
	std::vector<double>cable_core_circle_radius;  //����о����Բ�İ뾶�ó���
	cable_core_circle_radius.clear();
	std::vector<Circle> cable_core_circle_center;
	cable_core_circle_center.clear();

	for (int i = 0; i < cable_core_region_radius.size(); i++)
	{
		for (int j = 0; j < m_indexandcircles.at(cable_core_region_radius.at(i)).each_circle_points->size(); j++)   //����о��������Բ�ĵ㼯������
		{
			cable_core_pointcloud->push_back(m_indexandcircles.at(cable_core_region_radius.at(i)).each_circle_points->at(j));
		}
		cable_core_circle_radius.push_back(m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius);    //����о��������Բ�İ뾶����������Ϊ�˵ó���о����ľ�ֵ�뾶

		Circle core_circle;   //�洢Բ��
		core_circle.m_center_x = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_x;   //����о��������Բ��Բ�ļ���������Ϊ�������о���������
		core_circle.m_center_y = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_y;
		core_circle.m_center_z = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_center_z;
		core_circle.m_radius = m_indexandcircles.at(cable_core_region_radius.at(i)).once_circle.m_radius;
		cable_core_circle_center.push_back(core_circle);
	}

	sort(cable_core_circle_radius.begin(), cable_core_circle_radius.end());
	double core_circle_radius = 0;
	int remain_radius_count = 0;
	double core_circle_radius_average = 0;//��о����뾶��ֵ
	for (int i = 1; i<cable_core_circle_radius.size() - 1; i++) //ȥ�������ֵ����Сֵ�����ֵ
	{
		remain_radius_count++;
		core_circle_radius = core_circle_radius + cable_core_circle_radius.at(i);
	}
	core_circle_radius_average = core_circle_radius / remain_radius_count;  //��о���뾶ȥ�����ֵ����Сֵ��ľ�ֵ

	double core_circle_x_sum = 0;
	double core_circle_y_sum = 0;
	double core_circle_x_average = 0;//��о����Բ�ľ�ֵx
	double core_circle_y_average = 0;

	for (int i = 0; i < cable_core_circle_center.size(); i++)
	{
		core_circle_x_sum = core_circle_x_sum + cable_core_circle_center.at(i).m_center_x;
		core_circle_y_sum = core_circle_y_sum + cable_core_circle_center.at(i).m_center_y;
	}
	core_circle_x_average = core_circle_x_sum / cable_core_circle_center.size();  //������о������ԲԲ��x,y�ľ�ֵ
	core_circle_y_average = core_circle_y_sum / cable_core_circle_center.size();


	/*************************** 10.������2������о��ɢ*****************************************/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_loose_defect_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_loose_normal_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	core_loose_defect_points->clear(); core_loose_normal_points->clear();
	double point_2_line_distnace_deviation = 0;//��ƫ��ֵ
	int counter = 0;
	for (int i = 0; i < cable_core_pointcloud->size(); i++)   //������о�������е���ƽ��Բ�ģ�x��y��֮��ľ��룬������zֵ
	{
		double once_point_2_center_distnace = sqrt(pow((cable_core_pointcloud->at(i).x - core_circle_x_average), 2) + pow((cable_core_pointcloud->at(i).y - core_circle_y_average), 2));
		//if (once_point_2_center_distnace > core_circle_radius_average)  //����㵽ƽ��Բ�ľ��������оƽ���뾶����ô�ý���ֵ��ȥƽ���뾶������¼ƫ��ֵ
		if (once_point_2_center_distnace - core_circle_radius_average > 1)
		{
			counter++;
			point_2_line_distnace_deviation = point_2_line_distnace_deviation + (once_point_2_center_distnace - core_circle_radius_average);
			cable_core_pointcloud->at(i).r = 0;	cable_core_pointcloud->at(i).g = 255;	cable_core_pointcloud->at(i).b = 0;
			core_loose_defect_points->push_back(cable_core_pointcloud->at(i));	//��о��ɢ���ƴ洢
		}
		else
		{
			core_loose_normal_points->push_back(cable_core_pointcloud->at(i));   //��о��ɢ����ɢ���ƴ洢
		}

	}
	//showpointcloud_xyzrgb(cable_core_pointcloud);  //����Щ���ڰ뾶+1�ĵ������ʾ

	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_loose_defect_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_loose_normal_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//xyzrgb_2_xyz(core_loose_defect_points, core_loose_defect_points_xyz);
	//xyzrgb_2_xyz(core_loose_normal_points, core_loose_normal_points_xyz);
	//defect_generate_obj_loose(core_loose_defect_points_xyz, core_loose_normal_points_xyz);   //�ֲ�������ʾ������ʾȱ��


	double point_average = point_2_line_distnace_deviation / counter;
	double standard_parts_core_standard_value = 1.1947611367863262;  //�Է�����normal���¼��������ֵ
	int loose_flag = 0;
	if (point_average > standard_parts_core_standard_value)  //�жϵ�ƽ���в������ڱ�׼���ģ���֤����о��������ɢ
	{
		loose_flag = 1;
	}
	cable_region_sizz_measurement.conductor_deformation = loose_flag;		//10. conductor_deformation��־��0����ɢ��1��ɢ	 

	std::vector<Index_Strip> pointcloud_2_strip_all;						//������������������
	pointcloud_2_strip(cable_core_pointcloud, pointcloud_2_strip_all, 5);	//����о���ֵĵ��ƽ��г���,���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_top_plane_pointcloud->clear();
	for (int i = 0; i < pointcloud_2_strip_all.size(); i++)
	{
		for (int j = 0; j < pointcloud_2_strip_all.at(i).each_strip->size(); j++)
		{
			if (pointcloud_2_strip_all.at(i).each_strip->at(pointcloud_2_strip_all.at(i).each_strip->size() - 1).z - pointcloud_2_strip_all.at(i).each_strip->at(j).z  < 5)	//ȡ��ÿ����������1mm�ڵĵ㣬ֵȡ��һ�㣬������Ӵ��ĵ�ȡ����
			{
				cable_core_top_plane_pointcloud->push_back(pointcloud_2_strip_all.at(i).each_strip->at(j));   //��ÿ��������5mm�ڵĵ�ȡ����
			}
		}
	}
	//showpointcloud_xyzrgb(cable_core_top_plane_pointcloud);			//��ʾ��������������

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_uneven_defect_points(new pcl::PointCloud<pcl::PointXYZRGB>);  //�洢��о�������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr core_uneven_normal_points(new pcl::PointCloud<pcl::PointXYZRGB>);  //�洢��о�������
	core_uneven_defect_points->clear();  core_uneven_normal_points->clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delete(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_top_plane_pointcloud_delete->clear();
	double distance = 0;  //���潫ȡ�����Ķ�������Χ��ȥ����������о����Բ�뾶��ֵ�Ĺ�ϵ
	for (int i = 0; i < cable_core_top_plane_pointcloud->size(); i++)
	{
		distance = sqrt(pow((cable_core_top_plane_pointcloud->at(i).x - core_circle_x_average), 2) + pow((cable_core_top_plane_pointcloud->at(i).y - core_circle_y_average), 2)); //�����������)
		if (distance + 3  < core_circle_radius_average)  //����4mm��������Ϊ��ȷ����Χ�㶼��ȥ���ɾ�
		{
			cable_core_top_plane_pointcloud_delete->push_back(cable_core_top_plane_pointcloud->at(i));   //ȥ����о�������ҳ�����Բ���ϵĵ㣬�ҳ�������о����ĵ�
			core_uneven_defect_points->push_back(cable_core_top_plane_pointcloud->at(i));        //�洢��о�������***************************
		}
		else
		{
			core_uneven_normal_points->push_back(cable_core_top_plane_pointcloud->at(i));       //�洢��о�������****************
		}
	}
	//showpointcloud_xyzrgb(cable_core_top_plane_pointcloud_delete);	//��ʾ������ȥ��������

	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_uneve_defect_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr core_uneve_normal_points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//xyzrgb_2_xyz(core_uneven_defect_points, core_uneve_defect_points_xyz);
	//xyzrgb_2_xyz(core_uneven_normal_points, core_uneve_normal_points_xyz);
	//defect_generate_obj(core_uneve_defect_points_xyz, core_uneve_normal_points_xyz);   //�ֲ�������ʾ������ʾȱ��


	Plane_parameter plane_parameter;
	ransic_plane_estimation(cable_core_top_plane_pointcloud_delete, plane_parameter); //����о����ƽ��ʹ��ransic������ƽ����ϣ����ƽ�淨��
	pcl::PointNormal core_top_plane_normal; //��о��˵�ƽ�淨��
	core_top_plane_normal.normal_x = plane_parameter.plane_parameter_A;
	core_top_plane_normal.normal_y = plane_parameter.plane_parameter_B;
	core_top_plane_normal.normal_z = plane_parameter.plane_parameter_C;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cable_core_circle_center_pointcloud->clear();
	for (int i = 0; i < cable_core_circle_center.size(); i++)   //����о����Բ��Բ��ȡ��������������ɫ������һ�����ƣ�������ֱ�����
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
	//showpointcloud_xyzrgb(cable_core_circle_center_pointcloud);		//��ʾ��о��������Բ��Բ��

	Core_circle_center_line core_circle_center_line;
	ransic_line_estimation(cable_core_circle_center_pointcloud, core_circle_center_line);  //��оԲ����ֱ�����
	pcl::PointNormal core_circle_center_line_normal; //��оԲ��ֱ�ߵķ�����
	core_circle_center_line_normal.normal_x = core_circle_center_line.core_circle_center_line_x;
	core_circle_center_line_normal.normal_y = core_circle_center_line.core_circle_center_line_y;
	core_circle_center_line_normal.normal_z = core_circle_center_line.core_circle_center_line_z;

	float core_top_plane_core_circle_center_angel = 0;
	Eigen::Vector3f v1(core_top_plane_normal.normal_x, core_top_plane_normal.normal_y, core_top_plane_normal.normal_z);   //��о���淨��
	Eigen::Vector3f v2(core_circle_center_line_normal.normal_x, core_circle_center_line_normal.normal_y, core_circle_center_line_normal.normal_z);  //��о����Բ��ֱ�ߵķ�����
	core_top_plane_core_circle_center_angel = pcl::getAngle3D(v1, v2, true);  //���������߼нǣ�trueΪ�Ƕȱ�ʾ��falseΪ���ȱ�ʾ

	double Section_inclination = 0;
	double angle_2_radian = core_top_plane_core_circle_center_angel / (M_PI * 180);  //�Ƕ�תΪ����
	Section_inclination = tan(angle_2_radian) * (2 * core_circle_radius_average); //����������б�̶ȵ���tan��theta��*2r
	cable_region_sizz_measurement.conductor_section_inclination = Section_inclination;   //11.����������б*********************************************


																						 //***********************************************12.��о���氼͹��ƽ**************************************************************************************
	double standard_parts_standard_value = 0.14506740222996123;  //��׼����о�������е㵽���ƽ��Ĳв�͵ľ�ֵ,��Ϊÿ��������������һ�£����������ȡ���ֵ�в�������2mm������
	double residual_sum_top_plane_all_point = 0;
	double one_point_2_plane_distance_molecule = 0;
	double one_point_2_plane_distance_denominator = 0;
	double one_point_2_plane_distance = 0;

	for (int i = 0; i < cable_core_top_plane_pointcloud_delete->size(); i++)
	{
		one_point_2_plane_distance_molecule = plane_parameter.plane_parameter_A * cable_core_top_plane_pointcloud_delete->at(i).x + plane_parameter.plane_parameter_B *
			cable_core_top_plane_pointcloud_delete->at(i).y + plane_parameter.plane_parameter_C * cable_core_top_plane_pointcloud_delete->at(i).z + plane_parameter.plane_parameter_D;    //�㵽ƽ�����ķ���
		one_point_2_plane_distance_denominator = sqrt(pow(plane_parameter.plane_parameter_A, 2) + pow(plane_parameter.plane_parameter_B, 2) + pow(plane_parameter.plane_parameter_C, 2)); //�㵽ƽ�����ķ�ĸ
		one_point_2_plane_distance = abs(one_point_2_plane_distance_molecule / one_point_2_plane_distance_denominator);  //�㵽ƽ��ľ���,ȡ����ֵ		
		residual_sum_top_plane_all_point = residual_sum_top_plane_all_point + one_point_2_plane_distance;  //��о�������е㵽ƽ�����Ĳв��
	}
	double core_top_plane_point_residual_average = residual_sum_top_plane_all_point / cable_core_top_plane_pointcloud_delete->size();  //������о������в��ֵ
	int top_plane_flat_flag = 0;  //0��ʾ������ƽ��������Ϊ�ǰ�͹��ƽ��
	if (core_top_plane_point_residual_average > standard_parts_standard_value)  //����2�Ĳв�����
	{
		top_plane_flat_flag = 1;
	}
	cable_region_sizz_measurement.core_top_uneven = top_plane_flat_flag;  //12.�������氼͹��ƽ�������׼������ı�׼�Ȼ���Դ�Ϊ��ֵ�������ĥ�Ĵ������ֵ����Ϊ�ǰ�͹��ƽ��,Ϊ1���ǰ�͹��ƽ��Ϊ0������������������2

																		  //***********************************************13.������о����200mm������о����265mm��Բֱ���Ĳ�ֵ**************************************************************************************
	std::vector<int> distance_core_max_value_200_diameter_index; distance_core_max_value_200_diameter_index.clear(); //�洢������о����198mm-202mm��о��Բ������
	std::vector<int> distance_core_max_value_265_diameter_index; distance_core_max_value_265_diameter_index.clear(); //�洢������о����263mm-267mm��о��Բ������
	double diameter_200_sum = 0;	//�洢������о����200mm���Ҽ���Բֱ�����ܺ�
	double diameter_265_sum = 0;	//�洢������о����265mm���Ҽ���Բֱ�����ܺ�
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
	cable_region_sizz_measurement.outer_diameter_difference_200_265 = abs(distance_core_200_diameter - distance_core_265_diameter); //13.������о����200mm������о����265mm��Բֱ���Ĳ�ֵ


	cout << "��о���ȣ�" << cable_region_sizz_measurement.conductor_size << endl;
	cout << "��оǦ��ͷ���ȣ�" << cable_region_sizz_measurement.core_pencil_head_size << endl;
	cout << "�ڰ뵼�糤�ȣ�" << cable_region_sizz_measurement.inner_semiconducting_size << endl;
	cout << "�ڶ���Ǧ��ͷ���ȣ�" << cable_region_sizz_measurement.outside_pencil_head_size << endl;
	cout << "�ڶ���Ǧ��ͷ������о���˳��ȣ�" << cable_region_sizz_measurement.core_top_2_outside_pencil_head_distance << endl;
	cout << "��뵼����ɴ�ĩ�˾�����о���˳��ȣ�" << cable_region_sizz_measurement.core_outside_semiconducting_end << endl;
	cout << "��뵼����ɴ����ȣ�" << cable_region_sizz_measurement.outside_semiconducting_size << endl;
	cout << "����Ե�⾶ƫ�" << cable_region_sizz_measurement.main_insulation_deviation << endl;
	cout << "������о����15cm-220cm����Ե�⾶�ķ�Χ��" << cable_region_sizz_measurement.insulation_core_size_18_min << " - " << cable_region_sizz_measurement.insulation_core_size_18_max << endl;
	cout << "������ɢ���Σ�" << cable_region_sizz_measurement.conductor_deformation << endl;
	cout << "����������б��" << cable_region_sizz_measurement.conductor_section_inclination << endl;
	cout << "�������氼͹��" << cable_region_sizz_measurement.core_top_uneven << endl;
	cout << "������о����200mm��Բ�⾶��265mm��Բ��Ե�⾶�Ĳ�ֵ��" << cable_region_sizz_measurement.outer_diameter_difference_200_265 << endl;

	return cable_region_sizz_measurement;
}
/*���������������ư��Ƕȳ��һ��һ���ĵĵ���*/
bool Cable::pointcloud_2_strip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud, std::vector<Index_Strip> &pointcloud_2_strip_all, int strip_angle)
{
	rankpointcloud_z(cable_core_pointcloud);
	//std::cout << "��������������" << endl;
	if (cable_core_pointcloud->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	std::vector<int> index_strip;  //������
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����һ���ֵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����ʣ�ಿ�ֵ�
	int initial_angle = -180;
	pointcloud_2_strip_all.clear();   //�Դ��������������������г�ʼ��
	for (int i = 0; i < (360 / strip_angle); i++)  //ÿһ��ѭ���õ�һ����״�������
	{
		index_strip.clear();	  //��ʼ��
		pointcloud_strip->clear();  //��ʼ��
		remain->clear();
		for (int t = 0; t < cable_core_pointcloud->size(); t++)
		{
			double x = cable_core_pointcloud->at(t).x;
			double y = cable_core_pointcloud->at(t).y;
			if (x == 0)   //��ֹ��ĸΪ0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //�Ե���ÿһ������нǶȼ��㣬��ʵ�ְ��Ƕȳ���
														//cout << "i = " << i << " ʱ,t = " << t << " ʱ�Ƕȵ���" << angle << endl;
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

		//cout << "��" << i << "���ĵ���= " << pointcloud_strip->size() << endl; //ĳ�����������������ʾ
		if (pointcloud_strip->size())   //�������������д洢
		{
			Index_Strip index_strip_strcture;
			index_strip_strcture.index_each_strip.clear();
			index_strip_strcture.index_each_strip = index_strip;
			index_strip_strcture.remain_points = remain->makeShared();
			index_strip_strcture.each_strip = pointcloud_strip->makeShared();
			pointcloud_2_strip_all.push_back(index_strip_strcture);    //��ÿ�������������㷵��
		}
		//showpointcloud_xyzrgb(pointcloud_strip);   //��ǰ��������ʾ
		//showpointcloud_xyzrgb(remain);			//���³����ǰ����ʣ�����ʾ

	}

	return true;
}
/*RANSICƽ�����  ,��С���ˣ��չ������˵��뷨 ��ransic���չ˶����˵��뷨*/
bool Cable::ransic_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delet, Plane_parameter &plane_parameter)
{
	/*ransicƽ�����*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//����һ��ģ�Ͳ����������ڼ�¼���			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// ����һ���ָ���			
	seg.setOptimizeCoefficients(true);			//Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣			
	seg.setModelType(pcl::SACMODEL_PLANE);		// Mandatory-����Ŀ�꼸����״����Ҫ��ϵ���״		
	seg.setMethodType(pcl::SAC_RANSAC);			//�ָ�������������			
	seg.setDistanceThreshold(0.2);				//����������̷�Χ��Ҳ������ֵ  ͬ���ƽ��ľ��볬����ֵ�ĵ㣬�ͱ��ж�Ϊ��Ч���ݡ�
	seg.setInputCloud(cable_core_top_plane_pointcloud_delet);	//�������			
	seg.segment(*inliers, *coefficients);		//�ָ���ƣ����ƽ��ͷ����� coefficients�д洢����ƽ��ģ�͵�ϵ��A/B/C/D��inliers�洢��ϳ�ƽ��ĵ�,��������ģ�͵�����

	plane_parameter.plane_parameter_A = coefficients->values[0];
	plane_parameter.plane_parameter_B = coefficients->values[1];
	plane_parameter.plane_parameter_C = coefficients->values[2];  //�����ƽ������ȡ��ƽ�����A/B/C/D
	plane_parameter.plane_parameter_D = coefficients->values[3];

	/*��С����ƽ�����*/
	//Eigen::Vector4d centroid; //����
	//Eigen::Matrix3d covariance_matrix; //Э�������
	//pcl::computeMeanAndCovarianceMatrix(*cable_core_top_plane_pointcloud_delet, covariance_matrix, centroid);
	//Eigen::Matrix3d eigenVetors;
	//Eigen::Vector3d eigenValues;
	//pcl::eigen33(covariance_matrix, eigenVetors, eigenValues);
	////������С����ֵ��λ��
	//Eigen::Vector3d::Index minRow, minCol;
	//eigenValues.minCoeff(&minRow, &minCol);
	////��ȡƽ�淽�̵�ϵ��
	//Eigen::Vector3d normal = eigenVetors.col(minCol);
	//double D = -normal.dot(centroid.head<3>());

	//plane_parameter.plane_parameter_A = normal[0];
	//plane_parameter.plane_parameter_B = normal[1];
	//plane_parameter.plane_parameter_C = normal[2];  //�����ƽ������ȡ��ƽ�����A/B/C/D
	//plane_parameter.plane_parameter_D = D;

	return true;
}
/*RANSIC,���������*/
bool Cable::ransic_line_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud, Core_circle_center_line &core_circle_center_line)
{
	//ƽ�����
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);	//����һ��ģ�Ͳ����������ڼ�¼���			
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);	//inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����			
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;	// ����һ���ָ���			
	seg.setOptimizeCoefficients(true);			//Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣			
	seg.setModelType(pcl::SACMODEL_LINE);		// Mandatory-����Ŀ�꼸����״����Ҫ��ϵ���״		
	seg.setMethodType(pcl::SAC_RANSAC);			//�ָ�������������			
	seg.setDistanceThreshold(0.2);				//����������̷�Χ��Ҳ������ֵ  ͬ���ƽ��ľ��볬����ֵ�ĵ㣬�ͱ��ж�Ϊ��Ч���ݡ�
	seg.setInputCloud(cable_core_circle_center_pointcloud);	//�������			
	seg.segment(*inliers, *coefficients);		//�ָ���ƣ����ƽ��ͷ����� coefficients�д洢����ƽ��ģ�͵�ϵ��A/B/C/D��inliers�洢��ϳ�ƽ��ĵ�,��������ģ�͵�����

												/*//coefficient[0] ֱ����һ��� x ����
												coefficient[1] ֱ����һ��� y ����
												coefficient[2] ֱ����һ��� z ����
												coefficient[3] ֱ�߷��������� x ����
												coefficient[4] ֱ�߷��������� y ����
												coefficient[5] ֱ�߷��������� z ����
												*/
	core_circle_center_line.core_circle_center_line_x = coefficients->values[3];  //���ֱ�ߵķ�������
	core_circle_center_line.core_circle_center_line_y = coefficients->values[4];
	core_circle_center_line.core_circle_center_line_z = coefficients->values[5];

	return true;
}
/*RANSIC����Բ���*/
bool Cable::ransic_circle_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_main_insulation_circle_part_pointcloud, Circle &circle)
{

	pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>(cable_main_insulation_circle_part_pointcloud));	//ѡ����ϵ����뼸��ģ��
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3D);	//�����������һ���Զ���
	ransac.setDistanceThreshold(0.3);	//���þ�����ֵ����ģ�;���С��0.3�ĵ���Ϊ�ڵ�
	ransac.setMaxIterations(10000);		//��������������
	ransac.computeModel();				//ִ��ģ�͹���
	std::vector<int> inliers;			//�洢�ڵ�����������
	ransac.getInliers(inliers);			//��ȡ�ڵ��Ӧ������

										// ����������ȡ�ڵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud<pcl::PointXYZRGB>(*cable_main_insulation_circle_part_pointcloud, inliers, *cloud_circle);
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	circle.m_center_x = coefficient[0];  //Ϊ�˷��ظ����ú�������Բ��������뾶�����ô���
	circle.m_center_y = coefficient[1];
	circle.m_center_z = coefficient[2];
	circle.m_radius = coefficient[3];

	return true;
}
/*��Բ��Ϻ���*/
bool RGauss(const std::vector<std::vector<double> > & A, std::vector<double> & x)
{
	x.clear();
	int n = A.size();
	int m = A[0].size();
	x.resize(n);
	//����ϵ�����󣬷�ֹ�޸�ԭ����;
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
		//ѡ��Ԫ;
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
			//����ϵ�������l�к�k��;
			for (int i = 0; i < m; i++)
			{
				double temp = Atemp[l][i];
				Atemp[l][i] = Atemp[k][i];
				Atemp[k][i] = temp;
			}
		}
		//��Ԫ;
		for (int i = k + 1; i < n; i++)
		{
			double l = Atemp[i][k] / Atemp[k][k];
			for (int j = k; j < m; j++)
			{
				Atemp[i][j] = Atemp[i][j] - l*Atemp[k][j];
			}
		}
	}
	//�ش�;
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
/*XYZRGBתXYZ*/
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
/*ȫ�ֳ���������*/
bool Cable::regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)
{

	if (pointcloud_filtered->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	int strip_angle = 5;
	std::vector<int> index_strip;  //������
	std::vector<double> angle_number;  //��Ƕ�
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����һ���ֵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����ʣ�ಿ�ֵ�
	int initial_angle = -180;
	m_index_strip.clear();   //�Դ��������������������г�ʼ��
	for (int i = 0; i < (360 / strip_angle); i++)  //ÿһ��ѭ���õ�һ����״�������
	{
		index_strip.clear();	  //��ʼ��
		pointcloud_strip->clear();  //��ʼ��
		remain->clear();
		for (int t = 0; t < pointcloud_filtered->size(); t++)
		{
			double x = pointcloud_filtered->at(t).x;
			double y = pointcloud_filtered->at(t).y;
			if (x == 0)   //��ֹ��ĸΪ0,ֻ������x�ſ���Ϊ0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //�Ե���ÿһ������нǶȼ��㣬��ʵ�ְ��Ƕȳ���
														//cout << "i = " << i << " ʱ,t = " << t << " ʱ�Ƕȵ���" << angle << endl;
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

		//cout << "��" << i << "���ĵ���= " << pointcloud_strip->size() << endl; //ĳ�����������������ʾ
		if (pointcloud_strip->size())   //�������������д洢
		{
			int b, g, r; b = g = r = 0;
			random_colour(b, g, r);
			for (int j = 0; j < pointcloud_strip->size(); j++)  //��ÿһ���ֱ���ɫ
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
			m_index_strip.push_back(index_strip_strcture);    //��ÿ����״����ȫ�ֱ���

		}
	}
	return true;
}
/*�����������أ�����Ľ��������ȫ�ֱ���*/
bool Cable::regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<Index_Strip> &strip_vector, float strip_angle)
{

	if (pointcloud_filtered->empty())
	{
		std::cout << "point cloud is empty!" << std::endl;
		return false;
	}
	//int strip_angle = 1;
	std::vector<int> index_strip;  //������
	std::vector<double> angle_number;  //��Ƕ�
	std::vector<Index_Strip> strip_index_and_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����һ���ֵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain(new pcl::PointCloud<pcl::PointXYZRGB>);	//��һ����ʣ�ಿ�ֵ�
	int initial_angle = -180;
	strip_vector.clear();   //�Դ��������������������г�ʼ��
	for (int i = 0; i < (360 / strip_angle); i++)  //ÿһ��ѭ���õ�һ����״�������
	{
		index_strip.clear();	  //��ʼ��
		pointcloud_strip->clear();  //��ʼ��
		remain->clear();
		for (int t = 0; t < pointcloud_filtered->size(); t++)
		{
			double x = pointcloud_filtered->at(t).x;
			double y = pointcloud_filtered->at(t).y;
			if (x == 0)   //��ֹ��ĸΪ0,ֻ������x�ſ���Ϊ0
			{
				continue;
			}
			double angle = (atan2(y, x)) * 180 / M_PI;  //�Ե���ÿһ������нǶȼ��㣬��ʵ�ְ��Ƕȳ���
														//cout << "i = " << i << " ʱ,t = " << t << " ʱ�Ƕȵ���" << angle << endl;
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

		//cout << "��" << i << "���ĵ���= " << pointcloud_strip->size() << endl; //ĳ�����������������ʾ
		//����ͬ���ϲ�ͬɫ,����һ�����ܸ���ͬ��ɫ���Բ�ͬ��ɫ
		if (pointcloud_strip->size())   //�������������д洢
		{
			/*��ɫ*/
			//int b, g, r; b = g = r = 0;
			//random_colour(b, g, r);
			//for (int j = 0; j < pointcloud_strip->size(); j++)  //��ÿһ���ֱ���ɫ
			//{
			//	pointcloud_strip->at(j).b = b;
			//	pointcloud_strip->at(j).g = g;
			//	pointcloud_strip->at(j).r = r;
			//}
			/*��ɫ*/

			Index_Strip index_strip_strcture;
			index_strip_strcture.index_each_strip.clear();
			index_strip_strcture.index_each_strip = index_strip;
			index_strip_strcture.remain_points = remain->makeShared();
			index_strip_strcture.each_strip = pointcloud_strip->makeShared();
			strip_vector.push_back(index_strip_strcture);    //��ÿ����״����ȫ�ֱ���

		}
	}
	return true;
}
/*Բ�����*/
bool Cable::ransac_cylindrical_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cylindrical_pointcloud, Cylinder &cylinder)
{
	//-----------------------------Ŀ��㷨�߹���--------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;	// �������������ƶ���
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	n.setSearchMethod(tree);						       // ����������ʽ
	n.setInputCloud(Cylindrical_pointcloud);						           // �����������
	n.setKSearch(20);								       // ����K����������ĸ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);						          // ���㷨����������������浽normals��
														  //----------------------------Բ�����--------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;// ����Բ����ָ����
	seg.setInputCloud(Cylindrical_pointcloud);										// �����������
	seg.setInputNormals(normals);								    // �������뷨����
	seg.setOptimizeCoefficients(true);								// ���öԹ��Ƶ�ģ��ϵ����Ҫ�����Ż�
	seg.setModelType(pcl::SACMODEL_CYLINDER);						// ���÷ָ�ģ��ΪԲ����ģ��
	seg.setMethodType(pcl::SAC_RANSAC);								// ���ò���RANSAC�㷨���в�������
	seg.setNormalDistanceWeight(0.3);								// ���ñ��淨��Ȩ��ϵ��
	seg.setMaxIterations(10000);									// ���õ�����������
	seg.setDistanceThreshold(0.1);									// �����ڵ㵽ģ�;�������ֵ
	seg.setRadiusLimits(20, 40);									// ����Բ��ģ�Ͱ뾶�ķ�Χ

	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);	// ����ָ���
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);	// ����Բ����ģ��ϵ��
	seg.segment(*inliers_cylinder, *coefficients_cylinder);			// ִ�зָ���ָ������������浽inliers_cylinder�У�ͬʱ�洢ģ��ϵ��coefficients_cylinder

																	//cout << "����һ�����꣺(" << coefficients_cylinder->values[0] << ", "
																	//	<< coefficients_cylinder->values[1] << ", "
																	//	<< coefficients_cylinder->values[2] << ")"
																	//	<< endl;
																	//cout << "���߷���������(" << coefficients_cylinder->values[3] << ", "
																	//	<< coefficients_cylinder->values[4] << ", "
																	//	<< coefficients_cylinder->values[5] << ")"
																	//	<< endl;
																	//cout << "Բ����뾶��" << coefficients_cylinder->values[6] << endl;	

	cylinder.point_on_axis_x = coefficients_cylinder->values[0];
	cylinder.point_on_axis_y = coefficients_cylinder->values[1];
	cylinder.point_on_axis_z = coefficients_cylinder->values[2];
	cylinder.axis_direction_x = coefficients_cylinder->values[3];
	cylinder.axis_direction_y = coefficients_cylinder->values[4];
	cylinder.axis_direction_z = coefficients_cylinder->values[5];
	cylinder.radius = coefficients_cylinder->values[6];

	return 0;
}
/*����Ƭ*/
bool Cable::Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered)   	//��ÿ����������z�����,��ÿһ���ָ��һСƬһСƬ
{
	pcl::PointXYZRGB point;
	pcl::PointXYZRGB point2;
	std::vector<int> once_strip_slice_index;
	std::vector<Strip_slice> once_strip_get_slice;			 //������Ƭ������Ƭ����������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice(new pcl::PointCloud<pcl::PointXYZRGB>);
	double cable_z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;
	double cable_z_mim = pointcloud_filtered->at(0).z;
	double slice_point_heoght = 1;
	double increase_rate = slice_point_heoght;  //СƬ���Ƹ߶ȿ���
												//double increase_rate = (cable_z_max - cable_z_mim) / ( 2*(cable_z_max - cable_z_mim)  );  //��֤ÿƬ��2mm

	m_cable_strip_slice_all.clear();

	//std::cout << "��������������������������������������������������������Ƭ��ʼ��������������������������������������������" << endl;
	for (int strip = 0; strip < m_index_strip.size(); strip++)	//�Ե��µ�ÿ�������д���
	{
		//cout << "�� " << strip << "�����µĳ���Ϊ" << m_index_strip.at(strip).each_strip->at(m_index_strip.at(strip).each_strip->size() - 1).z - m_index_strip.at(strip).each_strip->at(0).z << endl;
		//std::cout << "�� " << strip << " ����Ƭ��ʼ��" << endl;
		once_strip_get_slice.clear();  //�洢һ���ϵ�Ƭ��Ƭ������
		point = m_index_strip.at(strip).each_strip->at(0);
		for (double k = cable_z_mim + (slice_point_heoght / 2); k < cable_z_max; k += increase_rate)   //��ÿ��С�����ҵ�z��������Ƭ����
		{
			point2 = point;
			point.z = k;
			point.x = point2.x;
			point.y = point2.y;
			pointcloud_strip_slice->clear();   //�洢һ��Ƭ
			once_strip_slice_index.clear();	   //�洢һ��Ƭ������
			for (int t = 0; t < m_index_strip.at(strip).each_strip->size(); t++)		//�ҳ�һ��zֵ��Χ�ڵ�Ƭ
			{
				double theta = m_index_strip.at(strip).each_strip->at(t).z - point2.z;
				if (-(slice_point_heoght / 2) < theta && theta < (slice_point_heoght / 2))
				{
					pointcloud_strip_slice->push_back(m_index_strip.at(strip).each_strip->at(t));
					once_strip_slice_index.push_back(t);
				}
			}
			//showpointcloud_xyzrgb(pointcloud_strip_slice);


			if (pointcloud_strip_slice->size())   //���ҳ�����СƬ��СƬ�������д洢
			{
				Strip_slice strip_slice;
				strip_slice.index_strip_slice = once_strip_slice_index;  //���ҳ�����СƬ�����������ṹ��
				strip_slice.pointcloud_strip_slice = pointcloud_strip_slice->makeShared();
				once_strip_get_slice.push_back(strip_slice);     //��һ��СƬ��СƬ������������
			}

			//����ͬƬ�ϲ�ͬɫ
			int b, g, r;
			for (int m = 0; m < once_strip_get_slice.size(); m++)
			{
				random_colour(b, g, r);	//random(0, 10)����һ��0-10���������������Ϊ�˸���ͬ��Ƭ�ṩ��ͬ��b��g��r���Ӷ�ʵ�ֲ�ͬ��ɫ����ʾ
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
		m_cable_strip_slice_all.push_back(once_strip_get_slice);    //���洢ÿ��СƬ�������洢������cable_strip_slice_all�ĳ�Ա��ÿ�����µ�СƬ������
																	//std::cout << "�� " << strip << " ����Ƭ����= " << once_strip_get_slice.size() << endl;
	}
	return true;
}
/*��Ƭ�������أ�����Ĳ�����ȫ�ֵ���Ƭ����*/
bool Cable::Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<std::vector<Strip_slice>> &slice_vector,
	std::vector<Index_Strip> XLPE_strip_vector, double slice_point_heoght)   	//��ÿ����������z�����,��ÿһ���ָ��һСƬһСƬ
{
	pcl::PointXYZRGB point;
	pcl::PointXYZRGB point2;
	std::vector<int> once_strip_slice_index;
	std::vector<Strip_slice> once_strip_get_slice;			 //������Ƭ������Ƭ����������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice(new pcl::PointCloud<pcl::PointXYZRGB>);
	double cable_z_max = pointcloud_filtered->at(pointcloud_filtered->size() - 1).z;
	double cable_z_mim = pointcloud_filtered->at(0).z;
	//double slice_point_heoght = 0.5;
	double increase_rate = slice_point_heoght;  //СƬ���Ƹ߶ȿ���
												//double increase_rate = (cable_z_max - cable_z_mim) / ( 2*(cable_z_max - cable_z_mim)  );  //��֤ÿƬ��2mm

	slice_vector.clear();

	//std::cout << "��������������������������������������������������������Ƭ��ʼ��������������������������������������������" << endl;
	for (int strip = 0; strip < XLPE_strip_vector.size(); strip++)	//�Ե��µ�ÿ�������д���
	{
		//cout << "�� " << strip << "�����µĳ���Ϊ" << XLPE_strip_vector.at(strip).each_strip->at(XLPE_strip_vector.at(strip).each_strip->size() - 1).z - XLPE_strip_vector.at(strip).each_strip->at(0).z << endl;
		//std::cout << "�� " << strip << " ����Ƭ��ʼ��" << endl;
		once_strip_get_slice.clear();  //�洢һ���ϵ�Ƭ��Ƭ������
		point = XLPE_strip_vector.at(strip).each_strip->at(0);
		for (double k = cable_z_mim + (slice_point_heoght / 2); k < cable_z_max; k += increase_rate)   //��ÿ��С�����ҵ�z��������Ƭ����
		{
			point2 = point;
			point.z = k;
			point.x = point2.x;
			point.y = point2.y;
			pointcloud_strip_slice->clear();   //�洢һ��Ƭ
			once_strip_slice_index.clear();	   //�洢һ��Ƭ������
			for (int t = 0; t < XLPE_strip_vector.at(strip).each_strip->size(); t++)		//�ҳ�һ��zֵ��Χ�ڵ�Ƭ
			{
				double theta = XLPE_strip_vector.at(strip).each_strip->at(t).z - point2.z;
				if (-(slice_point_heoght / 2) < theta && theta < (slice_point_heoght / 2))
				{
					pointcloud_strip_slice->push_back(XLPE_strip_vector.at(strip).each_strip->at(t));
					once_strip_slice_index.push_back(t);
				}
			}
			//showpointcloud_xyzrgb(pointcloud_strip_slice);


			if (pointcloud_strip_slice->size())   //���ҳ�����СƬ��СƬ�������д洢
			{
				Strip_slice strip_slice;
				strip_slice.index_strip_slice = once_strip_slice_index;  //���ҳ�����СƬ�����������ṹ��
				strip_slice.pointcloud_strip_slice = pointcloud_strip_slice->makeShared();
				once_strip_get_slice.push_back(strip_slice);     //��һ��СƬ��СƬ������������
			}

			//����ͬƬ�ϲ�ͬɫ,����һ�����ܸ���ͬ��ɫ���Բ�ͬ��ɫ
			//int b, g, r;
			//for (int m = 0; m < once_strip_get_slice.size(); m++)
			//{
			//	random_colour(b, g, r);	//random(0, 10)����һ��0-10���������������Ϊ�˸���ͬ��Ƭ�ṩ��ͬ��b��g��r���Ӷ�ʵ�ֲ�ͬ��ɫ����ʾ
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
			/*��ɫ*/

		}
		//showpointcloud_xyzrgb(XLPE_strip_vector.at(strip).each_strip);
		slice_vector.push_back(once_strip_get_slice);    //���洢ÿ��СƬ�������洢������cable_strip_slice_all�ĳ�Ա��ÿ�����µ�СƬ������
														 //std::cout << "�� " << strip << " ����Ƭ����= " << once_strip_get_slice.size() << endl;
	}
	return true;
}
/*�����ֵ����*/
bool random_colour(int &b, int &g, int &r)
{
	b = g = r = 0;
	b = random(0, 255);
	g = random(0, 255);
	r = random(0, 255);

	return true;
}
/*HSVתRGB*/
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