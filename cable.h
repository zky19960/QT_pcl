#pragma once
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/io.h>
#include <string>
#include <algorithm>
#include <fstream>
#include  <iostream> 
#include  <vector> 
#include  <string> 
#include  <math.h>
#include  <cstdlib>
#include  <ctime>
#include  <cstring>
#include  <pcl/io/pcd_io.h> 
#include  <pcl/io/ply_io.h> 
#include  <pcl/point_types.h> 
#include  <pcl/console/time.h>
#include  <pcl/console/print.h>
#include  <pcl/console/parse.h>
#include  <pcl/search/search.h> 
#include  <pcl/search/kdtree.h> 
#include  <pcl/point_cloud.h>
#include  <boost/thread/thread.hpp>
#include  <pcl/common/centroid.h>
#include  <pcl/common/common.h>
#include  <pcl/ModelCoefficients.h>
#include  <pcl/features/normal_3d.h> 
#include  <pcl/filters/passthrough.h> 
#include  <pcl/kdtree/kdtree_flann.h>
#include  <pcl/filters/extract_indices.h>
#include  <pcl/sample_consensus/ransac.h>
#include  <pcl/visualization/cloud_viewer.h> 
#include  <pcl/segmentation/region_growing.h> 
#include  <pcl/visualization/pcl_visualizer.h>
#include  <pcl/segmentation/sac_segmentation.h>
#include  <pcl/segmentation/extract_clusters.h>
#include  <pcl/filters/radius_outlier_removal.h>
#include  <pcl/features/integral_image_normal.h>
#include  <pcl/sample_consensus/sac_model_plane.h>
#include  <pcl/sample_consensus/sac_model_circle3d.h>
#include  <pcl/common/pca.h>
#include  <pcl/common/intersections.h>
#include  <pcl/segmentation/lccp_segmentation.h>
#include  <opencv2/opencv.hpp>
#include  <pcl/sample_consensus/sac_model_plane.h>
#include  <vtkAutoInit.h>
#include  <pcl/visualization/point_cloud_geometry_handlers.h>
#include  <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include  <pcl/common/eigen.h>
#include<random>
#include <pcl/common/generate.h> // ���ɸ�˹�ֲ��ĵ���
#include <boost/random.hpp>
using namespace std;
using namespace cv;

VTK_MODULE_INIT(vtkRenderingOpenGL2);    //������ʾ��������
#define random(a,b) (rand()%(b-a)+a)

										/*�ṹ�����Ͷ���*/

struct Circle					//�ṹ��Բ;����:x,y,z Ϊ��ά���꣬radiusΪ�뾶;
{
	double m_center_x;			//Բ������;
	double m_center_y;
	double m_center_z;
	double m_radius;			//�뾶;
};

struct IndexofCircle			//����Բ�Ľṹ���������־λ��Բ�ϵ������������
{
	Circle once_circle;			//����һ��Բ,����xyz��뾶r
	std::vector<int> each_circle_index;	//��vector�д洢Բ��ÿ���������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr each_circle_points;
};

struct Parameter_measurement
{

	double conductor_size;					//1.��о����ĳ���30mm
	double core_pencil_head_size;			//2.������о��Ǧ��ͷ����о���˵��ڰ뵼�����Ǧ��ͷ���ľ���95mm
	double inner_semiconducting_size;		//3.�ڰ뵼��ĳߴ�10mm
	double outside_pencil_head_size;		//4.Զ����о��Ǧ��ͷ������µĳߴ�30mm
	double core_top_2_outside_pencil_head_distance;						//5.Զ����оǦ��ͷ��������о���˵ľ���300mm
	double core_outside_semiconducting_end;								//6.��뵼���յ������о����600mm
	double outside_semiconducting_size;									//7.��뵼����ɴ��ĳߴ�40mm

	double main_insulation_deviation;			//8.����35cm-58cmͬһ����ľ�Ե�⾶ƫ��������ֵ����Сֵ֮��İ뾶���Բ��ϣ�,�����10��Բ����һ��
	double insulation_core_size_18_max;			//9.����������о����15cm-220��Բ���⾶���ֵ
	double insulation_core_size_18_min;			//9.����������о����15cm-220��Բ���⾶��Сֵ



	double conductor_deformation;			//10.������ɢ���Σ����Ϊ1������ɢ��Ϊ0���ǲ���ɢ
	double conductor_section_inclination;	//11.����������б,��о������ȥ��ÿ��������һ����Χ�㣬Ȼ���ٸ���ÿ���������ľ���ȥ��Բ����Χ�㣬��ʣ������ƽ�棬����ƽ�������߼нǣ�Ȼ��	h = tan��theta��*2r 
	double core_top_uneven;					//12.�������氼͹��ƽ�������׼������ı�׼�Ȼ���Դ�Ϊ��ֵ�������ĥ�Ĵ������ֵ����Ϊ�ǰ�͹��ƽ��,Ϊ1���ǰ�͹��ƽ��Ϊ0������������������

	double outer_diameter_difference_200_265;	//13.������о����200mm��265mm���⾶��
};

struct Index_Strip					//�ṹ������each_stripΪÿ��С����index_each_stripΪÿ��С��������,remain_points�ǵ��³���ʣ��ĵ�
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain_points;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr each_strip;
	std::vector<int> index_each_strip;	//��vector�д洢��������
};

struct Plane_parameter
{
	double plane_parameter_A;
	double plane_parameter_B;
	double plane_parameter_C;
	double plane_parameter_D;

};

struct Core_circle_center_line
{
	double  core_circle_center_line_x;
	double  core_circle_center_line_y;
	double  core_circle_center_line_z;
};

struct Cylinder
{
	double point_on_axis_x;
	double point_on_axis_y;
	double point_on_axis_z;
	double axis_direction_x;
	double axis_direction_y;
	double axis_direction_z;
	double radius;
};

struct Strip_slice
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice;   //һ�����ϵ�һ��СƬ
	std::vector<int> index_strip_slice;	//һ��СƬ����������Χ��һ����
};
struct Mapping_2D_3D
{
	int x;
	int y;
	double pixel_value;
};
struct Coordinate_value
{
	int x;
	int y;
	int pixel_value;

};

struct Contour_point
{
	cv::Point2f point_1;
	cv::Point2f point_2;
	cv::Point2f point_3;
	cv::Point2f point_4;
};

struct defection_quantification
{
	double defection_area; //ȱ��Ӱ��Բ�����
	double defection_area_defection;  //ȱ�ݱ����
	double defection_volume;//ȱ�����
	double defection_deepest_point;  //ȱ����ߵ�����͵�
	int defection_loaction_x;
	int defection_loaction_y;
	int defection_flag;
	double  conductEnd_2_outEnd_distance1;
};

struct Color_mapping   //���ݵ㵽ƽ��ľ�����ɫ
{
	int strip_index;
	int point_index_of_slice;
	int slice_index_of_strip;
	double point_2_plane_distance;
};


/*�����ں�������*/
bool random_colour(int &b, int &g, int &r);
bool loadPointcloud(std::string strFilename, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src);   //��ȡ�����ļ�
bool xyz2rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb);     //XYZתXYZRGB
bool showpointcloud_xyzhsv(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pointcloud_in);   //��ʾxyzhsv
bool showpointcloud_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in);        //������ʾxyz
bool showpointcloud_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in);	//������ʾxyzrgb	
bool showpointcloud_xyzrgbnormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud_in);
bool ransice_circular_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Circle &circle);
bool RGauss(const std::vector<std::vector<double> > & A, std::vector<double> & x);
std::vector<double> getEllipseparGauss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in);   //��Բ��Ϻ���
bool xyzrgb_2_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_in, pcl::PointCloud<pcl::PointXYZ>::Ptr point_out);  //��������ת����xyzrgbתxyz
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D);
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point); //RANSACƽ����Ϻ���
bool show_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice, pcl::PointCloud <pcl::Normal>::Ptr normals);
bool least_square_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pointCloud, float &A, float &B, float &C, float &D);
//bool add_gauss_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);

typedef struct
{
	std::vector<double> inner_semiconducting_circle_radius;
    std::vector<double> XLPE_insulation_circle_radius;
	std::vector<double> outer_semiconducting_circle_radius;
	std::vector<double> conduct_circle_radius;

	double cable_z_max_value = 0;

	
}Para;

class Cable
{
public:
	Cable();
	Cable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb);   //���캯������

public:
	bool filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_filtered);  //�����˲�
	bool z_value_circle_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double distance); //����zֵ�Ե��²��ֽ���Բ���
	bool calculation_standard_deviation(int index, int range, double &standard_fore, double &standard_back); //�Ը�����Բ����ǰ��Բ�İ뾶���׼��
	bool rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end);	//����ָ�������Բ֮��ı�׼���ϵ
	bool fusion_iteration_detail_segmentation(int circle_number, int field, int flag, std::vector<Circle> &region_circles);  //�ڴַָ�����ϣ�ȥ��������յ㣬�Ը�Сzֵ���Բ
	bool update_segmentation_start(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value); //�ַָ�󾫷ָ������㣬back/fore
	bool update_segmentation_end(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value);	//�ַָ�󾫷ָ�����յ㣬fore/back
	Parameter_measurement segmentation_region_size_measurement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<double> region_start, std::vector<double> region_end); //�������¸����ֲ���
	bool pointcloud_2_strip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud, std::vector<Index_Strip> &pointcloud_2_strip_all, int strip_angle);  //�Ը������ƽ��г�������������Ƕ�
	bool ransic_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delet, Plane_parameter &plane_parameter);  //ransicƽ�����
	bool ransic_line_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud, Core_circle_center_line &core_circle_center_line);	//ransicֱ�߹���
	bool ransic_circle_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_main_insulation_circle_part_pointcloud, Circle &circle);    //ransicԲ����
	bool ransac_cylindrical_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cylindrical_pointcloud, Cylinder &cylinder); //Բ�����
	bool regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);  //ȫ�ֳ���
	bool regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<Index_Strip> &strip_vector, float strip_angle); //�����������أ��еĵط�����ʹ��ȫ�ֳ���
	bool Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);  //����״���½�����Ƭ
	bool Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<std::vector<Strip_slice>> &slice_vector,
		std::vector<Index_Strip> XLPE_strip_vector, double slice_point_heoght);//��Ƭ�������أ��еĵط�������Ҫȫ����Ƭ
																			   //bool concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);
	bool concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, float each_strip_angle, double slice_height, double conductEnd_2_outEnd_distance,
		std::vector<defection_quantification> &Defection_quantification, std::vector<defection_quantification> &Defection_quantification_concave);
	bool function_loop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);
	
	void getcp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cp) //ui��������
	{
		m_pointcloud = *cp;
		m_pointcloud_rgb = m_pointcloud.makeShared();
	}

	Para getpara(){return para;}
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr __colour_point_cloud_hsv_all__;

	bool HSVtoRGB(std::vector<int> hsv, std::vector<int>& rgb);
	~Cable();

private:  //��private������һЩȫ�ֱ���
	pcl::PointCloud<pcl::PointXYZRGB> m_pointcloud; 				//RGB���͵���,ȫ�ֱ���														
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointcloud_rgb;		//RGB���͵��Ƶ�ָ��;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointcloud_filtered;   //rgb�˲���ĵ���
	std::vector<Index_Strip> m_index_strip;				//������������������
	std::vector<IndexofCircle> m_indexandcircles;   //�����Բ������
	std::vector<std::vector<Strip_slice>> m_cable_strip_slice_all; //����������������Ƭ,Ҳ����ÿ������Ƭ����һ��vector�������Ƕ�������ڶ���vector�����ÿ����Ƭ�Լ�Ƭ������

	Para para;

private:
	bool rankpointcloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank);						//����zֵ��С�Ե��ƽ�������;
	bool heapify_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank, int n_first, int n_end);		//zֵ������



};

