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
#include <pcl/common/generate.h> // 生成高斯分布的点云
#include <boost/random.hpp>
using namespace std;
using namespace cv;

VTK_MODULE_INIT(vtkRenderingOpenGL2);    //法线显示所需声明
#define random(a,b) (rand()%(b-a)+a)

										/*结构体类型定义*/

struct Circle					//结构体圆;参数:x,y,z 为三维坐标，radius为半径;
{
	double m_center_x;			//圆心坐标;
	double m_center_y;
	double m_center_z;
	double m_radius;			//半径;
};

struct IndexofCircle			//包含圆的结构体变量，标志位，圆上点的索引的容器
{
	Circle once_circle;			//定义一个圆,包含xyz与半径r
	std::vector<int> each_circle_index;	//该vector中存储圆中每个点的索引
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr each_circle_points;
};

struct Parameter_measurement
{

	double conductor_size;					//1.线芯导体的长度30mm
	double core_pencil_head_size;			//2.靠近线芯的铅笔头，线芯顶端到内半导电这个铅笔头起点的距离95mm
	double inner_semiconducting_size;		//3.内半导电的尺寸10mm
	double outside_pencil_head_size;		//4.远离线芯的铅笔头这个下坡的尺寸30mm
	double core_top_2_outside_pencil_head_distance;						//5.远离线芯铅笔头起点距离线芯顶端的距离300mm
	double core_outside_semiconducting_end;								//6.外半导电终点距离线芯顶端600mm
	double outside_semiconducting_size;									//7.外半导电过渡带的尺寸40mm

	double main_insulation_deviation;			//8.计算35cm-58cm同一截面的绝缘外径偏差，计算最大值与最小值之间的半径差（椭圆拟合）,这里隔10个圆计算一个
	double insulation_core_size_18_max;			//9.测量距离线芯顶端15cm-220处圆的外径最大值
	double insulation_core_size_18_min;			//9.测量距离线芯顶端15cm-220处圆的外径最小值



	double conductor_deformation;			//10.导体松散变形，结果为1就是松散，为0就是不松散
	double conductor_section_inclination;	//11.导体切面倾斜,线芯抽条，去除每条最上面一定范围点，然后再根据每个点与轴心距离去除圆柱周围点，用剩余点拟合平面，计算平面与轴线夹角，然后	h = tan（theta）*2r 
	double core_top_uneven;					//12.导体切面凹凸不平，计算标准件切面的标准差，然后以此为阈值，后面打磨的大于这个值就认为是凹凸不平的,为1就是凹凸不平，为0就是正常，留有余量

	double outer_diameter_difference_200_265;	//13.距离线芯顶端200mm与265mm的外径差
};

struct Index_Strip					//结构体条，each_strip为每个小条，index_each_strip为每个小条的索引,remain_points是电缆抽条剩余的点
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr remain_points;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr each_strip;
	std::vector<int> index_each_strip;	//该vector中存储条的索引
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_strip_slice;   //一个条上的一个小片
	std::vector<int> index_strip_slice;	//一个小片的索引，范围是一个条
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
	double defection_area; //缺陷影响圆柱面积
	double defection_area_defection;  //缺陷表面积
	double defection_volume;//缺陷体积
	double defection_deepest_point;  //缺陷最高点与最低点
	int defection_loaction_x;
	int defection_loaction_y;
	int defection_flag;
	double  conductEnd_2_outEnd_distance1;
};

struct Color_mapping   //根据点到平面的距离上色
{
	int strip_index;
	int point_index_of_slice;
	int slice_index_of_strip;
	double point_2_plane_distance;
};


/*非类内函数声明*/
bool random_colour(int &b, int &g, int &r);
bool loadPointcloud(std::string strFilename, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src);   //读取点云文件
bool xyz2rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb);     //XYZ转XYZRGB
bool showpointcloud_xyzhsv(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pointcloud_in);   //显示xyzhsv
bool showpointcloud_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in);        //点云显示xyz
bool showpointcloud_xyzrgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in);	//点云显示xyzrgb	
bool showpointcloud_xyzrgbnormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud_in);
bool ransice_circular_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Circle &circle);
bool RGauss(const std::vector<std::vector<double> > & A, std::vector<double> & x);
std::vector<double> getEllipseparGauss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in);   //椭圆拟合函数
bool xyzrgb_2_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_in, pcl::PointCloud<pcl::PointXYZ>::Ptr point_out);  //点云类型转换，xyzrgb转xyz
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D);
bool ransac_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr strip_one_slice, float &A, float &B, float &C, float &D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr interior_point); //RANSAC平面拟合函数
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
	Cable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rgb);   //构造函数重载

public:
	bool filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_filtered);  //点云滤波
	bool z_value_circle_fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double distance); //根据z值对电缆部分进行圆拟合
	bool calculation_standard_deviation(int index, int range, double &standard_fore, double &standard_back); //对给定的圆索引前后圆的半径求标准差
	bool rough_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, double circle_distance, std::vector<double> &region_start, std::vector<double> &region_end);	//区域分割，根据拟合圆之间的标准差关系
	bool fusion_iteration_detail_segmentation(int circle_number, int field, int flag, std::vector<Circle> &region_circles);  //在粗分割基础上，去区域起点终点，以更小z值拟合圆
	bool update_segmentation_start(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value); //粗分割后精分割，对于起点，back/fore
	bool update_segmentation_end(std::vector<Circle> region_circles, int k, double cable_z_minimum_value, double &update_region_z_value);	//粗分割后精分割，对于终点，fore/back
	Parameter_measurement segmentation_region_size_measurement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<double> region_start, std::vector<double> region_end); //测量电缆各部分参数
	bool pointcloud_2_strip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_pointcloud, std::vector<Index_Strip> &pointcloud_2_strip_all, int strip_angle);  //对给定点云进行抽条，按极坐标角度
	bool ransic_plane_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_top_plane_pointcloud_delet, Plane_parameter &plane_parameter);  //ransic平面估计
	bool ransic_line_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_core_circle_center_pointcloud, Core_circle_center_line &core_circle_center_line);	//ransic直线估计
	bool ransic_circle_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_main_insulation_circle_part_pointcloud, Circle &circle);    //ransic圆估计
	bool ransac_cylindrical_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cylindrical_pointcloud, Cylinder &cylinder); //圆柱拟合
	bool regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);  //全局抽条
	bool regionSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<Index_Strip> &strip_vector, float strip_angle); //抽条函数重载，有的地方不能使用全局抽条
	bool Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);  //对条状电缆进行切片
	bool Strip_2_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, std::vector<std::vector<Strip_slice>> &slice_vector,
		std::vector<Index_Strip> XLPE_strip_vector, double slice_point_heoght);//切片函数重载，有的地方并不需要全局切片
																			   //bool concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);
	bool concave_convex_defect_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered, float each_strip_angle, double slice_height, double conductEnd_2_outEnd_distance,
		std::vector<defection_quantification> &Defection_quantification, std::vector<defection_quantification> &Defection_quantification_concave);
	bool function_loop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered);
	
	void getcp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cp) //ui交互函数
	{
		m_pointcloud = *cp;
		m_pointcloud_rgb = m_pointcloud.makeShared();
	}

	Para getpara(){return para;}
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr __colour_point_cloud_hsv_all__;

	bool HSVtoRGB(std::vector<int> hsv, std::vector<int>& rgb);
	~Cable();

private:  //该private用来存一些全局变量
	pcl::PointCloud<pcl::PointXYZRGB> m_pointcloud; 				//RGB类型点云,全局变量														
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointcloud_rgb;		//RGB类型点云的指针;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointcloud_filtered;   //rgb滤波后的点云
	std::vector<Index_Strip> m_index_strip;				//存条和条索引的容器
	std::vector<IndexofCircle> m_indexandcircles;   //存拟合圆的容器
	std::vector<std::vector<Strip_slice>> m_cable_strip_slice_all; //存整根电缆上条的片,也就是每个条的片，第一个vector里面存的是多个条，第二个vector里面存每条的片以及片的索引

	Para para;

private:
	bool rankpointcloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank);						//根据z值大小对点云进行有序化;
	bool heapify_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_rank, int n_first, int n_end);		//z值堆排序



};

