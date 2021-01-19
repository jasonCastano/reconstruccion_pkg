#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>

"""
Este Script se subscribe a las nubes de puntos de los cuatro sensores e implementando un filtro CropBox se filtra un área de trabajo de interés.
Posteriormente implementando un filtro RANSAC se reconoce los puntos pertencientes a un plano y son eliminados, lo que se busca es que se conserven
únicamente los puntos pertencientes a los objetos, en este caso personas, que se encuentran dentro del área de trabajo. 
"""
using namespace std;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

ros::Publisher pub_d1;
ros::Publisher pub_d3;
ros::Publisher pub_d4;
ros::Publisher pub_d5;

Eigen::Vector4f minPoint;
Eigen::Vector4f maxPoint;
pcl::CropBox<pcl::PointXYZRGB> cropFilter;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes(new pcl::PointCloud<pcl::PointXYZRGB>);
/*
Eigen::Affine3f T0i = Eigen::Affine3f::Identity();
Eigen::Affine3f T0d = Eigen::Affine3f::Identity();
Eigen::Affine3f T0s = Eigen::Affine3f::Identity();
Eigen::Affine3f T00 = Eigen::Affine3f::Identity();*/

pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> r_filter;
pcl::ExtractIndices<pcl::PointXYZRGB> extract;
std::vector<int> inliers;
pcl::PointIndices::Ptr inliers_ind(new pcl::PointIndices());

const float d_th = 0.09;

void pos0_callback(const sensor_msgs::PointCloud2ConstPtr& data){

    pcl::fromROSMsg(*data, *cloud);

    //std::cout<< "Mensaje de points_d1 recibido... \n" << std::endl;

    cropFilter.setInputCloud(cloud);
    cropFilter.filter(*Tcloud);
    r_filter.setInputCloud(Tcloud);
    r_filter.filter(*Tcloud);

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(Tcloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (d_th);
    ransac.computeModel();
    ransac.getInliers(inliers);

    //std::cout<< inliers.at(0) << std::endl;
    
    //pcl::copyPointCloud(*Tcloud, inliers, *planes);
    
    inliers_ind->indices = inliers;

    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers_ind);
    extract.setNegative(true);
    extract.filter(*planes);
    

    //*Tcloud -= *planes;

    pub_d1.publish(planes);
}

void posd_callback(const sensor_msgs::PointCloud2ConstPtr& data){

    pcl::fromROSMsg(*data, *cloud);

    cropFilter.setInputCloud(cloud);
    cropFilter.filter(*Tcloud);
    r_filter.setInputCloud(Tcloud);
    r_filter.filter(*Tcloud);
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(Tcloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (d_th);
    ransac.computeModel();
    ransac.getInliers(inliers);

    /*extract.setInputCloud(Tcloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*Tcloud);*/

    //pcl::copyPointCloud(*Tcloud, inliers, *planes);

    inliers_ind->indices = inliers;

    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers_ind);
    extract.setNegative(true);
    extract.filter(*planes);
    

    pub_d3.publish(planes);
}

void posi_callback(const sensor_msgs::PointCloud2ConstPtr& data){

    pcl::fromROSMsg(*data, *cloud);

    cropFilter.setInputCloud(cloud);
    cropFilter.filter(*Tcloud);
    r_filter.setInputCloud(Tcloud);
    r_filter.filter(*Tcloud);

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(Tcloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (d_th);
    ransac.computeModel();
    ransac.getInliers(inliers);
/*
    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*Tcloud);
*/
  //  pcl::copyPointCloud(*Tcloud, inliers, *planes);

    inliers_ind->indices = inliers;

    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers_ind);
    extract.setNegative(true);
    extract.filter(*planes);


    pub_d4.publish(planes);
}

void poss_callback(const sensor_msgs::PointCloud2ConstPtr& data){

    pcl::fromROSMsg(*data, *cloud);

    cropFilter.setInputCloud(cloud);
    cropFilter.filter(*Tcloud);
    r_filter.setInputCloud(Tcloud);
    r_filter.filter(*Tcloud);

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(Tcloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (d_th);
    ransac.computeModel();
    ransac.getInliers(inliers);
/*
    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*Tcloud);*/

   // pcl::copyPointCloud(*Tcloud, inliers, *planes);

    inliers_ind->indices = inliers;

    extract.setInputCloud(Tcloud);
    extract.setIndices(inliers_ind);
    extract.setNegative(true);
    extract.filter(*planes);


    pub_d5.publish(planes);
}

int main(int argc, char** argv){

    ros::init(argc, argv,"full_pc");
    ros::NodeHandle nh;

    pub_d1 = nh.advertise<sensor_msgs::PointCloud2>("filter_d1",1);
    pub_d3 = nh.advertise<sensor_msgs::PointCloud2>("filter_d3",1);
    pub_d4 = nh.advertise<sensor_msgs::PointCloud2>("filter_d4",1);
    pub_d5 = nh.advertise<sensor_msgs::PointCloud2>("filter_d5",1);

    minPoint[0] = -2;
	minPoint[1] = -1;
	minPoint[2] = 2;

    maxPoint[0] = 2;
	maxPoint[1] = 1;
	maxPoint[2] = 4;

    cropFilter.setMax(maxPoint);
    cropFilter.setMin(minPoint);

    r_filter.setRadiusSearch(0.05);
    r_filter.setMinNeighborsInRadius(20);
    r_filter.setKeepOrganized(true);
/*
    T0d.translation() << -1.48, -0.01, 1.40;
    T0i.translation() << 1.47, 0.02, 1.43;
    T0s.translation() << -0.03, 0, 3.08;
    T00.translation() << 0.01, 0.05 , 0;

    float theta = M_PI;
    T0i.rotate(Eigen::AngleAxisf(-theta/2, Eigen::Vector3f::UnitY()));
    T0s.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitY()));
    T0d.rotate(Eigen::AngleAxisf(theta/2, Eigen::Vector3f::UnitY()));
*/
    ros::Subscriber sub_pos0 = nh.subscribe("/depth_registerd/points_d1",1,pos0_callback);
    ros::Subscriber sub_posd = nh.subscribe("/depth_registerd/points_d3",1,posd_callback);
    ros::Subscriber sub_posi = nh.subscribe("/depth_registerd/points_d4",1,posi_callback);
    ros::Subscriber sub_poss = nh.subscribe("/depth_registerd/points_d5",1,poss_callback);

    std::cout << "Subscriptores completos \n" << std::endl;
    
    ros::spin();

}
