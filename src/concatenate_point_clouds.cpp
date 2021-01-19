#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/transforms.h>

#Este Script se subscribe a cada una de las nubes de puntos de cada sensor y las concatena teniendo en cuenta su ubicación espacial en una única nube de puntos

pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr d1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr d3_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr d4_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr d5_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher pub;

Eigen::Quaternionf q_d1(0.997188818112,-0.0749297072727,0,0);
Eigen::Quaternionf q_d3(-0.00595392682759,-0.0220990544393,-0.996666732536,-0.0783045771571);
Eigen::Quaternionf q_d4(0.692359991806,-0.0602043253322,0.716541741445,0.0598415718807);
Eigen::Quaternionf q_d5(0.712136105765,-0.0570927383017,-0.697478177833,-0.0559175960266);

Eigen::Vector3f d_d1 = Eigen::Vector3f(0.01,0.05,0.0);
Eigen::Vector3f d_d3 = Eigen::Vector3f(0.03,0.15,7.3);
Eigen::Vector3f d_d4 = Eigen::Vector3f(-3.6,0.2,3.63);
Eigen::Vector3f d_d5 = Eigen::Vector3f(3.75,0.0,3.7);


void callback(const sensor_msgs::PointCloud2ConstPtr& d1_points, const sensor_msgs::PointCloud2ConstPtr& d3_points, const sensor_msgs::PointCloud2ConstPtr& d4_points, const sensor_msgs::PointCloud2ConstPtr& d5_points)
{
    pcl::fromROSMsg(*d1_points, *d1_cloud);
    pcl::fromROSMsg(*d3_points, *d3_cloud);
    pcl::fromROSMsg(*d4_points, *d4_cloud);
    pcl::fromROSMsg(*d5_points, *d5_cloud);

    pcl::transformPointCloud(*d1_cloud,*d1_cloud,d_d1,q_d1);
    pcl::transformPointCloud(*d3_cloud,*d3_cloud,d_d3,q_d3);
    pcl::transformPointCloud(*d4_cloud,*d4_cloud,d_d4,q_d4);
    pcl::transformPointCloud(*d5_cloud,*d5_cloud,d_d5,q_d5);

    *full_cloud=*d1_cloud;
    *full_cloud+=*d3_cloud;
    *full_cloud+=*d4_cloud;
    *full_cloud+=*d5_cloud;

    pub.publish(full_cloud);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "concatenate_node");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("concatenate_points",1);
    
    #Se implementa message_filters que permite que se implemente la función de callback una vez se ha recibido mensajes de los cuatro sensores. 

    message_filters::Subscriber<sensor_msgs::PointCloud2> d1_points(nh, "filter_d1",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> d3_points(nh, "filter_d3",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> d4_points(nh, "filter_d4",1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> d5_points(nh, "filter_d5",1);

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> sync(d1_points,d3_points,d4_points,d5_points,2);
    sync.registerCallback(boost::bind(&callback,_1,_2,_3,_4));

    ros::spin();

}
