#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <std_msgs/Float32.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>


float x_vel , min_dist;
ros::Publisher pub1 , pub_value;

void cloud_cb (const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
    std_msgs::Float32 value;
    value.data = static_cast<float>(value.data);
    
    pcl::PCLPointCloud2 cl2;
    pcl_conversions::toPCL( *cloud_msg , cl2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2(cl2, *cloud1);

    geometry_msgs::Twist msg;
    msg.linear.x = x_vel ;


    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud1 , centroid );
    // ROS_WARN("centr_x: %f", centroid[0]); // x centroid
    // ROS_WARN("centr_y: %f", centroid[1]); // y centroid
    ROS_WARN("centr_z: %f", centroid[2]); // z centroid

    if (centroid[2] > min_dist){
        pub1.publish(msg);
        value.data = 0;
    }
    else {
        value.data = 1.0;
    }

    pub_value.publish(value);

}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "pointcloud_distance");
    ros::NodeHandle nh;

    std::string in_topic, out_topic;
    nh.param("pointcloud_distance/input_topic", in_topic, std::string("/camera/depth/points"));
    nh.param("pointcloud_distance/x_vel", x_vel , 0.1f);
    nh.param("pointcloud_distance/min_dist", min_dist, 1.0f);

    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_cb);
    pub_value = nh.advertise<std_msgs::Float32>("value",1);
    pub1 = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::spin ();
}