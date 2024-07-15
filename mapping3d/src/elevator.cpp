#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <std_msgs/UInt32.h>

class Elevator
{
public:
  Elevator() : nh_("~")

  {
    cloud_sub_ = nh_.subscribe("/aligned_cloud", 1, &Elevator::cloudCallback, this);
    odom_sub_ = nh_.subscribe("/gazebo/controllers/diff_drive/odom", 1, &Elevator::odomCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("elevated_cloud", 1);


    
    //Inicializar vector para nube de puntos
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    


// procesamiento del escaneo del láser y la odometría para determinar la elevación actual del sensor
// ...

    // Add elevation to each point
    for (auto& point : cloud->points)
    {
      point.x += elevation_;
    }

    // Publish elevated cloud
    sensor_msgs::PointCloud2 elevated_cloud_msg;
    pcl::toROSMsg(*cloud, elevated_cloud_msg);
    elevated_cloud_msg.header = cloud_msg->header;
    cloud_pub_.publish(elevated_cloud_msg);
    
  }
  
  double getCurrentElevation(double base_z)
  {
    // In this example, we simply return the base height as the current elevation
    return base_z;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    // Get elevation from odometry
    elevation_ = odom_msg->pose.pose.position.x;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher elevated_clouds_pub_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> elevated_clouds_;
  double elevation_ = 0.0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevator_node");
  Elevator elevator;
  
  ros::spin();
  return 0;
}

