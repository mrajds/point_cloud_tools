#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>

#include <point_cloud_tools/MeshToCloudPose.h>

sensor_msgs::PointCloud2 send_cloud;
sensor_msgs::PointCloud2 table_cloud;
bool sensor_data = false;
bool table_data = false;

void cloud_cb(sensor_msgs::PointCloud2 dat)
{
  send_cloud = dat;
  sensor_data = true;
}

void table_cb(sensor_msgs::PointCloud2 dat)
{
  table_cloud = dat;
  table_data = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ICP_Test_client");
  ros::NodeHandle n;
  ros::Rate rate(10);
  ros::ServiceClient client = n.serviceClient<point_cloud_tools::MeshToCloudPose>("TableTop_PointCloud_Pose_Estimation");
  point_cloud_tools::MeshToCloudPose req;
  req.request.mesh_path="/home/mohanraj/ycb/053_mini_soccer_ball/google_16k/nontextured.stl";
  ros::Subscriber sub = n.subscribe("/object_cloud",100,cloud_cb);
  ros::Subscriber subt = n.subscribe("/table_cloud",100,table_cb);
  ROS_INFO("Subscriber Created");
  while(!(sensor_data && table_data))
    ros::spinOnce();
    rate.sleep();
  req.request.object_cloud = send_cloud;
  req.request.table_cloud = table_cloud;
  if(client.call(req))
    ROS_INFO("Success");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Object_Mesh",0);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "ICPTest";
  marker.header.stamp = ros::Time();
  marker.ns = "ICPTest";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = std::string("file://") + req.request.mesh_path;
  marker.color.a = 0.8;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  for(int i=0; i<10; i++)
    {
      marker_pub.publish(marker);
      rate.sleep();
    }
  return 0;
}
