#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <point_cloud_tools/MeshToCloudPose.h>
#include <ros/ros.h>

#include <multi_obj/transformation_analyzation.h>


bool debug_;
ros::Publisher align_cloud, processed_pub;

TransformationAnalyzation <pcl::PointXYZ> ta;

void processSensorClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr table, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
  {
    tf::Transform tr;
    Eigen::Affine3d tr_eigen;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointXYZ minPt, maxPt;
    double yaw = ta.computeYaw(object);
    
    ta.yawObject(tr, yaw);
    tf::transformTFToEigen(tr,tr_eigen);
    pcl::transformPointCloud(*object, *object, tr_eigen);
    pcl::transformPointCloud(*table, *table, tr_eigen);
    
    
    pcl::getMinMax3D(*object, minPt, maxPt);

    pass.setInputCloud(table);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minPt.x, maxPt.x);
    pass.filter(*out);

    pass.setInputCloud(out);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPt.y, maxPt.y);
    pass.filter(*out);

    *out = *object + *out;
    ta.yawObject(tr, -yaw);
    tf::transformTFToEigen(tr,tr_eigen);
    pcl::transformPointCloud(*object, *object, tr_eigen);
    pcl::transformPointCloud(*table, *table, tr_eigen);
    pcl::transformPointCloud(*out, *out, tr_eigen);
  }

void appendPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
  float eps = 0.01;
  tf::Transform tr;
  Eigen::Affine3d tr_eigen;
  pcl::PointXYZ minPt, maxPt, Pt;
  double yaw = ta.computeYaw(cloud);
  
  ta.yawObject(tr, yaw);
  tf::transformTFToEigen(tr,tr_eigen);
  pcl::transformPointCloud(*cloud, *out, tr_eigen);

  pcl::getMinMax3D(*out, minPt, maxPt);
  Pt.z = minPt.z;
  for(float x=minPt.x-eps;x<maxPt.x+eps;x+=0.001)
    {
      for(float y=minPt.y-eps;y<maxPt.y+eps;y+=0.001)
	{
	  Pt.x=x;
	  Pt.y=y;
	  out->push_back(Pt);
	}
    }

  ta.yawObject(tr, -yaw);
  tf::transformTFToEigen(tr,tr_eigen);
  pcl::transformPointCloud(*out, *out, tr_eigen);
}

bool MeshFitCloudPoseSrv(point_cloud_tools::MeshToCloudPose::Request &req, point_cloud_tools::MeshToCloudPose::Response &res)
{
  pcl::PCLPointCloud2 tempCloud;
  pcl::PointCloud<pcl::PointXYZ> Final;
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointXYZ minPt, maxPt;
  Eigen::Matrix4f guess, out_mat;
  float x, y, z, roll, pitch, yaw;
  tf::Quaternion rot;
  tf::Transform out_tf;
  tf::TransformBroadcaster trans_br;
  
  if(debug_)
    {
      ROS_INFO("Service Call Received");
    }
  
  pcl_conversions::toPCL(req.object_cloud, tempCloud);
  pcl::fromPCLPointCloud2(tempCloud, *camera_object_cloud);

  pcl_conversions::toPCL(req.table_cloud, tempCloud);
  pcl::fromPCLPointCloud2(tempCloud, *camera_table_cloud);

  processSensorClouds(camera_object_cloud,camera_table_cloud,camera_processed_cloud);
  
  pcl::PolygonMesh::Ptr stl_file(new pcl::PolygonMesh);
  if(pcl::io::loadPolygonFileSTL(req.mesh_path, *stl_file) < 0)
    {
      std::stringstream Error;
      Error << "\"" << req.mesh_path <<"\" "  << "File Not Found";
      ROS_INFO(Error.str().c_str());
      return false;
    }
  pcl::fromPCLPointCloud2(stl_file->cloud, *full_object_cloud);

  if(debug_)
    ROS_INFO("Loaded Clouds");

  appendPlane(full_object_cloud, full_processed_cloud);

  
  camera_processed_cloud->header.stamp = full_object_cloud->header.stamp;
  
  icp.setInputSource(camera_processed_cloud);
  icp.setInputTarget(full_processed_cloud);
  icp.setMaximumIterations(500);
  icp.setTransformationEpsilon (1e-12);
  icp.setEuclideanFitnessEpsilon (1e-6);
  icp.setRANSACOutlierRejectionThreshold(0.001);

  pcl::getMinMax3D(*camera_processed_cloud, minPt, maxPt);
  
  guess << 1,0,0,-(minPt.x+maxPt.x)*0.5,
    0,1,0,-(minPt.y+maxPt.y)*0.5,
    0,0,1,-minPt.z,
    0,0,0,1;
    
  icp.align(Final, guess);
  Final.header.frame_id = "/world";
  full_object_cloud->header.frame_id = "/world";
  sensor_msgs::PointCloud2 send_cloud;
  pcl::toROSMsg(*camera_processed_cloud,send_cloud);
  
  out_mat = icp.getFinalTransformation();

  icp.setInputSource(camera_object_cloud);
  icp.setInputTarget(full_object_cloud);
  icp.align(Final, out_mat);
  out_mat = icp.getFinalTransformation();
  
  Eigen::Transform<float, 3, Eigen::Affine> out_trans(out_mat);
  pcl::getTranslationAndEulerAngles(out_trans, x, y, z, roll, pitch, yaw);
  rot.setRPY(roll,pitch,yaw);

  res.estimated_pose.header.stamp = ros::Time::now();
  res.estimated_pose.header.frame_id = "/world";
  res.estimated_pose.pose.position.x = x;
  res.estimated_pose.pose.position.y = y;
  res.estimated_pose.pose.position.z = z;

  res.estimated_pose.pose.orientation.x = rot[0];
  res.estimated_pose.pose.orientation.y = rot[1];
  res.estimated_pose.pose.orientation.z = rot[2];
  res.estimated_pose.pose.orientation.w = rot[3];

  if(debug_)
    {
      out_tf.setOrigin(tf::Vector3(x,y,z));
      out_tf.setRotation(rot);
      ros::Rate r(10);
      for(int i=0; i<10; i++)
	{
	  trans_br.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),"/ICPTest","/world"));
	  r.sleep();
	}
      align_cloud.publish(send_cloud); 
      pcl::toROSMsg(*full_object_cloud,send_cloud);
      processed_pub.publish(send_cloud);
    }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TableTop_PointCloud_Pose_Estimation_Node");
  ros::NodeHandle n;
  n.param("debug",debug_,false);

  if(debug_)
    {
      align_cloud = n.advertise<sensor_msgs::PointCloud2>("/align_cloud",1);
      processed_pub = n.advertise<sensor_msgs::PointCloud2>("/processed",1);
    }

  ros::ServiceServer service = n.advertiseService("TableTop_PointCloud_Pose_Estimation", MeshFitCloudPoseSrv);
    while(n.ok())
    {
      ros::spinOnce();
    }
}
