#include <iostream>
#include <map>
#include <mutex>

#include "ros/ros.h"

#include <robosherlock_msgs/RSObjectDescriptions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <uwds3_msgs/WorldStamped.h>
#include <uwds3_msgs/World.h>
#include <uwds3_msgs/SceneNode.h>

#include <std_msgs/String.h>

#include "rs_display/PropertyData.h"
#include "rs_display/Object.h"
#include "rs_display/DataReader.h"

// Info on clicked object pose
//#define DEBUG_PRINT



rs::Object getObject(rs::PropertyData& obj)
{
  rs::Object object;

  object.setTrackId(std::stoi(obj["rs.annotation.Tracking"]["trackingID"].value()));

  for(size_t i = 0; i < obj["rs.annotation.SemanticColor"].size("color"); i++)
  {
    object.setColor(obj["rs.annotation.SemanticColor"].at("color",i).value(),
                    std::stof(obj["rs.annotation.SemanticColor"].at("ratio", i).value()));
  }

  for(size_t i = 0; i < obj["rs.annotation.Shape"].size("shape"); i++)
  {
    object.setShape(obj["rs.annotation.Shape"].at("shape",i).value(),
                    std::stof(obj["rs.annotation.Shape"].at("confidence",i).value()));
  }

  object.setSize(obj["rs.annotation.SemanticSize"]["size"].value(),
                 std::stof(obj["rs.annotation.SemanticSize"]["confidence"].value()));

  object.set3DPose(obj["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]);
  object.setScale(obj["rs.annotation.Geometry"]["boundingBox"]["rs.pcl.BoundingBox3D"]);

  object.setTimestamp(std::stof(obj["timestamp"].value()));

  return std::move(object);
}

ros::Publisher* pub;
ros::Publisher* click_pub;
ros::Publisher* click_pose_pub;
OntologyManipulator* onto_ = nullptr;
std::mutex mut_;
std::vector<rs::Object> objects_;
std::vector<rs::Object> objects_prev_;

std::vector<rs::Object> merge(std::vector<rs::Object>& current, std::vector<rs::Object>& prev, int type)
{
  std::vector<rs::Object> res;

  for(size_t i = 0; i < current.size();)
  {
    int prev_index = -1;
    float max_similarity = 0.4;
    for(size_t j = 0; j < prev.size(); j++)
    {
      float sim = 0;
      if(type == 0)
        sim = (2*current[i].poseSimilarity(prev[j]) + current[i].scaleSimilarity(prev[j])) / 3.0;
      else
        sim = current[i].scaleSimilarity(prev[j]);

      if(sim > max_similarity)
      {
        max_similarity = sim;
        prev_index = j;
      }
    }

    if(prev_index != -1)
    {
      std::cout << "match " << type << " " << current[i].getName() << " with " << prev[prev_index].getName() << " " << max_similarity << std::endl;
      current[i].merge(prev[prev_index]);
      res.push_back(std::move(current[i]));
      prev.erase(prev.begin() + prev_index);
      current.erase(current.begin() + i);
    }
    else
      i++;
  }

  return std::move(res);
}

void Callback(const robosherlock_msgs::RSObjectDescriptionsConstPtr& msg, ros::Publisher world_publisher)
{

  rs::DataReader reader;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  uwds3_msgs::WorldStamped robWorld_stamped;
  uwds3_msgs::World robWorld;

  geometry_msgs::TransformStamped mainTransform;
  geometry_msgs::PoseStamped poseIntoMap;
  geometry_msgs::PoseStamped poseIntoBaseFootprint;

  std::cout << msg->obj_descriptions.size() << std::endl;

  std::vector<rs::PropertyData> datas;

  for(auto obj : msg->obj_descriptions)
    datas.push_back(reader.get(obj));

  /*for(auto obj : datas)
    obj.print();*/

  objects_prev_ = std::move(objects_);
  std::vector<rs::Object> objects;
  for(auto obj : datas)
  {
    auto tmp = getObject(obj);
    if(tmp.smallerThan(0.5))
      objects.push_back(tmp);
  }

  std::vector<rs::Object> objects_poses = merge(objects, objects_prev_, 0);
  std::vector<rs::Object> objects_scales = merge(objects, objects_prev_, 1);

  objects.insert(objects.end(), objects_poses.begin(), objects_poses.end());
  objects.insert(objects.end(), objects_scales.begin(), objects_scales.end());

  for(auto& obj : objects)
  {
    obj.setId();
    obj.upadteInOntology(onto_);
    
    // Create scene node from objects and add them to the world
    uwds3_msgs::SceneNode tmp_scene;
    visualization_msgs::Marker marker;
 
    tmp_scene.id = std::to_string(obj.getId());
    tmp_scene.label = obj.getName();

    ROS_ERROR_STREAM("ID :" << tmp_scene.id);

    tmp_scene.type = tmp_scene.OBJECT;
    tmp_scene.is_perceived = true;
    tmp_scene.is_located = true;


    marker = obj.getMarker();
    
    // pose
    //tmp_scene.pose_stamped.pose.pose = marker.pose;
    //tmp_scene.pose_stamped.header = marker.header;

    poseIntoBaseFootprint.pose = marker.pose;
    poseIntoBaseFootprint.header = marker.header;

    try
    {
    mainTransform = tfBuffer.lookupTransform("map","base_footprint",ros::Time(0),ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex)
    {
    ROS_WARN("%s",ex.what());
    return;
    }

    tf2::doTransform(poseIntoBaseFootprint,poseIntoMap,mainTransform);

    tmp_scene.pose_stamped.pose.pose = poseIntoMap.pose;
    tmp_scene.pose_stamped.header = poseIntoMap.header;

    // shape
    tmp_scene.has_shape = true;
    tmp_scene.shapes.resize(1);
    if(marker.type == visualization_msgs::Marker::CUBE)
    {
      
      tmp_scene.shapes[0].type = tmp_scene.shapes[0].BOX;
      tmp_scene.shapes[0].dimensions.resize(3);  
      tmp_scene.shapes[0].dimensions[0]= marker.scale.x; 
      tmp_scene.shapes[0].dimensions[1]= marker.scale.y; 
      tmp_scene.shapes[0].dimensions[2]= marker.scale.z; 
    }
    else if (marker.type == visualization_msgs::Marker::CYLINDER)
    {
      tmp_scene.shapes[0].type = tmp_scene.shapes[0].CYLINDER;
      tmp_scene.shapes[0].dimensions.resize(2);
      tmp_scene.shapes[0].dimensions[0]= marker.scale.x; 
      tmp_scene.shapes[0].dimensions[1]= marker.scale.y; 
    }

    tmp_scene.shapes[0].color.r = marker.color.r;
    tmp_scene.shapes[0].color.g = marker.color.g;
    tmp_scene.shapes[0].color.b = marker.color.b;
    tmp_scene.shapes[0].color.a = marker.color.a;
    tmp_scene.shapes[0].name = obj.getName();


    robWorld.scene.push_back(tmp_scene);
    //pub->publish(marker);
    //pub->publish(obj.getMarkerName());
  }

  // Publish the world 
  robWorld_stamped.world = robWorld;
  robWorld_stamped.header.stamp = ros::Time::now();
  robWorld_stamped.header.frame_id = "map";
  world_publisher.publish(robWorld_stamped);

  mut_.lock();
  objects_ = std::move(objects);
  if(objects_prev_.size())
  {
    std::cout << objects_prev_.size() << " not found" << std::endl;
    for(size_t i = 0; i < objects_prev_.size(); i++)
    {
      if(objects_prev_[i].olderThan(2.0))
        objects_prev_[i].removeFromOntology(onto_);
    }
    objects_.insert(objects_.end(), objects_prev_.begin(), objects_prev_.end());
  }
  mut_.unlock();

}

void clickCallback(const geometry_msgs::PointStamped& msg)
{
  std::string name;
  geometry_msgs::Pose obj_pose;

  float min_size = 100000;

  mut_.lock();
  for(auto& obj : objects_)
  {
    float dist = obj.dist(msg.point.x, msg.point.y, msg.point.z);
    if(dist < min_size)
    {
      name = obj.getName();
      obj_pose = obj.getPose();
      min_size = dist;
    }
  }
  mut_.unlock();

  // Publish name of clicked object
  std_msgs::String res_msg;
  res_msg.data = name;
  click_pub->publish(res_msg);

  // Publish pose of the clicked object
  geometry_msgs::Pose pose;
  pose.position.x = obj_pose.position.x;
  pose.position.y = obj_pose.position.y;
  pose.position.z = obj_pose.position.z;
  pose.orientation.x = obj_pose.orientation.x;
  pose.orientation.y = obj_pose.orientation.y;
  pose.orientation.z = obj_pose.orientation.z;
  pose.orientation.w = obj_pose.orientation.w;
  click_pose_pub->publish(pose);


  #ifdef DEBUG_PRINT
  std::cout << "***** Name of clicked object is [" << name << "]" << std::endl;

  std::cout << "***** Pose of clicked object is :" << std::endl;
  std::cout << "--------- Position --------- " << pose.position.x << std::endl;
  std::cout << "X :" << pose.position.x << std::endl;
  std::cout << "Y :" << pose.position.y << std::endl;
  std::cout << "Z :" << pose.position.z << std::endl;
  std::cout << "--------- Orientation ---------" << std::endl;
  std::cout << "X :" << pose.orientation.x << std::endl;
  std::cout << "Y :" << pose.orientation.y << std::endl;
  std::cout << "Z :" << pose.orientation.z << std::endl;
  std::cout << "W :" << pose.orientation.w << std::endl;
  std::cout << "---------------------" << std::endl;
  #endif
}



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rs_display");
  ros::NodeHandle n;

  OntologiesManipulator ontos(&n);
  ontos.add("robot");
  onto_ = ontos.get("robot");

  ros::Publisher world_pub = n.advertise<uwds3_msgs::WorldStamped>("ar_tracks", 1);

  // Generic subscribe to RoboSherlock_USER/result_advertiser
  ros::Subscriber sub = n.subscribe<robosherlock_msgs::RSObjectDescriptions>(std::string("RoboSherlock_") + std::string(getenv("USER"))+"/result_advertiser", 1000, boost::bind(&Callback, _1, world_pub));

  // Susbscribe to topic given by Rviz when "Publish Point" tool is used on an object
  ros::Subscriber click_sub = n.subscribe("/clicked_point", 1000, clickCallback);

  // Give Rviz marker to show the object as colored boxes
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pub = &marker_pub;

  // Publish name of the clicked object in rviz to topic /clicked_object
  ros::Publisher c_pub = n.advertise<std_msgs::String>("clicked_object", 10);
  click_pub = &c_pub;

  // Publish pose of the clicked object in rviz to topic /clicked_object_pose
  ros::Publisher c_pose_pub = n.advertise<geometry_msgs::Pose>("clicked_object_pose", 10);
  click_pose_pub = &c_pose_pub;

  std::cout << "**** RS Display Init Done" << std::endl;

  ros::spin();

  return 0;
}
