#include "rs_display/Object.h"

#include "ros/ros.h"

namespace rs
{

IdManager<size_t> Object::id_manager_;

Object::Object()
{
  id_ = -1;
  shape_confidence_ = 0;
  size_confidence_ = 0;
  opacity_ = 1.0;
  nb_seen_ = 0;
}

void Object::setId()
{
  if(id_ == -1)
  {
    id_ = id_manager_.getNewId();
    std::cout << "set " << id_ << std::endl;
  }
  name_ = "obj_" + std::to_string(id_);
}

visualization_msgs::Marker Object::getMarker()
{
  visualization_msgs::Marker marker;

  if(name_ == "")
  {
    std::cout << "no name. id = " << id_ << std::endl;
    return marker;
  }

  marker.header.frame_id = frame_;
  marker.header.stamp = ros::Time(timestamp_);
  marker.ns = "rs";
  marker.id = id_;
  if(shape_ == "box")
    marker.type = visualization_msgs::Marker::CUBE;
  else
    marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose_[0];
  marker.pose.position.y = pose_[1];
  marker.pose.position.z = pose_[2];

  marker.pose.orientation.x = orientation_[0];
  marker.pose.orientation.y = orientation_[1];
  marker.pose.orientation.z = orientation_[2];
  marker.pose.orientation.w = orientation_[3];

  marker.scale.x = scale_[0];
  marker.scale.y = scale_[1];
  marker.scale.z = scale_[2];

  auto color = toColor(colors_[0].first);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = opacity_;

  marker.text = name_;
  marker.lifetime = ros::Duration(5);

  return marker;
}

geometry_msgs::Pose Object::getPose()
{
  geometry_msgs::Pose pose; 

  pose.position.x = pose_[0];
  pose.position.y = pose_[1];
  pose.position.z = pose_[2];

  pose.orientation.x = orientation_[0];
  pose.orientation.y = orientation_[1];
  pose.orientation.z = orientation_[2];
  pose.orientation.w = orientation_[3];

  return pose;
}

visualization_msgs::Marker Object::getMarkerName()
{
  visualization_msgs::Marker marker;

  if(name_ == "")
  {
    std::cout << "no name. id = " << id_ << std::endl;
    return marker;
  }

  marker.header.frame_id = frame_;
  marker.header.stamp = ros::Time(timestamp_);
  marker.ns = "rs_name";
  marker.id = id_;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose_[0];
  marker.pose.position.y = pose_[1];
  marker.pose.position.z = pose_[2] + scale_[2];

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.text = name_;
  marker.lifetime = ros::Duration(5);

  return marker;
}

bool compareColors(std::pair<std::string, float> c1, std::pair<std::string, float> c2)
{
    return (c1.second > c2.second);
}

void Object::setColor(const std::string& color, float ratio)
{
  int index = -1;
  for(size_t i = 0; i < colors_.size(); i++)
    if(colors_[i].first == color)
      index = i;

  if(index == -1)
    colors_.push_back(std::pair<std::string, float>(color, ratio));
  else
    colors_[index].second = ratio;

  std::sort(colors_.begin(), colors_.end(), compareColors);
}

void Object::setShape(const std::string& shape, float confidence)
{
  if(shape == "flat")
    return;

  if(confidence > shape_confidence_)
  {
    shape_ = shape;
    shape_confidence_ = confidence;
  }
}

void Object::setSize(const std::string& size, float confidence)
{
  if(confidence > size_confidence_)
  {
    size_ = size;
    size_confidence_ = confidence;
  }
}

void Object::set3DPose(PropertyData data)
{
  frame_ = data["frame"].value();

  auto pose = toVect(data["translation"].value());
  pose_[0] = pose[0];
  pose_[1] = pose[1];
  pose_[2] = pose[2];

  auto rot = toVect(data["rotation"].value());
  orientation_[0] = rot[0];
  orientation_[1] = rot[1];
  orientation_[2] = rot[2];
  orientation_[3] = rot[3];
}

void Object::setScale(PropertyData data)
{
  scale_[0] = std::stof(data["width"].value());
  scale_[1] = std::stof(data["height"].value());
  scale_[2] = std::stof(data["depth"].value());
}

void Object::upadteInOntology(OntologyManipulator* onto)
{
  nb_seen_++;
  if(nb_seen_ < 3)
    return;
  
  if(onto == nullptr)
    return;

  if(!onto->individuals.exist(name_))
    onto->feeder.addInheritage(name_, "Object");

  addInOntology(onto, name_, "hasShape", shape_);
  addInOntology(onto, name_, "hasSize", size_);

  std::vector<std::string> colors;
  for(auto& color : colors_){
    if (color.second >= 0.3){  
      colors.push_back(color.first);
    }
  }
  addInOntology(onto, name_, "hasColor", colors);
}

void Object::removeFromOntology(OntologyManipulator* onto)
{
  nb_seen_ = 0;
  if(onto == nullptr)
    return;

  onto->feeder.removeConcept(name_);
}

void Object::resetPose()
{
  pose_[0] = 0;
  pose_[1] = 0;
  pose_[2] = 0;
}

float Object::dist(float x, float y, float z)
{
  return ((x - pose_[0]) * (x - pose_[0]) +
          (y - pose_[1]) * (y - pose_[1]) +
          (z - pose_[2]) * (z - pose_[2]));
}

bool Object::smallerThan(float size)
{
  if((scale_[0] > size) || (scale_[1] > size) || (scale_[2] > size))
    return false;
  else
    return true;
}

bool Object::olderThan(float duration)
{
  if((ros::Time::now() - ros::Duration(duration)).toSec() > timestamp_)
    return true;
  else
    return false;
}

void Object::merge(const Object& other)
{
  id_ = other.id_;

  pose_[0] = (pose_[0] + other.pose_[0]) / 2.0;
  pose_[1] = (pose_[1] + other.pose_[1]) / 2.0;
  pose_[2] = (pose_[2] + other.pose_[2]) / 2.0;

  scale_[0] = (scale_[0] + 2*other.scale_[0]) / 3.0;
  scale_[1] = (scale_[1] + 2*other.scale_[1]) / 3.0;
  scale_[2] = (scale_[2] + 2*other.scale_[2]) / 3.0;

  orientation_[0] = other.orientation_[0];
  orientation_[1] = other.orientation_[1];
  orientation_[2] = other.orientation_[2];
  orientation_[3] = other.orientation_[3];

  if(size_confidence_ < other.size_confidence_)
  {
    size_ = other.size_;
    size_confidence_ = other.size_confidence_;
  }

  if(shape_confidence_ < other.shape_confidence_)
  {
    shape_ = other.shape_;
    shape_confidence_ = other.shape_confidence_;
  }

  for(auto color : other.colors_)
  {
    int index = -1;
    for(size_t i = 0; i < colors_.size(); i++)
      if(colors_[i].first == color.first)
        index = i;

    if(index == -1)
      colors_.push_back(color);
    else if(colors_[index].second < color.second)
      colors_[index].second = color.second;
  }

  opacity_ = other.opacity_;
  nb_seen_ = other.nb_seen_;
  std::sort(colors_.begin(), colors_.end(), compareColors);
}

float Object::poseSimilarity(const Object& other)
{
  if((pose_[0] == 0) || (pose_[1] == 0) || (pose_[2] == 0))
    return 0;

  float pose_x = coordSimilarity(pose_[0], other.pose_[0], 0.1);
  float pose_y = coordSimilarity(pose_[1], other.pose_[1], 0.1);
  float pose_z = coordSimilarity(pose_[2], other.pose_[2], 0.1);

  if((pose_x == 0) || (pose_y == 0))
    return 0;
  else
    return (pose_x + pose_y + pose_z) / 3.0;
}

float Object::scaleSimilarity(const Object& other)
{
  float scale_x = coordSimilarity(scale_[0], other.scale_[0], 0.05);
  float scale_y = coordSimilarity(scale_[1], other.scale_[1], 0.05);
  float scale_z = coordSimilarity(scale_[2], other.scale_[2], 0.05);

  if((scale_x == 0) || (scale_y == 0) || (scale_z == 0))
    return 0;
  else
    return (scale_x + scale_y + scale_z) / 3.0;
}

std::array<float, 4> Object::toColor(const std::string& str)
{
  std::array<float, 4> res;

  if(str == "yellow")
    res = {1.0, 1.0, 0, opacity_};
  else if(str == "blue")
    res = {0, 0, 1.0, opacity_};
  else if(str == "black")
    res = {0, 0, 0, opacity_};
  else if(str == "white")
    res = {1.0, 1.0, 1.0, opacity_};
  else if(str == "red")
    res = {1.0, 0, 0, opacity_};
  else if(str == "green")
    res = {0, 1.0, 0, opacity_};
  else if(str == "cyan")
    res = {0, 1.0, 1.0, opacity_};
  else if(str == "magenta")
    res = {1.0, 0, 1.0, opacity_};
  else
    res = {0.5, 0.5, 0.5, opacity_};

  return res;
}

std::vector<float> Object::toVect(const std::string& str)
{
  std::vector<std::string> str_vect;
  std::vector<float> flt_vect;
  split(str, str_vect, ",");
  for(auto it : str_vect)
    flt_vect.push_back(std::stof(it));
  return flt_vect;
}

bool Object::split(const std::string &text, std::vector<std::string> &strs, const std::string& delim)
{
  std::string tmp_text = text;
  while(tmp_text.find(delim) != std::string::npos)
  {
    size_t pos = tmp_text.find(delim);
    std::string part = tmp_text.substr(0, pos);
    tmp_text = tmp_text.substr(pos + delim.size(), tmp_text.size() - pos - delim.size());
    if(part != "")
      strs.push_back(part);
  }
  strs.push_back(tmp_text);
  if(strs.size() > 1)
    return true;
  else
    return false;
}

float Object::opposite(float prev, float current)
{
  if(prev < 0)
  {
    if((prev >= -(current + 0.1)) && (prev <= -(current - 0.1)))
      return -current;
    else
      return current;
  }
  else
  {
    if((prev <= -(current + 0.1)) && (prev >= -(current - 0.1)))
      return -current;
    else
      return current;
  }
}

float Object::coordSimilarity(float s1, float s2, float threshold)
{
  float s_abs = std::abs(s1 - s2);
  if(s_abs > threshold)
    return 0;
  else
    return 1 - (s_abs / threshold);
}

void Object::addInOntology(OntologyManipulator* onto, const std::string& indiv, const std::string& prop, const std::string& on)
{
  std::vector<std::string> onto_res;
  onto_res = onto->individuals.getOn(indiv, prop);
  if(onto_res.size())
  {
    bool exist = false;
    for(auto r : onto_res)
    {
      if(r == on)
        exist = true;
      else
        onto->feeder.removeProperty(indiv, prop, r);
    }

    if(!exist)
      onto->feeder.addProperty(indiv, prop, on);
  }
  else
    onto->feeder.addProperty(indiv, prop, on);
}

void Object::addInOntology(OntologyManipulator* onto, const std::string& indiv, const std::string& prop, const std::vector<std::string>& ons)
{
  std::vector<std::string> onto_res;
  onto_res = onto->individuals.getOn(indiv, prop);
  if(onto_res.size())
  {
    for(size_t i = 0; i < onto_res.size();)
    {
      if(std::find(ons.begin(), ons.end(), onto_res[i]) == ons.end())
      {
        onto->feeder.removeProperty(indiv, prop, onto_res[i]);
        onto_res.erase(onto_res.begin() + i);
      }
      else
        i++;
    }

    for(size_t i = 0; i < ons.size(); i++)
    {
      if(std::find(onto_res.begin(), onto_res.end(), ons[i]) == onto_res.end())
        onto->feeder.addProperty(indiv, prop, ons[i]);
    }
  }
  else
  {
    for(auto& on : ons)
      onto->feeder.addProperty(indiv, prop, on);
  }
}

} // namespace rs
