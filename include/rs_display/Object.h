#ifndef RS_OBJECT_H
#define RS_OBJECT_H

#include <string>
#include <vector>
#include <array>

#include <visualization_msgs/Marker.h>

#include "ontoloGenius/utility/OntologiesManipulator.h"
#include "ontoloGenius/utility/OntologyManipulator.h"

#include "rs_display/PropertyData.h"
#include "rs_display/IdManager.h"

namespace rs
{

class Object
{
public:
  Object();

  void setId();

  visualization_msgs::Marker getMarker();
  visualization_msgs::Marker getMarkerName();

  std::string getName() { return name_; }
  geometry_msgs::Pose getPose();

  void setName(const std::string name) { name_ = name; }
  void setTrackId(int id) { track_id_ = id; }
  void setOpacity(float op) { opacity_ = op; }
  void setColor(const std::string& color, float ratio);
  void setShape(const std::string& shape, float confidence);
  void setSize(const std::string& size, float confidence);
  void set3DPose(PropertyData data);
  void setScale(PropertyData data);
  void setTimestamp(float timestamp) { timestamp_ = timestamp; }

  void upadteInOntology(OntologyManipulator* onto);
  void removeFromOntology(OntologyManipulator* onto);

  void resetPose();
  float dist(float x, float y, float z);
  bool smallerThan(float size);
  bool olderThan(float duration);

  void merge(const Object& other);
  float poseSimilarity(const Object& other);
  float scaleSimilarity(const Object& other);

private:
  static IdManager<size_t> id_manager_;

  std::string name_;
  int id_;
  int track_id_;
  std::vector<std::pair<std::string, float>> colors_;
  float opacity_;
  std::string shape_;
  float shape_confidence_;
  std::string size_;
  float size_confidence_;
  std::array<float, 3> pose_;
  std::array<float, 4> orientation_;
  std::array<float, 3> scale_;
  std::string frame_;
  float timestamp_;
  size_t nb_seen_;

  std::array<float, 4> toColor(const std::string& str);
  std::vector<float> toVect(const std::string& str);
  bool split(const std::string &text, std::vector<std::string> &strs, const std::string& delim);
  float opposite(float prev, float current);

  float coordSimilarity(float s1, float s2, float threshold);

  void addInOntology(OntologyManipulator* onto, const std::string& indiv, const std::string& prop, const std::string& on);
  void addInOntology(OntologyManipulator* onto, const std::string& indiv, const std::string& prop, const std::vector<std::string>& ons);
};

} // namespace rs

#endif // RS_OBJECT_H
