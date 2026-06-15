#include <vtkMRMLROS2PublisherPlanningSceneToolNode.h>
#include <vtkMRMLROS2PublisherInternals.h>
#include <vtkSlicerToROS2.h>

#include <vtkMatrix4x4.h>
#include <vtkMRMLModelNode.h>

#include <moveit_msgs/msg/planning_scene.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

vtkStandardNewMacro(vtkMRMLROS2PublisherPlanningSceneToolNode);

typedef vtkMRMLROS2PublisherNativeInternals<vtkMRMLModelNode*, moveit_msgs::msg::PlanningScene>
vtkMRMLROS2PublisherPlanningSceneToolInternals;

namespace
{
std::string Trim(const std::string& input)
{
  auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) {
    return std::isspace(ch);
  });
  auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) {
    return std::isspace(ch);
  }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

std::vector<std::string> ParseCsv(const char* csv)
{
  std::vector<std::string> values;
  if (!csv) {
    return values;
  }

  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      values.push_back(item);
    }
  }
  return values;
}
}

vtkMRMLROS2PublisherPlanningSceneToolNode::vtkMRMLROS2PublisherPlanningSceneToolNode()
{
  this->AddNodeReferenceRole("source");
  mInternals = new vtkMRMLROS2PublisherPlanningSceneToolInternals(this);
  this->SetQoSReliability(vtkMRMLROS2PublisherNode::Reliable);
}

vtkMRMLROS2PublisherPlanningSceneToolNode::~vtkMRMLROS2PublisherPlanningSceneToolNode()
{
  delete mInternals;
}

vtkMRMLNode * vtkMRMLROS2PublisherPlanningSceneToolNode::CreateNodeInstance(void)
{
  return SelfType::New();
}

const char * vtkMRMLROS2PublisherPlanningSceneToolNode::GetNodeTagName(void)
{
  return "ROS2PublisherPlanningSceneTool";
}

void vtkMRMLROS2PublisherPlanningSceneToolNode::SetSourceNodeID(const char* sourceNodeID)
{
  this->SetAndObserveNodeReferenceID("source", sourceNodeID);
}

const char* vtkMRMLROS2PublisherPlanningSceneToolNode::GetSourceNodeID()
{
  return this->GetNodeReferenceID("source");
}

vtkMRMLModelNode* vtkMRMLROS2PublisherPlanningSceneToolNode::GetSourceNode()
{
  return vtkMRMLModelNode::SafeDownCast(this->GetNodeReference("source"));
}

size_t vtkMRMLROS2PublisherPlanningSceneToolNode::PublishAttach(
    const char* linkName,
    const char* touchLinksCsv,
    const char* subframeName,
    vtkMatrix4x4* subframePose)
{
  return this->PublishAttach(this->GetSourceNode(), linkName, touchLinksCsv, subframeName, subframePose);
}

size_t vtkMRMLROS2PublisherPlanningSceneToolNode::PublishAttach(
    vtkMRMLModelNode* modelNode,
    const char* linkName,
    const char* touchLinksCsv,
    const char* subframeName,
    vtkMatrix4x4* subframePose)
{
  if (!modelNode || !linkName || std::string(linkName).empty() || !this->IsAddedToROS2Node()) {
    return 0;
  }

  auto rosNode = mInternals->GetROSNode();
  if (!rosNode) {
    return 0;
  }

  moveit_msgs::msg::AttachedCollisionObject attachedObject;
  attachedObject.link_name = linkName;
  attachedObject.touch_links = ParseCsv(touchLinksCsv);

  vtkSlicerToROS2(modelNode, attachedObject.object, rosNode);
  attachedObject.object.header.frame_id = linkName;
  attachedObject.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (subframeName && !std::string(subframeName).empty() && subframePose) {
    geometry_msgs::msg::Pose pose;
    vtkSlicerToROS2(subframePose, pose, rosNode);
    attachedObject.object.subframe_names.push_back(subframeName);
    attachedObject.object.subframe_poses.push_back(pose);
  }

  moveit_msgs::msg::PlanningScene planningScene;
  planningScene.is_diff = true;
  planningScene.robot_state.is_diff = true;
  planningScene.robot_state.attached_collision_objects.push_back(attachedObject);

  mNumberOfCalls++;
  const auto justSent = (reinterpret_cast<vtkMRMLROS2PublisherPlanningSceneToolInternals *>(mInternals))->PublishDirect(planningScene);
  mNumberOfMessagesSent += justSent;
  return justSent;
}

size_t vtkMRMLROS2PublisherPlanningSceneToolNode::PublishDetach(vtkMRMLModelNode* modelNode, const char* linkName)
{
  if (!modelNode) {
    return 0;
  }
  return this->PublishDetach(modelNode->GetName(), linkName);
}

size_t vtkMRMLROS2PublisherPlanningSceneToolNode::PublishDetach(const char* objectId, const char* linkName)
{
  if (!objectId || std::string(objectId).empty() || !this->IsAddedToROS2Node()) {
    return 0;
  }

  auto rosNode = mInternals->GetROSNode();
  if (!rosNode) {
    return 0;
  }

  moveit_msgs::msg::AttachedCollisionObject attachedObject;
  if (linkName) {
    attachedObject.link_name = linkName;
    attachedObject.object.header.frame_id = linkName;
  }
  attachedObject.object.header.stamp = rosNode->get_clock()->now();
  attachedObject.object.id = objectId;
  attachedObject.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene planningScene;
  planningScene.is_diff = true;
  planningScene.robot_state.is_diff = true;
  planningScene.robot_state.attached_collision_objects.push_back(attachedObject);

  mNumberOfCalls++;
  const auto justSent = (reinterpret_cast<vtkMRMLROS2PublisherPlanningSceneToolInternals *>(mInternals))->PublishDirect(planningScene);
  mNumberOfMessagesSent += justSent;
  return justSent;
}
