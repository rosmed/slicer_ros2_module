#ifndef __vtkMRMLROS2SubscriberNode_h
#define __vtkMRMLROS2SubscriberNode_h
// MRML includes
#include <vtkMRML.h>
#include <vtkMRMLNode.h>
#include <vtkMRMLStorageNode.h>
#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>


// VTK includes
#include <vtkObject.h>

// STD includes
#include <string>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include "vtkSlicerRos2ModuleLogicExport.h"



class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberNode : public vtkMRMLNode
{

public:

//  static vtkMRMLROS2SubscriberNode *New(); // vtkObject - Create an object with Debug turned off, modified time initialized to zero, and reference counting on.
  vtkTypeMacro(vtkMRMLROS2SubscriberNode, vtkMRMLNode);

  void PrintSelf(ostream&, vtkIndent) override {};
  inline const char* GetNodeTagName() override { return mTopic.c_str(); };
  virtual vtkMRMLNode* CreateNodeInstance() override { return nullptr; }; //Create instance of the default node. Like New only virtual. -> Explanation from doxygen

  // TODO: look at vtk New operator for templated classes, what is CreateNodeInstance supposed to do - how do they work together (both constructors?) - might CreateNodeInstance be for copy / paste (look at documentation)
  // might have it return a new pointer - try to restore sub from Slicer

  void SetTopic(const std::string & topic);
  virtual std::string GetLastMessageYAML(void) const = 0; // Conclusions - virtual functions are causing new error?? Not being linked properly (location??)


protected:
  //----------------------------------------------------------------
  // Constructor and destroctor
  //----------------------------------------------------------------

  vtkMRMLROS2SubscriberNode();
  ~vtkMRMLROS2SubscriberNode();

  std::string mTopic;

public:
  //----------------------------------------------------------------
  // Connector configuration
  //----------------------------------------------------------------

  virtual void SetSubscriber(std::shared_ptr<rclcpp::Node> mNodePointer) = 0;


private:
  //----------------------------------------------------------------
  // Data
  //----------------------------------------------------------------

};


#endif
