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
#include "vtkSlicerRos2ModuleLogicExport.h"



class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberNode : public vtkMRMLNode
{

public:

//  static vtkMRMLROS2SubscriberNode *New();
  vtkTypeMacro(vtkMRMLROS2SubscriberNode, vtkMRMLNode);

  void PrintSelf(ostream&, vtkIndent) override{};
  inline const char* GetNodeTagName() override { return "SlicerROS2Module"; };
  //virtual vtkMRMLNode* CreateNodeInstance() override;
  // TODO: look at vtk New operator for templated classes, what is CreateNodeInstance supposed to do - how do they work together (both constructors?) - might CreateNodeInstance be for copy / paste (look at documentation)
  // might have it return a new pointer - try to restore sub from Slicer

  void SetTopic(const std::string & topic);
  virtual std::string GetLastMessageYAML(void) const = 0;


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

template <typename _ros_type, typename _slicer_type>
class  VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberImplementation: public vtkMRMLROS2SubscriberNode
{
   private:
      _ros_type mLastMessage;
      std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription;

   public:
     typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> ThisType;
     void SetSubscriber(std::shared_ptr<rclcpp::Node> nodePointer){
       mSubscription= nodePointer->create_subscription<_ros_type>(mTopic, 10000, std::bind(&ThisType::SubscriberCallback, this, std::placeholders::_1));
     }

     void GetLastMessage(_slicer_type & result) const {
      // maybe add some check that we actually received a message?
       vtkROS2ToSlicer(mLastMessage, result);
     }


      // this is the ROS callback when creating
     void SubscriberCallback(_ros_type & message) {
       mLastMessage = message;
       this->Modified(); // or whatever vtk uses to indicates the node has been modified
     }

     std::string GetLastMessageYAML(void) const {
       return mLastMessage.to_yaml();   // I think this exists, not totally sure
     }
};
inline void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkMatrix4x4 & result)
{
// do conversion here
}

typedef vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkMatrix4x4> vtkMRMLROS2SubscriberPoseStamped;


#endif
