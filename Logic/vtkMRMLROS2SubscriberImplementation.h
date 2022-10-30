#ifndef __vtkMRMLROS2SubscriberImplementation_h
#define __vtkMRMLROS2SubscriberImplementation_h

#include <vtkMRMLROS2SubscriberNode.h>
#include "vtkSlicerRos2ModuleLogicExport.h"
#include <vtkROS2ToSlicer.h>

template <typename _ros_type, typename _slicer_type>
class  VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberImplementation: public vtkMRMLROS2SubscriberNode
{
   private:
      _ros_type mLastMessage;
      std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription;

   public:
     typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> ThisType;
//     static ThisType *New(); // vtkObject - Create an object with Debug turned off, modified time initialized to zero, and reference counting on.
     virtual vtkMRMLNode* CreateNodeInstance() override { return new ThisType(); };//ThisType::New(); }; //Create instance of the default node. Like New only virtual. -> Explanation from doxygen

     void SetSubscriber(std::shared_ptr<rclcpp::Node> nodePointer) {
       mSubscription= nodePointer->create_subscription<_ros_type>(mTopic, 10000, std::bind(&ThisType::SubscriberCallback, this, std::placeholders::_1));
     }

     void GetLastMessage(_slicer_type & result) const {
      // maybe add some check that we actually received a message?
       vtkROS2ToSlicer(mLastMessage, result);
     }


      // this is the ROS callback when creating
     void SubscriberCallback(const _ros_type & message) {
       mLastMessage = message;
//       std::cerr << GetLastMessageYAML() << std::endl; // useful for debugging
       this->Modified(); // or whatever vtk uses to indicates the node has been modified
     }

     std::string GetLastMessageYAML(void) const override {
       std::stringstream out;
       rosidl_generator_traits::to_yaml(mLastMessage, out);
       return out.str(); //.to_yaml();   // I think this exists, not totally sure
     }
};

#endif