#ifndef __vtkMRMLROS2ServiceClientInternals_h
#define __vtkMRMLROS2ServiceClientInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2ServiceClientInternals
{
public:
  vtkMRMLROS2ServiceClientInternals(vtkMRMLROS2ServiceClientNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2ServiceClientInternals() = default;

  virtual bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                             const std::string & topic, std::string & errorMessage) = 0;
  virtual bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                                  const std::string & topic, std::string & errorMessage) = 0;
  virtual bool IsAddedToROS2Node(void) const = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
protected:
  vtkMRMLROS2ServiceClientNode * mMRMLNode;
  std::shared_ptr<rclcpp::Node> mROSNode = nullptr;
};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2ServiceClientTemplatedInternals: public vtkMRMLROS2ServiceClientInternals
{
public:
  typedef vtkMRMLROS2ServiceClientTemplatedInternals<_slicer_type, _ros_type> SelfType;

  vtkMRMLROS2ServiceClientTemplatedInternals(vtkMRMLROS2ServiceClientNode *  mrmlNode):
    vtkMRMLROS2ServiceClientInternals(mrmlNode)
  {}

protected:
  std::shared_ptr<rclcpp::Client<_ros_type>> mServiceClient = nullptr;

  /**
   * Add the ServiceClient to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NodeNode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                     const std::string & topic, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!mrmlROSNodePtr) return false;

    vtkMRMLROS2ServiceClientNode * pub = mrmlROSNodePtr->GetServiceClientNodeByTopic(topic);
    if ((pub != nullptr)
        && pub->IsAddedToROS2Node()) {
      errorMessage = "there is already a service_client for topic \"" + topic + "\" added to the ROS node";
      return false;
    }
    mROSNode = mrmlROSNodePtr->mInternals->mNodePointer;
    mServiceClient = mROSNode->create_client<_ros_type>(topic); 
    mrmlROSNodePtr->SetNthNodeReferenceID("service_client",
                                          mrmlROSNodePtr->GetNumberOfNodeReferences("service_client"),
                                          mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    mrmlROSNodePtr->WarnIfNotSpinning("adding service_client for \"" + topic + "\"");
    return true;
  }

  bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                          const std::string & topic, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!rosNodePtr) return false;

    vtkMRMLROS2ServiceClientNode * pub = rosNodePtr->GetServiceClientNodeByTopic(topic);
    if (pub == nullptr || !pub->IsAddedToROS2Node()) {
      errorMessage = "there isn't a service_client for topic \"" + topic + "\" which can be deleted from the ROS node";
      return false;
    }

    mMRMLNode->SetNodeReferenceID("node", nullptr);
    rosNodePtr->RemoveNthNodeReferenceID("service_client",
                                         rosNodePtr->GetNumberOfNodeReferences("service_client"));

    mServiceClient.reset();
    mROSNode.reset();

    return true;
  }

  bool IsAddedToROS2Node(void) const override
  {
    return (mServiceClient != nullptr);
  }

  const char * GetROSType(void) const override
  {
    return rosidl_generator_traits::name<_ros_type>();
  }

  const char * GetSlicerType(void) const override
  {
    return typeid(_slicer_type).name();
  }
};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2ServiceClientNativeInternals:
  public vtkMRMLROS2ServiceClientTemplatedInternals<_slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2ServiceClientTemplatedInternals<_slicer_type, _ros_type> BaseType;

  vtkMRMLROS2ServiceClientNativeInternals(vtkMRMLROS2ServiceClientNode * mrmlNode):
    BaseType(mrmlNode)
  {}

  _slicer_type mLastMessageSlicer;

  // size_t Publish(const _slicer_type & message)
  // {
  //   const auto nbSubscriber = this->mServiceClient->get_subscription_count();
  //   if (nbSubscriber != 0) {
  //     _ros_type rosMessage;
  //     vtkSlicerToROS2(message, rosMessage, BaseType::mROSNode);
  //     this->mServiceClient->publish(rosMessage);
  //   }
  //   return nbSubscriber;
  // }

  size_t RequestService()
  {
    float x = 2;
    float y = 2;
    float theta = 0.15; 
    std::string name = "Test Name";

    auto request = std::make_shared<_ros_type::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    auto result = this->mServiceClient->async_send_request(request);
    // Handle response in a callback or wait for the response
    return 1;
  }

};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2ServiceClientVTKInternals:
  public vtkMRMLROS2ServiceClientTemplatedInternals< _slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2ServiceClientTemplatedInternals< _slicer_type, _ros_type> BaseType;

  vtkMRMLROS2ServiceClientVTKInternals(vtkMRMLROS2ServiceClientNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

  // size_t Publish(_slicer_type * message)
  // {
  //   const auto nbSubscriber = this->mServiceClient->get_subscription_count();
  //   if (nbSubscriber != 0) {
  //     _ros_type rosMessage;
  //     vtkSlicerToROS2(message, rosMessage, BaseType::mROSNode);
  //     this->mServiceClient->publish(rosMessage);
  //   }
  //   return nbSubscriber;
  // }

    size_t RequestService(_slicer_type * message)
    {
      std::cout << "Requested message" << message << std::endl;
      float x = 2;
      float y = 2;
      float theta = 0.15; 
      std::string name = "TestSlicerServiceClient";

      auto request = std::make_shared<typename _ros_type::Request>();
      request->x = x;
      request->y = y;
      request->theta = theta;
      request->name = name;

      auto result = this->mServiceClient->async_send_request(request);
      // Handle response in a callback or wait for the response
      return 1;
    }


};

#endif // __vtkMRMLROS2ServiceClientInternals_h
