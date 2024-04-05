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
  virtual const char * GetSlicerTypeIn(void) const = 0;
  virtual const char * GetSlicerTypeOut(void) const = 0;
protected:
  vtkMRMLROS2ServiceClientNode * mMRMLNode;
  std::shared_ptr<rclcpp::Node> mROSNode = nullptr;
};



template <typename _slicer_type_in, typename _slicer_type_out, typename _ros_type>
class vtkMRMLROS2ServiceClientTemplatedInternals: public vtkMRMLROS2ServiceClientInternals
{
public:
  typedef vtkMRMLROS2ServiceClientTemplatedInternals<_slicer_type_in, _slicer_type_out, _ros_type> SelfType;

  vtkMRMLROS2ServiceClientTemplatedInternals(vtkMRMLROS2ServiceClientNode *  mrmlNode):
    vtkMRMLROS2ServiceClientInternals(mrmlNode)
  {}

protected:
  std::shared_ptr<rclcpp::Client<_ros_type>> mServiceClient = nullptr;
  std::shared_future<std::shared_ptr<typename _ros_type::Response>> mServiceResponseFuture;
  bool isRequestInProgress = false;

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

  const char * GetSlicerTypeIn(void) const override
  {
    return typeid(_slicer_type_in).name();
  }

  const char * GetSlicerTypeOut(void) const override
  {
    return typeid(_slicer_type_out).name();
  }

  
};


template <typename _slicer_type_in, typename _slicer_type_out, typename _ros_type>
class vtkMRMLROS2ServiceClientVTKInternals:
  public vtkMRMLROS2ServiceClientTemplatedInternals< _slicer_type_in, _slicer_type_out, _ros_type>
{
public:
  typedef vtkMRMLROS2ServiceClientTemplatedInternals< _slicer_type_in, _slicer_type_out, _ros_type> BaseType;

  vtkMRMLROS2ServiceClientVTKInternals(vtkMRMLROS2ServiceClientNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type_out>();
  }

  vtkSmartPointer<_slicer_type_out> mLastMessageSlicer;
  bool lastResponseSuccess = false;

    bool PreRequestCheck(void) const
    {
      return (this->mServiceClient != nullptr);
    }

    bool GetLastResponseStatus()
    {
      // if no request is pending and the last response was successful
      return !this->isRequestInProgress && this->lastResponseSuccess;
    }

    _slicer_type_out * GetLastResponse()
    {
      // todo maybe add some check that we actually received a message?
      return mLastMessageSlicer.GetPointer(); 
    }

    void ServiceCallback(typename rclcpp::Client<_ros_type>::SharedFuture future) {

      
      std::shared_ptr<typename _ros_type::Response> service_response_ = future.get();    
      this->isRequestInProgress = false;
      // vtkSmartPointer<_slicer_type_out> response = vtkNew<_slicer_type_out>();
      vtkROS2ToSlicer(*service_response_, this->mLastMessageSlicer);
      std::cerr << "ServiceNode::ProcessResponse: Value stored in the table + received response: "<< std::endl;

      this->lastResponseSuccess = true; // TODO: implement error handling

      // mLastMessageSlicer = response;

      //           << responseTable->GetValueByName(0, "message") << std::endl;

      // this->lastResponse = response;
      // this->lastResponseTable = responseTable;
    }

    size_t SendAsyncRequest(_slicer_type_in * message)
    {

      auto request = std::make_shared<typename _ros_type::Request>();
      vtkSlicerToROS2(message, *request, this->mROSNode);
      std::cerr << "ServiceNode::SendAsyncRequest sending request" << std::endl;
      this->isRequestInProgress = true;
      this->mServiceResponseFuture = this->mServiceClient->async_send_request(request, std::bind(&vtkMRMLROS2ServiceClientVTKInternals<_slicer_type_in, _slicer_type_out, _ros_type>::ServiceCallback, this, std::placeholders::_1));

      // std::cout << "Requested message" << message << std::endl;
      // float x = 2;
      // float y = 2;
      // float theta = 0.15; 
      // std::string name = "TestSlicerServiceClient";

      // auto request = std::make_shared<typename _ros_type::Request>();
      // request->x = x;
      // request->y = y;
      // request->theta = theta;
      // request->name = name;

      // auto result = this->mServiceClient->async_send_request(request);
      // Handle response in a callback or wait for the response

      // Write a simple python server and trigger it (switch bool response and increment the message response)
      // Look at subscriber callback on how to handle it
      // Maybe create a vtkSlicerROS2 fn to convert std_srv callback 

      // put the respones also in a vtkTable
      // python should be able to show the vtkTable

      // write a test

      return 1;
    }


};

#endif // __vtkMRMLROS2ServiceClientInternals_h
