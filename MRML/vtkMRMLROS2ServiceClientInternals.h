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
                             const std::string & service, std::string & errorMessage) = 0;
  virtual bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                                  const std::string & service, std::string & errorMessage) = 0;
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
                     const std::string & service, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!mrmlROSNodePtr) return false;

    vtkMRMLROS2ServiceClientNode * pub = mrmlROSNodePtr->GetServiceClientNodeByService(service);
    if ((pub != nullptr)
        && pub->IsAddedToROS2Node()) {
      errorMessage = "there is already a service_client for service \"" + service + "\" added to the ROS node";
      return false;
    }
    mROSNode = mrmlROSNodePtr->mInternals->mNodePointer;
    mServiceClient = mROSNode->create_client<_ros_type>(service);
    mrmlROSNodePtr->SetNthNodeReferenceID("service_client",
                                          mrmlROSNodePtr->GetNumberOfNodeReferences("service_client"),
                                          mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    mrmlROSNodePtr->WarnIfNotSpinning("adding service_client for \"" + service + "\"");
    return true;
  }

  bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                          const std::string & service, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!rosNodePtr) return false;

    vtkMRMLROS2ServiceClientNode * pub = rosNodePtr->GetServiceClientNodeByService(service);
    if (pub == nullptr || !pub->IsAddedToROS2Node()) {
      errorMessage = "there isn't a service_client for service \"" + service + "\" which can be deleted from the ROS node";
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

protected:
  vtkSmartPointer<_slicer_type_out> mLastMessageSlicer;
  bool lastResponseSuccess = false;
  bool isRequestInProgress = false;
  std::chrono::steady_clock::time_point lastRequestTime;
  std::chrono::duration<double> mConnectionTimeout;
  std::chrono::duration<double> mRequestTimeout;


public:
  bool PreRequestCheck(void) const
  {
    // return (this->mServiceClient != nullptr);
    if(!this->mServiceClient)
      {
        std::cerr << "ServiceNode::PreRequestCheck: ServiceClient is not initialized" << std::endl;
        return false;
      }
    if(!this->IsServerConnected()){
      std::cerr << "ServiceNode::PreRequestCheck: Server is not connected" << std::endl;
      return false;
    }
    if (this->isRequestInProgress)
      {
        std::cerr << "ServiceNode::PreRequestCheck: Request is already in progress" << std::endl;
        return false;
      }
    return true;
  }

  bool IsServerConnected(void) const
  {
    return this->mServiceClient && this->mServiceClient->service_is_ready();
  }


  bool GetLastResponseStatus(void) const
  {
    // if no request is pending and the last response was successful
    return !this->isRequestInProgress && this->lastResponseSuccess;
  }

  _slicer_type_out* GetLastResponse(void)
  {
    if (!this->lastResponseSuccess) {
      std::cerr<< "GetLastResponse: No valid response available" << std::endl;
      return nullptr;
    }
    return mLastMessageSlicer.GetPointer();
  }

  void ServiceCallback(typename rclcpp::Client<_ros_type>::SharedFuture future)
  {
    try {
      std::shared_ptr<typename _ros_type::Response> service_response_ = future.get();
      this->isRequestInProgress = false;
      vtkROS2ToSlicer(*service_response_, this->mLastMessageSlicer);
      std::cerr << "ServiceNode ServiceCallback has received response"<< std::endl;
      this->lastResponseSuccess = true;
    }
    catch (const std::exception& e) {
      std::cerr << "ServiceNode::ServiceCallback: Exception: " << e.what() << std::endl;
      this->lastResponseSuccess = false;
    }
    this->isRequestInProgress = false;
  }

  size_t SendAsyncRequest(_slicer_type_in* message)
  {
    if (!this->PreRequestCheck()) {
      return 0;
    }

    auto request = std::make_shared<typename _ros_type::Request>();
    vtkSlicerToROS2(message, *request, this->mROSNode);
    // vtkDebugMacro("ServiceNode::SendAsyncRequest sending request");

    this->isRequestInProgress = true;
    this->lastRequestTime = std::chrono::steady_clock::now();

    try {
      this->mServiceResponseFuture
	= this->mServiceClient->async_send_request(
						   request,
						   std::bind(&vtkMRMLROS2ServiceClientVTKInternals<_slicer_type_in, _slicer_type_out, _ros_type>::ServiceCallback,
							     this,
							     std::placeholders::_1)).future;
      return 1;
    }
    catch (const std::exception& e) {
      std::cerr << "Error sending async request: " << e.what() << std::endl;
      this->isRequestInProgress = false;
      return 0;
    }
  }

  // Not using these functions yet

  bool WaitForServer(const std::chrono::duration<double>& timeout = std::chrono::seconds(5))
  {
    if (!this->mServiceClient) {
      std::cerr << "Service client is not initialized" << std::endl;
      return false;
    }
    return this->mServiceClient->wait_for_service(timeout);
  }

  void SetConnectionTimeout(double seconds)
  {
    this->mConnectionTimeout = std::chrono::duration<double>(seconds);
  }

  void SetRequestTimeout(double seconds)
  {
    this->mRequestTimeout = std::chrono::duration<double>(seconds);
  }

  bool IsRequestTimedOut(void) const
  {
    if (!this->isRequestInProgress) {
      return false;
    }
    auto elapsed = std::chrono::steady_clock::now() - this->lastRequestTime;
    return elapsed > this->mRequestTimeout;
  }

  void CancelCurrentRequest(void)
  {
    if (this->isRequestInProgress && this->mServiceResponseFuture.valid()) {
      this->mServiceResponseFuture.cancel();
      this->isRequestInProgress = false;
      std::cerr << "ServiceNode::CancelCurrentRequest: Request has been cancelled" << std::endl;
    }
  }


};

#endif // __vtkMRMLROS2ServiceClientInternals_h
