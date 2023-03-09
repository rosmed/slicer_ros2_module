#ifndef __vtkMRMLROS2NodeNode_h
#define __vtkMRMLROS2NodeNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2NodeInternals;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2BroadcasterNode;
class vtkMRMLROS2Tf2BufferNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2NodeNode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type> friend class vtkMRMLROS2SubscriberTemplatedInternals;
  template <typename _slicer_type, typename _ros_type> friend class vtkMRMLROS2PublisherTemplatedInternals;
  friend class vtkMRMLROS2ParameterInternals;
  friend class vtkMRMLROS2Tf2BufferNode;
  friend class vtkMRMLROS2ParameterNode; // allowed?
  friend class vtkMRMLROS2Tf2BroadcasterNode;
  friend class vtkMRMLROS2RobotNode;

 public:
  typedef vtkMRMLROS2NodeNode SelfType;
  vtkTypeMacro(vtkMRMLROS2NodeNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  /*! Calls rclcpp::init if needed and then create the internal ROS
    node. */
  void Create(const std::string & nodeName);
  void Destroy(void); // THIS IS KILLING SLICER
  inline const std::string GetROS2NodeName(void) const {
    return mROS2NodeName;
  }

  /*! Helper method to create a subscriber given a subscriber type and
    a topic. This method will create the corresponding MRML node if
    there is no existing subscriber for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new subscriber was not created. */
  vtkMRMLROS2SubscriberNode * CreateAndAddSubscriber(const char * className, const std::string & topic);
  
  /*! Helper method to create a publisher given a publisher type and
    a topic. This method will create the corresponding MRML node if
    there is no existing publisher for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new publisher was not created. */
  vtkMRMLROS2PublisherNode * CreateAndAddPublisher(const char * className, const std::string & topic);
  
  vtkMRMLROS2ParameterNode * CreateAndAddParameter(const std::string & topic);

  vtkMRMLROS2Tf2BroadcasterNode * CreateAndAddBroadcaster(const char * className, const std::string & parent_id, const std::string & child_id);

  vtkMRMLROS2SubscriberNode * GetSubscriberNodeByTopic(const std::string & topic);
  vtkMRMLROS2PublisherNode * GetPublisherNodeByTopic(const std::string & topic);
  vtkMRMLROS2ParameterNode * GetParameterNodeByNode(const std::string & node);
  bool RemoveSubscriberNode(const std::string & topic);
  bool RemovePublisherNode(const std::string & topic);
  bool RemoveParameterNode(const std::string & nodeName);
  vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> GetTf2BroadcasterByID(const std::string & nodeID);
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> GetTf2Buffer(bool createIfNeeded = true);
  static vtkMRMLROS2NodeNode * CheckROS2NodeExists(vtkMRMLScene * scene, const char* nodeId, std::string & errorMessage);

  void Spin(void);

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2NodeNode();
  ~vtkMRMLROS2NodeNode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> mTf2Buffer; // enforce a single buffer per node - if using tf on that node we know we need a buffer - if not don't use it
  
  std::string mMRMLNodeName = "ros2:node:undefined";
  std::string mROS2NodeName = "undefined";

  std::vector<vtkMRMLROS2ParameterNode* > mParameterNodes;


  // For ReadXMLAttributes
  inline void SetROS2NodeName(const std::string & name) {
    mROS2NodeName = name;
  }
};

#endif // __vtkMRMLROS2NodeNode_h
