#ifndef __vtkMRMLROS2NODENode_h
#define __vtkMRMLROS2NODENode_h

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

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2NODENode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type> friend class vtkMRMLROS2SubscriberTemplatedInternals;
  template <typename _slicer_type, typename _ros_type> friend class vtkMRMLROS2PublisherTemplatedInternals;
  friend class vtkMRMLROS2ParameterInternals;
  friend class vtkMRMLROS2Tf2BufferNode;
  friend class vtkMRMLROS2ParameterNode; // allowed?
  friend class vtkMRMLROS2Tf2BroadcasterNode;

 public:
  typedef vtkMRMLROS2NODENode SelfType;
  vtkTypeMacro(vtkMRMLROS2NODENode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  /*! Calls rclcpp::init if needed and then create the internal ROS
    node. */
  void Create(const std::string & nodeName);
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

  void Spin(void);

  vtkMRMLROS2SubscriberNode* GetSubscriberNodeByTopic(const std::string & topic);
  vtkMRMLROS2PublisherNode* GetPublisherNodeByTopic(const std::string & topic);
  vtkMRMLROS2ParameterNode* GetParameterNodeByNode(const std::string & node);
  vtkMRMLROS2Tf2BroadcasterNode* GetBroadcasterByID(const std::string & nodeID);
  vtkMRMLROS2Tf2BufferNode* GetBuffer(void);

  // void AddBuffer(vtkMRMLROS2Tf2BufferNode * node); // if buffer has lookup - add it to the node (otherwise don't)

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2NODENode();
  ~vtkMRMLROS2NODENode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> mBuffer; // enforce a single buffer per node - if using tf on that node we know we need a buffer - if not don't use it
  
  std::string mMRMLNodeName = "ros2:node:undefined";
  std::string mROS2NodeName = "undefined";

  std::vector<vtkMRMLROS2ParameterNode* > mParameterNodes;

  // For ReadXMLAttributes
  inline void SetROS2NodeName(const std::string & name) {
    mROS2NodeName = name;
  }
};

#endif // __vtkMRMLROS2NODENode_h
