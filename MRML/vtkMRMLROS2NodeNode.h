#ifndef __vtkMRMLROS2NodeNode_h
#define __vtkMRMLROS2NodeNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declarations
class vtkMatrix4x4;
class vtkMRMLROS2NodeInternals;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2BroadcasterNode;
class vtkMRMLROS2Tf2LookupNode;
class vtkMRMLROS2RobotNode;
class vtkMRMLROS2ServiceNode;
class vtkMRMLROS2ServiceClientNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2NodeNode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type> friend class vtkMRMLROS2SubscriberTemplatedInternals;
  template <typename _slicer_type, typename _ros_type> friend class vtkMRMLROS2PublisherTemplatedInternals;
  friend class vtkMRMLROS2ParameterInternals;
  friend class vtkMRMLROS2ParameterNode;
  friend class vtkMRMLROS2Tf2BroadcasterNode;
  friend class vtkMRMLROS2Tf2LookupNode;
  friend class vtkMRMLROS2RobotNode;
  friend class vtkMRMLROS2ServiceInternals;
  friend class vtkMRMLROS2ServiceNode;
  template <typename _slicer_type_in, typename _slicer_type_out, typename _ros_type> friend class vtkMRMLROS2ServiceClientTemplatedInternals;

 public:
  typedef vtkMRMLROS2NodeNode SelfType;
  vtkTypeMacro(vtkMRMLROS2NodeNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  std::vector<std::string> mRobotNames;

  /*! Calls rclcpp::init if needed and then create the internal ROS
    node. */
  void Create(const std::string & nodeName);
  void Destroy(void); // THIS IS KILLING SLICER
  inline const std::string GetROS2NodeName(void) const {
    return mROS2NodeName;
  }

  /*! List all existing subscribers classes registered
    in the scene. */
  std::string RegisteredROS2SubscriberNodes(void);

  /*! List all existing publishers classes registered
    in the scene. */
  std::string RegisteredROS2PublisherNodes(void);

  /*! Helper method to create a subscriber given a subscriber type and
    a topic. This method will create the corresponding MRML node if
    there is no existing subscriber for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new subscriber was not created. */
  vtkMRMLROS2SubscriberNode * CreateAndAddSubscriberNode(const char * className, const std::string & topic);

  /*! Helper method to create a publisher given a publisher type and
    a topic. This method will create the corresponding MRML node if
    there is no existing publisher for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new publisher was not created. */
  vtkMRMLROS2PublisherNode * CreateAndAddPublisherNode(const char * className, const std::string & topic);

  /*! Helper method to create a parameter node.  You need to provide
    the name of the ROS node that holds the parameters you want to
    monitor. */
  vtkMRMLROS2ParameterNode * CreateAndAddParameterNode(const std::string & monitoredNodeName);

  /*! Helper method to create a tf2 broadcaster.  You need to provide
    the child and parend IDs as they will be broadcasted to tf2. */
  vtkMRMLROS2Tf2BroadcasterNode * CreateAndAddTf2BroadcasterNode(const std::string & parent_id, const std::string & child_id);

  vtkMRMLROS2Tf2LookupNode * CreateAndAddTf2LookupNode(const std::string & parent_id, const std::string & child_id);

  vtkMRMLROS2RobotNode * CreateAndAddRobotNode(const std::string & robotName, const std::string & parameterNodeName, const std::string & parameterName, const std::string & fixedFrame);

  // new
  vtkMRMLROS2ServiceNode * CreateAndAddServiceNode(const std::string & monitoredNodeName);
  vtkMRMLROS2ServiceClientNode * CreateAndAddServiceClientNode(const char * className, const std::string & topic);

  vtkMRMLROS2SubscriberNode * GetSubscriberNodeByTopic(const std::string & topic);
  vtkMRMLROS2PublisherNode * GetPublisherNodeByTopic(const std::string & topic);
  vtkMRMLROS2ParameterNode * GetParameterNodeByNode(const std::string & node);
  vtkMRMLROS2ParameterNode * GetParameterNodeByNodeID(const std::string & nodeID);
  vtkMRMLROS2Tf2BroadcasterNode * GetTf2BroadcasterNodeByID(const std::string & nodeID);
  vtkMRMLROS2Tf2BroadcasterNode * GetTf2BroadcasterNodeByParentChild(const std::string & parent_id, const std::string & child_id);
  vtkMRMLROS2Tf2LookupNode * GetTf2LookupNodeByID(const std::string & nodeID);
  vtkMRMLROS2Tf2LookupNode * GetTf2LookupNodeByParentChild(const std::string & parent_id, const std::string & child_id);
  vtkMRMLROS2RobotNode * GetRobotNodeByName(const std::string & robotName);
  // new
  // vtkMRMLROS2ServiceNode * GetServiceNodeByNode(const std::string & node);
  vtkMRMLROS2ServiceClientNode * GetServiceClientNodeByTopic(const std::string & topic);

  bool RemoveAndDeleteSubscriberNode(const std::string & topic);
  bool RemoveAndDeletePublisherNode(const std::string & topic);
  bool RemoveAndDeleteParameterNode(const std::string & nodeName);
  bool RemoveAndDeleteParameterNodeByNodeID(const std::string & nodeID);
  bool RemoveAndDeleteTf2LookupNode(const std::string & nodeID);
  bool RemoveAndDeleteTf2LookupNode(const std::string & parent_id, const std::string & child_id);
  bool RemoveAndDeleteTf2BroadcasterNode(const std::string & nodeID);
  bool RemoveAndDeleteTf2BroadcasterNode(const std::string & parent_id, const std::string & child_id);
  bool RemoveAndDeleteRobotNode(const std::string & robotName);
  // new
  bool RemoveAndDeleteServiceClientNode(const std::string & topic);

  void Spin(void);
  inline bool GetSpinning(void) const {
    return mSpinning;
  }
  void WarnIfNotSpinning(const std::string & contextMessage) const;

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2NodeNode();
  ~vtkMRMLROS2NodeNode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;

  std::string mMRMLNodeName = "ros2:node:undefined";
  std::string mROS2NodeName = "undefined";

  std::vector<vtkMRMLROS2ParameterNode* > mParameterNodes;
  bool mSpinning = false;

  /*! Creates the tf2 buffer if needed, return true if created. */
  bool SetTf2Buffer(void);
  void SpinTf2Buffer(void);
  vtkSmartPointer<vtkMatrix4x4> mTemporaryMatrix;

  // For ReadXMLAttributes
  inline void SetROS2NodeName(const std::string & name) {
    mROS2NodeName = name;
  }
};

#endif // __vtkMRMLROS2NodeNode_h
