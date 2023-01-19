#ifndef __vtkMRMLROS2RobotNode_h
#define __vtkMRMLROS2RobotNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2RobotNode: public vtkMRMLNode
{

 public:
  typedef vtkMRMLROS2RobotNode SelfType;
  vtkTypeMacro(vtkMRMLROS2RobotNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  /*! Calls rclcpp::init if needed and then create the internal ROS
    node. */
  inline const std::string GetROS2RobotName(void) const {
    return mROS2RobotName;
  }

  void SetRobotName(const std::string & robotName);

//   void InitializeRobotDescription();

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

//   std::unique_ptr<vtkMRMLROS2RobotInternals> mInternals;
//   vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> mBuffer; // enforce a single buffer per node - if using tf on that node we know we need a buffer - if not don't use it
  
//   std::string mMRMLNodeName = "ros2:node:undefined";
  std::string mROS2RobotName = "undefined";

//   std::vector<vtkMRMLROS2ParameterNode* > mParameterNodes;

  // For ReadXMLAttributes
  inline void SetROS2RobotName(const std::string & name) {
    mROS2RobotName = name;
  }
};

#endif // __vtkMRMLROS2RobotNode_h
