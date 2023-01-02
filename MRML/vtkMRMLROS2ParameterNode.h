#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <utility>
#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2ParameterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ParameterNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2ParameterInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2ParameterNode, vtkMRMLNode);

  //newly added

  typedef vtkMRMLROS2ParameterNode SelfType;
  typedef std::pair<std::string,std::string> ParameterKey; // pair: {nodeName, parameterName}
  static SelfType * New(void);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  bool AddToROS2Node(const char * nodeId);

  bool IsAddedToROS2Node(void) const;

  /*! Add a node and parameter to monitor */
  bool AddParameter(const std::string &nodeName, const std::string &parameterName);
  /*! Remove a parameter that is being monitored. If no parameters are being monitored for a node, stop monitoring the node as well*/
  bool RemoveParameter(const std::string &nodeName, const std::string &parameterName);

  /*! Main methods, recommended for C++ users since we can check return code and avoid copy for result.
   Returns data type if the parameter is tracked. Else it returns an empty string */
  std::string GetParameterType(const ParameterKey & key, std::string & result);
  /*! convenience methods for users to skip pair creation, mostly for Python users */
  inline std::string GetParameterType(const std::string &nodeName, const std::string &parameterName) {
    std::string result;
    GetParameterType(ParameterKey(nodeName, parameterName), result);
    return result;
  }

  /*! Main methods, recommended for C++ users since we can check return code and avoid copy for result.
   Prints value of a tracked parameter after converting it to a string */
  bool PrintParameterValue(const ParameterKey & key, std::string & result);
  /*! convenience methods for users to skip pair creation, mostly for Python users */
  inline std::string PrintParameterValue(const std::string &nodeName, const std::string &parameterName) {
    std::string result;
    PrintParameterValue(ParameterKey(nodeName, parameterName), result);
    return result;
  }

  /*! Main methods, recommended for C++ users since we can check return code and avoid copy for result. 
  Returns the value of the parameter if it is a string. Users should always make sure that the key exists 
  and the parameter type is string before calling this method*/
  bool GetParameterAsString(const ParameterKey & key, std::string & result);
 /*! convenience methods for users to skip pair creation, mostly for Python users */
  inline std::string GetParameterAsString(const std::string &nodeName, const std::string &parameterName) {
    std::string result;
    GetParameterAsString(ParameterKey(nodeName, parameterName), result);
    return result;
  }

  /*! Returns the value of the parameter if it is an Integer . Users should always make sure the key exists and 
  the parameter type is an integer with GetParameterType before calling this method.  
  Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
  bool GetParameterAsInteger(const ParameterKey & key, int & result);
 /*! convenience methods for users to skip pair creation, mostly for Python users */
  inline int GetParameterAsInteger(const std::string &nodeName, const std::string &parameterName) {
    int result;
    GetParameterAsInteger(ParameterKey(nodeName, parameterName), result);
    return result;
  }

  void listTrackedParameters();

  std::vector<std::string> GetTrackedNodeList();

  std::vector<ParameterKey> GetTrackedNodesAndParametersList();

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2ParameterNode();
  ~vtkMRMLROS2ParameterNode();

  vtkMRMLROS2ParameterInternals * mInternals = nullptr;
  std::string mMRMLNodeName = "ros2:parameterNode";

  // For ReadXMLAttributes
  vtkGetMacro(mMRMLNodeName,std::string);
  vtkSetMacro(mMRMLNodeName,std::string);

};

#endif // __vtkMRMLROS2ParameterNode_h
