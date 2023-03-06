#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkCommand.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <memory> //for shared_ptr

// forward declaration for internals
class vtkMRMLROS2ParameterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ParameterNode : public vtkMRMLNode {
    // friend declarations
    friend class vtkMRMLROS2ParameterInternals;
    friend class vtkMRMLROS2NodeNode;

   protected:
    vtkMRMLROS2ParameterNode(void);
    ~vtkMRMLROS2ParameterNode(void);

   public:

    enum Events
    {
      ParameterModifiedEvent = vtkCommand::UserEvent + 54
    };

    vtkTypeMacro(vtkMRMLROS2ParameterNode, vtkMRMLNode);
    typedef vtkMRMLROS2ParameterNode SelfType;
    static SelfType* New(void);
    void PrintSelf(ostream& os, vtkIndent indent) override;
    vtkMRMLNode* CreateNodeInstance(void) override;
    const char* GetNodeTagName(void) override;

    bool AddToROS2Node(const char* nodeId, const std::string& monitoredNodeName);

    /*! Get the name of the node holding the parameters we're looking for. */
    inline const std::string & GetNodeName(void) const { // todo:: MonitoredNode
      return mMonitoredNodeName;
    }

    bool IsAddedToROS2Node(void) const;
    bool IsMonitoredNodeReady(void) const; 

    /* Add a node and parameter to monitor */
    bool AddParameter(const std::string& parameterName); 
    /* Remove a parameter that is being monitored. If no parameters are being monitored for a node, stop monitoring the node as well*/
    bool RemoveParameter(const std::string& parameterName); 

    bool IsParameterSet(const std::string& parameterName) const; 

    /* Returns data type if the parameter is monitored. Else it returns an empty string */
    bool GetParameterType(const std::string& parameterName, std::string& result); 
    /* convenience methods for users to return output, mostly for Python users */
    std::string GetParameterType(const std::string& parameterName); 


    /*!  Prints value of a monitored parameter after converting it to a string. Returns empty string if value not set */
   std::string PrintParameter(const std::string& parameterName); 

// todo: in RbotoNode, make sure we check with IsParameterSet and GetParameterType 

    /* Returns true if it is a boolean and it is set. Users should always make sure that the key exists
    and the parameter type is boolean before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsBool(const std::string& parameterName, bool& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    bool GetParameterAsBool(const std::string& parameterName);

    /* Returns true if it is a Integer and it is set. Users should always make sure that the key exists
    and the parameter type is Integer before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsInteger(const std::string& parameterName, int& result);
    /* convenience methods for users to return output, mostly for Python users */
    int GetParameterAsInteger(const std::string& parameterName);

    /* Returns true if it is a Double and it is set. Users should always make sure that the key exists
    and the parameter type is Double before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsDouble(const std::string& parameterName, double& result);
    /* convenience methods for users to return output, mostly for Python users */
    double GetParameterAsDouble(const std::string& parameterName);

    /* Returns true if it is a String and it is set. Users should always make sure that the key exists
    and the parameter type is String before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsString(const std::string& parameterName, std::string& result);
    /* convenience methods for users to return output, mostly for Python users */
    std::string GetParameterAsString(const std::string& parameterName);

    /* Returns true if it is a Vector of Bools and it is set. Users should always make sure that the key exists
    and the parameter type is Vector of Bools before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfBools(const std::string& parameterName, std::vector<char>& result);
    /* convenience methods for users to return output, mostly for Python users */
    std::vector<char> GetParameterAsVectorOfBools(const std::string& parameterName);

    /* Returns true if it is a Vector of Ints and it is set. Users should always make sure that the key exists
    and the parameter type is Vector of Ints before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfIntegers(const std::string& parameterName, std::vector<int>& result);
    /* convenience methods for users to return output, mostly for Python users */
    std::vector<int> GetParameterAsVectorOfIntegers(const std::string& parameterName);

    /* Returns true if it is a Vector of Doubles and it is set. Users should always make sure that the key exists
    and the parameter type is Vector of Doubles before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfDoubles(const std::string& parameterName, std::vector<double>& result);
    /* convenience methods for users to return output, mostly for Python users */
    std::vector<double> GetParameterAsVectorOfDoubles(const std::string& parameterName);

    /* Returns true if it is a Vector of Strings and it is set. Users should always make sure that the key exists
    and the parameter type is Vector of Strings before calling this method
    Main method, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfStrings(const std::string& parameterName, std::vector<std::string>& result);
    /* convenience methods for users to return output, mostly for Python users */
    std::vector<std::string> GetParameterAsVectorOfStrings(const std::string& parameterName);

    virtual void ParameterSet(void)
    {
      this->InvokeCustomModifiedEvent(vtkMRMLROS2ParameterNode::ParameterModifiedEvent);
    }

    // Save and load
    virtual void ReadXMLAttributes(const char** atts) override;
    virtual void WriteXML(std::ostream& of, int indent) override;
    void UpdateScene(vtkMRMLScene* scene) override;

   protected:

    bool SetupParameterEventSubscriber(void); 
    bool CheckParameterExistsAndIsSet(const std::string &parameterName) const;
    void SetMonitoredParameterNamesCache(const std::vector<std::string>& MonitoredParameterNamesCache); 


   protected:

    std::shared_ptr<vtkMRMLROS2ParameterInternals> mInternals = nullptr; // todo-address : changed to shared_ptr

    std::string mMRMLNodeName = "ros2:param:undefined";
    std::string mMonitoredNodeName = "undefined";
    bool mIsInitialized = false;

    // For ReadXMLAttributes
    vtkGetMacro(mMRMLNodeName, std::string);
    vtkSetMacro(mMRMLNodeName, std::string);
    vtkGetMacro(mMonitoredNodeName, std::string);
    vtkSetMacro(mMonitoredNodeName, std::string);

    private:
    // vector to store all parameter names that are monitored by the node. This is used for saving and reloading state.
    std::vector<std::string> MonitoredParameterNamesCache = {};
    std::vector<std::string> GetMonitoredParameterNamesCache();

};

#endif  // __vtkMRMLROS2ParameterNode_h

// ros2 = slicer.mrmlScene.GetFirstNodeByName('ros2:node:test_node')
