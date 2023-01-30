#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkCommand.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <deque>
#include <utility>

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

    // newly added

    typedef vtkMRMLROS2ParameterNode SelfType;
    typedef std::pair<std::string, std::string> ParameterKey;  // pair: {nodeName, parameterName}
    static SelfType* New(void);
    void PrintSelf(ostream& os, vtkIndent indent) override;
    vtkMRMLNode* CreateNodeInstance(void) override;
    const char* GetNodeTagName(void) override;
    bool AddToROS2Node(const char* nodeId, const std::string& trackedNodeName);

    bool SetupParameterEventSubscriber(void);
    bool IsAddedToROS2Node(void) const;
    bool IsParameterServerReady(void) const;

    /* Add a node and parameter to monitor */
    bool AddParameterForTracking(const std::string& parameterName);
    /* Remove a parameter that is being monitored. If no parameters are being monitored for a node, stop monitoring the node as well*/
    bool RemoveParameterFromTracking(const std::string& parameterName);

    bool IsParameterValueSet(const std::string& parameterName) const;

    /* Main methods, recommended for C++ users since we can check return code and avoid copy for result.
     Returns data type if the parameter is tracked. Else it returns an empty string */
    std::string GetParameterType(const std::string& parameterName, std::string& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::string GetParameterType(const std::string& parameterName) {
        std::string result;
        // TODO : Add Warning
        GetParameterType(parameterName, result);
        return result;
    }

    /* Main methods, recommended for C++ users since we can check return code and avoid copy for result.
     Prints value of a tracked parameter after converting it to a string */
    bool PrintParameterValue(const std::string& parameterName, std::string& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::string PrintParameterValue(const std::string& parameterName) {
        std::string result;
        PrintParameterValue(parameterName, result);
        return result;
    }

    /* Main methods, recommended for C++ users since we can check return code and avoid copy for result.
    Returns the value of the parameter if it is a boolean. Users should always make sure that the key exists
    and the parameter type is string before calling this method*/
    bool GetParameterAsBool(const std::string& parameterName, bool& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline bool GetParameterAsBool(const std::string& parameterName) {
        bool result;
        GetParameterAsBool(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is an Integer . Users should always make sure the key exists and
    the parameter type is an integer with GetParameterType before calling this method.
    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsInteger(const std::string& parameterName, int& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline int GetParameterAsInteger(const std::string& parameterName) {
        int result;
        GetParameterAsInteger(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a double. Users should always make sure the key exists and
    the parameter type is a double with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsDouble(const std::string& parameterName, double& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline double GetParameterAsDouble(const std::string& parameterName) {
        double result;
        GetParameterAsDouble(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a string. Users should always make sure that the key exists
    and the parameter type is string before calling this method

    Main methods, recommended for C++ users since we can check return code and avoid copy for result.*/
    bool GetParameterAsString(const std::string& parameterName, std::string& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::string GetParameterAsString(const std::string& parameterName) {
        std::string result;
        GetParameterAsString(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of bools. Users should always make sure the key exists and
    the parameter type is a vector of bools with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    // bool GetParameterAsVectorOfBools(const std::string & parameterName, std::vector<bool> & result);
    // /* convenience methods for users to skip pair creation, mostly for Python users */

    // TODO : unable to build for some reasong
    // inline std::vector<bool> GetParameterAsVectorOfBools(const std::string &parameterName) {
    //   std::vector<bool> result;
    //   // GetParameterAsVectorOfBools(parameterName, result);
    //   return result;
    // }

    /* Returns the value of the parameter if it is a vector of ints. Users should always make sure the key exists and
    the parameter type is a vector of ints with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfIntegers(const std::string& parameterName, std::vector<int64_t>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<int64_t> GetParameterAsVectorOfIntegers(const std::string& parameterName) {
        std::vector<int64_t> result;
        GetParameterAsVectorOfIntegers(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of doubles. Users should always make sure the key exists and
    the parameter type is a vector of doubles with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfDoubles(const std::string& parameterName, std::vector<double>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<double> GetParameterAsVectorOfDoubles(const std::string& parameterName) {
        std::vector<double> result;
        GetParameterAsVectorOfDoubles(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of strings. Users should always make sure the key exists and
    the parameter type is a vector of strings with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfStrings(const std::string& parameterName, std::vector<std::string>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<std::string> GetParameterAsVectorOfStrings(const std::string& parameterName) {
        std::vector<std::string> result;
        GetParameterAsVectorOfStrings(parameterName, result);
        return result;
    }

    virtual void ParameterSet()
    {
     this->InvokeCustomModifiedEvent(vtkMRMLROS2ParameterNode::ParameterModifiedEvent);
    }
 
    // Save and load
    virtual void ReadXMLAttributes(const char** atts) override;
    virtual void WriteXML(std::ostream& of, int indent) override;
    void UpdateScene(vtkMRMLScene* scene) override;

   protected:
// vector to store all parameter names that are tracked by the node. This is used for saving and reloading state.
    std::vector<std::string> mTrackedParameterNamesList = {}; 
    void SetmTrackedParameterNamesList(const std::deque<std::string>& mTrackedParameterNamesList);
    std::deque<std::string> GetmTrackedParameterNamesList();

   protected:
    vtkMRMLROS2ParameterInternals* mInternals = nullptr;
    std::string mMRMLNodeName = "ros2:param:undefined";
    std::string mTrackedNodeName = "undefined";
    bool mIsInitialized = false;

    // For ReadXMLAttributes
    vtkGetMacro(mMRMLNodeName, std::string);
    vtkSetMacro(mMRMLNodeName, std::string);
    vtkGetMacro(mTrackedNodeName, std::string);
    vtkSetMacro(mTrackedNodeName, std::string);

};

#endif  // __vtkMRMLROS2ParameterNode_h

// ros2 = slicer.mrmlScene.GetFirstNodeByName('ros2:node:test_node')
