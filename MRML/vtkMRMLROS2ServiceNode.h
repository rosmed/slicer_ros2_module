#ifndef __vtkMRMLROS2ServiceNode_h
#define __vtkMRMLROS2ServiceNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkCommand.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <memory> //for shared_ptr
#include <vtkTable.h>
// #include <rclcpp/rclcpp.hpp>
// #include "std_srvs/srv/trigger.hpp"

// forward declaration for internals
class vtkMRMLROS2ServiceInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ServiceNode : public vtkMRMLNode {
    // friend declarations
    friend class vtkMRMLROS2ServiceInternals;
    friend class vtkMRMLROS2NodeNode;

   protected:
    vtkMRMLROS2ServiceNode(void);
    ~vtkMRMLROS2ServiceNode(void);

   public:

    enum Events
    {
      ServiceModifiedEvent = vtkCommand::UserEvent + 54
    };

    vtkTypeMacro(vtkMRMLROS2ServiceNode, vtkMRMLNode);
    typedef vtkMRMLROS2ServiceNode SelfType;
    static SelfType* New(void);
    void PrintSelf(ostream& os, vtkIndent indent) override;
    vtkMRMLNode* CreateNodeInstance(void) override;
    const char* GetNodeTagName(void) override;

    bool AddToROS2Node(const char* nodeId, const std::string& monitoredNodeName);
    // bool RemoveFromROS2Node(const char *nodeId);

    // bool Spin(void);

    /*! Get the name of the node holding the services we're looking for. */
    inline const std::string & GetMonitoredNodeName(void) const {
      return mMonitoredNodeName;
    }

    vtkTable *GetLastResponseAsTable(void);
    bool GetLastResponse(vtkSmartPointer<vtkTable> &output);
    bool InitializeRequest(void);
    void SendAsyncRequest(void);
    void SendBlockingRequest(unsigned int wait_time_ms = 10000);
    bool IsAddedToROS2Node(void) const;
    // bool IsMonitoredNodeReady(void) const;

//     /*! Add a node and service to monitor */
//     bool AddService(const std::string& serviceName);
//     /*! Remove a service that is being monitored. If no services are being monitored for a node, stop monitoring the node as well*/
//     bool RemoveService(const std::string& serviceName);

//     bool IsServiceSet(const std::string& serviceName, bool noWarning = false) const;

//     /*! Returns data type if the service is monitored. Else it returns an empty string */
//     bool GetServiceType(const std::string& serviceName, std::string& result);
//     /*! convenience methods for users to return output, mostly for Python users */
//     std::string GetServiceType(const std::string& serviceName);


//     /*! Prints value of a monitored service after converting it to a
//       string. Returns empty string if value not set */
//    std::string PrintService(const std::string& serviceName);

// // todo-address: in RbotoNode, make sure we check with IsServiceSet and GetServiceType

//     /*! Returns true if it is a Integer and it is set. Users should
//       always make sure that the key exists and the service type is
//       Integer before calling this method Main method, recommended for
//       C++ users since we can check return code and avoid copy for
//       result. */
//     bool GetServiceAsInteger(const std::string& serviceName, int& result);
//     /*! Convenience methods for users to return output, mostly for
//       Python users */
//     int GetServiceAsInteger(const std::string& serviceName);


 

    // // Save and load
    // virtual void ReadXMLAttributes(const char** atts) override;
    // virtual void WriteXML(std::ostream& of, int indent) override;
    // void UpdateScene(vtkMRMLScene* scene) override;

   protected:

    // bool SetupServiceEventSubscriber(void);
    // bool CheckServiceExistsAndIsSet(const std::string &serviceName) const;
    // void SetMonitoredServiceNamesCache(const std::vector<std::string>& MonitoredServiceNamesCache);


   protected:

    std::shared_ptr<vtkMRMLROS2ServiceInternals> mInternals = nullptr; // todo-address : changed to shared_ptr

    std::string mMRMLNodeName = "ros2:param:undefined";
    std::string mMonitoredNodeName = "undefined";
    bool mIsServiceServerReady = false;

    // For ReadXMLAttributes
    vtkGetMacro(mMRMLNodeName, std::string);
    vtkSetMacro(mMRMLNodeName, std::string);
    vtkGetMacro(mMonitoredNodeName, std::string);
    vtkSetMacro(mMonitoredNodeName, std::string);

    private:
    // vector to store all service names that are monitored by the node. This is used for saving and reloading state.
    std::vector<std::string> MonitoredServiceNamesCache = {};
    std::vector<std::string> GetMonitoredServiceNamesCache();

};

#endif  // __vtkMRMLROS2ServiceNode_h

// ros2 = slicer.mrmlScene.GetFirstNodeByName('ros2:node:test_node')
