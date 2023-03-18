#include <vtkMRMLROS2Utils.h>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>

#include <rclcpp/rclcpp.hpp>


bool vtkMRMLROS2::ROSInit(void)
{
  try {
    std::string nodeName = "SlicerROS";
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[nodeName.size() + 1];
    strcpy(argv[0], nodeName.c_str());
    int argc = 1;
    rclcpp::init(argc, argv);
  } catch (...) {
    std::cerr << "vtkMRMLROS2::ROSInit: rclcpp::init was called multiple times. This is fine." << std::endl; // Key word this
    return false;
  }
  return true;
}


void vtkMRMLROS2::ROSShutdown(void)
{
  rclcpp::shutdown();
}


vtkMRMLROS2NodeNode * vtkMRMLROS2::CheckROS2NodeExists(vtkMRMLNode * nodeInScene, const char * nodeId, std::string & errorMessage)
{
  vtkMRMLScene * scene = nodeInScene->GetScene();
  if (!scene) {
    errorMessage = std::string("node \"") + nodeInScene->GetName() + "\" needs to be added to the scene first";
    return nullptr;
  }
  vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
  if (!rosNodeBasePtr) {
    errorMessage = "unable to locate node";
    return nullptr;
  }
  vtkMRMLROS2NodeNode * rosNodePtr = dynamic_cast<vtkMRMLROS2NodeNode *>(rosNodeBasePtr);
  if (!rosNodePtr) {
    errorMessage = std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NodeNode";
    return nullptr;
  }
  return rosNodePtr;
}
