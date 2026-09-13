#include <vtkMRMLROS2Utils.h>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>

#include <rclcpp/rclcpp.hpp>

#include <signal.h>
#if defined(__APPLE__)
#include <dlfcn.h>
#endif


void vtkMRMLROS2SignalHandler(int)
{
  vtkMRMLROS2::ROSShutdown();
  abort();
}


bool vtkMRMLROS2::ROSInit(void)
{
  if (rclcpp::ok()) {
    return true;
  }

  try {
#if defined(__APPLE__)
    // Pre-load the logging implementation via @rpath so that subsequent
    // bare-name dlopen("librcl_logging_spdlog.dylib") finds it in memory.
    dlopen("@rpath/librcl_logging_spdlog.dylib", RTLD_NOW | RTLD_GLOBAL);
#endif

    const char * fake_argv[] = {"SlicerROS", nullptr};
    int argc = 1;
    rclcpp::init(argc, fake_argv);
    // remove ROS signal handlers since they won't abort
    rclcpp::uninstall_signal_handlers();
    // use our own to make sure ROS closes properly
    signal(SIGINT, vtkMRMLROS2SignalHandler);
  } catch (const std::exception & e) {
    vtkGenericWarningMacro(<< "vtkMRMLROS2::ROSInit failed: " << e.what());
    return false;
  } catch (...) {
    vtkGenericWarningMacro(<< "vtkMRMLROS2::ROSInit failed with an unknown exception.");
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
