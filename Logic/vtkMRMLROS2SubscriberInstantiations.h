#ifndef __vtkMRMLROS2SubscriberInstantiations_h
#define __vtkMRMLROS2SubscriberInstantiations_h

#include <vtkROS2ToSlicer.h>
#include <vtkMRMLROS2SubscriberImplementation.h>

typedef vtkMRMLROS2SubscriberImplementation<std_msgs::msg::String, std::string> vtkMRMLROS2SubscriberString;

typedef vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkSmartPointer<vtkMatrix4x4> > vtkMRMLROS2SubscriberPoseStamped;

/*
typedef vtkMRMLROS2SubscriberImplementation<std_msgs::msg::String, std::string> vtkMRMLROS2SubscriberStringBase;
class vtkMRMLROS2SubscriberString: public vtkMRMLROS2SubscriberStringBase
{
public:
  typedef vtkMRMLROS2SubscriberString SelfType;
  // vtkTypeMacro(SelfType, vtkMRMLROS2SubscriberStringBase);
  static SelfType * New(void);
  virtual vtkMRMLNode * CreateNodeInstance(void) override {
    return SelfType::New();
  }
protected:
  vtkMRMLROS2SubscriberString() {};
  ~vtkMRMLROS2SubscriberString() {};
};

typedef vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkMatrix4x4> vtkMRMLROS2SubscriberPoseStampedBase;
class vtkMRMLROS2SubscriberPoseStamped: public vtkMRMLROS2SubscriberPoseStampedBase
{
public:
  typedef vtkMRMLROS2SubscriberPoseStamped SelfType;
  // vtkTypeMacro(SelfType, vtkMRMLROS2SubscriberPoseStampedBase);
  static SelfType * New(void);

  virtual vtkMRMLNode * CreateNodeInstance(void) override {
    return SelfType::New();
  }
protected:
  vtkMRMLROS2SubscriberPoseStamped() {};
  ~vtkMRMLROS2SubscriberPoseStamped() {};
};
*/

#endif
