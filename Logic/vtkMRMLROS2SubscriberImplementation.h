#ifndef __vtkMRMLROS2SubscriberImplementation_h
#define __vtkMRMLROS2SubscriberImplementation_h

#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkSlicerRos2ModuleLogicExport.h>

template <typename _ros_type, typename _slicer_type>
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberImplementation: public vtkMRMLROS2SubscriberNode
{
 protected:
  _ros_type mLastMessageROS;
  std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription;

 protected:

  vtkMRMLROS2SubscriberImplementation() {}
  ~vtkMRMLROS2SubscriberImplementation() {}

  /**
   * This is the ROS callback for the subscription.  This methods
   * saves the ROS message as-is and set the modified flag for the
   * MRML node
   */
  void SubscriberCallback(const _ros_type & message) {
    // \todo is there a timestamp in MRML nodes we can update from the ROS message?
    mLastMessageROS = message;
    mNumberOfMessages++;
    this->Modified();
  }

 public:
  typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> SelfType;
  vtkTemplateTypeMacro(SelfType, vtkMRMLROS2SubscriberNode);

  void PrintSelf(ostream&, vtkIndent) override {}

  // todo this method should not require a direct pointer on the
  // rclcpp::Node but shoudl use a MRMLROS2NodeNode, either by name or
  // vtk pointer, dynamic cast and then retrieve the rclcpp::Node -
  // this will require to add some friend declarations
  void SetSubscriber(std::shared_ptr<rclcpp::Node> nodePointer) {
    std::cerr << "SetSubscriber for " << mTopic << std::endl;
    mSubscription= nodePointer->create_subscription<_ros_type>(mTopic, 10000,
							       std::bind(&SelfType::SubscriberCallback, this, std::placeholders::_1));
  }

  const char * GetROSType(void) const override
  {
    return typeid(_ros_type).name();
  }

  const char * GetSlicerType(void) const override
  {
    return typeid(_slicer_type).name();
  }

  /**
   * Retrieve the last message in YAML format, this can be used in
   * Python to get all the information received.  The main drawbacks
   * are that it involves string parsing which takes time and doesn't
   * provide the full resolution for floating point numbers.
   */
  std::string GetLastMessageYAML(void) const override
  {
    std::stringstream out;
    rosidl_generator_traits::to_yaml(mLastMessageROS, out);
    return out.str();
  }

  /**
   * Attempting to save the file to XML
   *
   */
  virtual void WriteXML(ostream& of, int nIndent) override {
    this->Superclass::WriteXML(of, nIndent);
    // of = std::cerr;
    std::cerr << "This is being used" << endl; //Sanity check to make sure function is being called;

    this->content = "random data"; // setting random value

    of << " random_manual_field=\"" << this->content << "\""; //manually writing

    vtkMRMLWriteXMLBeginMacro(of);
    vtkMRMLWriteXMLStdStringMacro(random_manual_field_2, content); // using vtk macro - This requires vtkGetMacro to be set for the string
    vtkMRMLWriteXMLEndMacro();

    std::stringstream ss;
    ss << of.rdbuf();
    std::string myString = ss.str();

    std::cerr << ss.str() << "End mine" << endl;
    // std::cerr << of.str() << "End mine" << endl;
  }
  
  // GetParent - SrtPartent
  std::string content; // defining a random string
  vtkGetMacro(content, std::string); // used in writeXML
};




template <typename _ros_type, typename _slicer_type>
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberVTKImplementation:
  public vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type>
{
 private:
  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

 protected:

  vtkMRMLROS2SubscriberVTKImplementation() {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  ~vtkMRMLROS2SubscriberVTKImplementation() {}

 public:
  typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> SuperClass;
  typedef vtkMRMLROS2SubscriberVTKImplementation<_ros_type, _slicer_type> SelfType;
  vtkTemplateTypeMacro(SelfType, SuperClass);

  static SelfType * New(void); // vtkObject - Create an object with Debug turned off, modified time initialized to zero, and reference counting on.

  // Create instance of the default node. Like New only virtual. -> Explanation from doxygen
  vtkMRMLNode * CreateNodeInstance(void) override {
    return SelfType::New();
  }

  void GetLastMessage(vtkSmartPointer<_slicer_type> result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  vtkVariant GetLastMessageVariant(void) override
  {
    GetLastMessage(mLastMessageSlicer);
    return vtkVariant(mLastMessageSlicer.GetPointer());
  }
};



template <typename _ros_type, typename _slicer_type>
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberNativeImplementation:
  public vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type>
{
 private:
  _slicer_type mLastMessageSlicer;

 protected:

  vtkMRMLROS2SubscriberNativeImplementation() {}

  ~vtkMRMLROS2SubscriberNativeImplementation() {}

 public:
  typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> SuperClass;
  typedef vtkMRMLROS2SubscriberNativeImplementation<_ros_type, _slicer_type> SelfType;
  vtkTemplateTypeMacro(SelfType, SuperClass);

  static SelfType * New(void); // vtkObject - Create an object with Debug turned off, modified time initialized to zero, and reference counting on.

  // Create instance of the default node. Like New only virtual. -> Explanation from doxygen
  vtkMRMLNode * CreateNodeInstance(void) override {
    return SelfType::New();
  }

  void GetLastMessage(_slicer_type & result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  vtkVariant GetLastMessageVariant(void) override
  {
    GetLastMessage(mLastMessageSlicer);
    return vtkVariant(mLastMessageSlicer);
  }
};


#endif
