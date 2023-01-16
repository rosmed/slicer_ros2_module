import slicer

def run():
    print('creating a ROS2 node')
    ros2Node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLROS2NODENode')
    print('initialize the ROS2 node')
    ros2Node.Create('testNode')
    print('add a publisher on topic test_string_xkcd')
    testPub = ros2Node.CreateAndAddPublisher('vtkMRMLROS2PublisherStringNode', 'test_string_xkcd')
    print('add a subscriber on topic test_string_xkcd')
    testSub = ros2Node.CreateAndAddSubscriber('vtkMRMLROS2SubscriberStringNode', 'test_string_xkcd')
    print('publish')
    testPub.Publish('xkcd')

if __name__ == '__main__':
    slicer_ros2_module_test()
