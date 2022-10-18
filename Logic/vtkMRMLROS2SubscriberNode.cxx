/*=auto=========================================================================
Portions (c) Copyright 2009 Brigham and Women's Hospital (BWH) All Rights Reserved.
See Doc/copyright/copyright.txt
or http://www.slicer.org/copyright/copyright.txt for details.
Program:   3D Slicer
Module:    $RCSfile: vtkMRMLGradientAnisotropicDiffusionFilterNode.cxx,v $
Date:      $Date: 2006/03/17 15:10:10 $
Version:   $Revision: 1.2 $
=========================================================================auto=*/

// OpenIGTLinkIF MRML includes
#include "vtkMRMLROS2SubscriberNode.h"
#include <vtkNew.h>


//------------------------------------------------------------------------------
vtkMRMLNodeNewMacro(vtkMRMLROS2SubscriberNode);

//----------------------------------------------------------------------------
vtkMRMLROS2SubscriberNode::vtkMRMLROS2SubscriberNode()
{

}

//----------------------------------------------------------------------------
vtkMRMLROS2SubscriberNode::~vtkMRMLROS2SubscriberNode()
{
}


//----------------------------------------------------------------------------
void vtkMRMLROS2SubscriberNode::SetSubscriber(std::shared_ptr<rclcpp::Node> mNodePointer){
  std::cerr << "Set sub called" << std::endl;
  mSubscription= mNodePointer->create_subscription<geometry_msgs::msg::PoseStamped>("/blah_blah", 10000, std::bind(&vtkMRMLROS2SubscriberNode::SubscriberCallBack, this, std::placeholders::_1));
}

void vtkMRMLROS2SubscriberNode::SubscriberCallBack(const geometry_msgs::msg::PoseStamped& pose){
  std::cerr << "Sub callback" << std::endl;
}
