/*=auto=========================================================================
Portions (c) Copyright 2009 Brigham and Women's Hospital (BWH) All Rights Reserved.
See Doc/copyright/copyright.txt
or http://www.slicer.org/copyright/copyright.txt for details.
Program:   3D Slicer
Module:    $RCSfile: vtkMRMLGradientAnisotropicDiffusionFilterNode.cxx,v $
Date:      $Date: 2006/03/17 15:10:10 $
Version:   $Revision: 1.2 $
=========================================================================auto=*/

#include "vtkMRMLROS2SubscriberNode.h"

//----------------------------------------------------------------------------
vtkMRMLROS2SubscriberNode::vtkMRMLROS2SubscriberNode()
{
}

//----------------------------------------------------------------------------
vtkMRMLROS2SubscriberNode::~vtkMRMLROS2SubscriberNode()
{
}

void vtkMRMLROS2SubscriberNode::SetTopic(const std::string & topic)
{
  mTopic = topic;
  mNodeName = "ros2:" + topic;
  this->SetName(mNodeName.c_str());
  // mNodeName.append(topic);
  // mNodeNamePtr = mNodeName.c_str();
}

size_t vtkMRMLROS2SubscriberNode::GetNumberOfMessages(void) const
{
  return mNumberOfMessages;
}

