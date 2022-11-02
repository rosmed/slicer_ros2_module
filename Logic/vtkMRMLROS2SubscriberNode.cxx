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
}

size_t vtkMRMLROS2SubscriberNode::GetNumberOfMessages(void) const
{
  return mNumberOfMessages;
}

// vtkMRMLNodeNewMacro(vtkMRMLROS2SubscriberPoseStamped);

// void vtkMRMLROS2SubscriberNode::SubscriberCallBack(const geometry_msgs::msg::PoseStamped& pose){
//   std::cerr << "Sub callback" << std::endl;
//   std::cerr << pose.pose.position.x << std::endl;
  //vtkMRMLTransformNode *parentTransformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(("hi"));
  //TODO: need to figure out how to update tf node from this function (somehow will need to get the 'this' part)
  // This code is copied from logic so I can probably put it in a function and reuse
//   auto x = pose.pose.position.x*MM_TO_M_CONVERSION;
//   auto y = pose.pose.position.y*MM_TO_M_CONVERSION;
//   auto z = pose.pose.position.z*MM_TO_M_CONVERSION;
//   auto q_w = pose.pose.orientation.w;
//   auto q_x = pose.pose.orientation.x;
//   auto q_y = pose.pose.orientation.y;
//   auto q_z = pose.pose.orientation.z;
//   // Pose should be stored in a vtk MRML Transform node
//
//   // Copy contents into a vtkMRMLTransformNode
//   const double q[4] = {q_w, q_x, q_y, q_z};
//   double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
//   vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix
//   vtkNew<vtkMatrix4x4> Tf;
//   for (size_t row = 0; row < 3; row++) {
//     for (size_t column = 0; column < 3; column++) {
//       Tf->SetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
//      }
//   }
//   // Apply translation vector
//   Tf->SetElement(0,3, x);
//   Tf->SetElement(1,3, y);
//   Tf->SetElement(2,3, z);
//
//   vtkMRMLScene* scene = GetScene();
//   vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(scene->GetFirstNodeByName("/blah_blah"));
//   if (transformNode != 0){
//     transformNode->SetMatrixTransformToParent(Tf);
//     transformNode->TransformModified();
//   }





