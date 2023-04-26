#include <vtkMRMLROS2CISST.h>

#include <vtkSlicerToROS2.h>

const double M_TO_MM = 0.001;

#include <cisst_msgs/msg/cartesian_impedance_gains.hpp>

void vtkSlicerToROS2(vtkMatrix4x4 * input, cisst_msgs::msg::CartesianImpedanceGains & result,
		     const std::shared_ptr<rclcpp::Node> &) // the input should be something related to the closest point on the volume
{
  // stiffness = elasticity
  // damping = viscosity
  result.pos_stiff_neg.x = 0.0;
  result.pos_stiff_pos.x = 0.0;
  result.pos_damping_neg.x = 0.0;
  result.pos_damping_pos.x = 0.0;
  result.pos_stiff_neg.y = 0.0;
  result.pos_stiff_pos.y = 0.0;
  result.pos_damping_neg.y = 0.0;
  result.pos_damping_pos.y = 0.0;
  // These gains are preconfigured for our application - they can be modified according to your device/ specific needs
  result.pos_stiff_neg.z = -10.0;
  result.pos_stiff_pos.z = 10.0;
  result.pos_damping_neg.z = -20.0;
  result.pos_damping_pos.z = 20.0;

  double stiffOri = -0.2;
  double dampOri = -0.01;
  result.ori_stiff_neg.x = stiffOri;
  result.ori_stiff_pos.x = stiffOri;
  result.ori_damping_neg.x = dampOri;
  result.ori_damping_pos.x = dampOri;
  result.ori_stiff_neg.y = stiffOri;
  result.ori_stiff_pos.y = stiffOri;
  result.ori_damping_neg.y = dampOri;
  result.ori_damping_pos.y = dampOri;
  result.ori_stiff_neg.z = 0.0;
  result.ori_stiff_pos.z = 0.0;
  result.ori_damping_neg.z = 0.0;
  result.ori_damping_pos.z = 0.0;

  result.force_position.x = input->GetElement(0, 3) * M_TO_MM;
  result.force_position.y = input->GetElement(1, 3) * M_TO_MM;
  result.force_position.z = input->GetElement(2, 3) * M_TO_MM;

  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);
  result.force_orientation.x = q[1];
  result.force_orientation.y = q[2];
  result.force_orientation.z = q[3]; // should this come from breach warning
  result.force_orientation.w = q[0];
  result.torque_orientation.x = q[1];
  result.torque_orientation.y = q[2];
  result.torque_orientation.z = q[3];
  result.torque_orientation.w = q[0];
}

#include <vtkMRMLROS2PublisherInternals.h>
VTK_MRML_ROS_PUBLISHER_VTK_CXX(vtkMatrix4x4, cisst_msgs::msg::CartesianImpedanceGains, CartesianImpedanceGains);
