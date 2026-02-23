from typing import Iterable
import numpy as np

ORIGIN = np.array([0.0, 0.0, 1.0])

class Joint:
  def __init__(self, angle: float = 0.0, radians: bool = True, name: str = None):
    # The angle of the joint, in radians, defined as the angle 
    # between the link and the previous link
    self.angle = angle if radians else angle * np.pi/180.0
    self.name = name

class Link:
  def __init__(self, length: float = 0.0, name: str = None):
    # The length of the link, in meters
    self.length = length
    self.name = name

class Transform:
  # Homogeneous transformation matrix to represent the transform
  def __init__(self, joint: Joint, link: Link):
    self.joint = joint
    self.link = link
    self.transform = np.array(
      [[np.cos(self.joint.angle), -np.sin(self.joint.angle), self.link.length * np.cos(self.joint.angle)],
      [np.sin(self.joint.angle), np.cos(self.joint.angle), self.link.length * np.sin(self.joint.angle)],
      [0.0, 0.0, 1.0]])

class Chain:
  def __init__(self,  links: Iterable[Link], joints: Iterable[Joint]):
    # The links of the chain and the joints that connect them
    # The links are connected to the joints in order and vice versa
    # The position of the first joint is defined by the origin 
    # and the position of the last link is the end effector
    self.links = links
    self.joints = joints
    self.joint_angles = np.array([joint.angle for joint in joints])

    assert len(self.links) == len(self.joints), "The number of links and joints must be the same"
    assert len(self.links) > 0, "The chain must have at least one link"

  def forward_kinematics(self):
    # Calculate the forward kinematics of the chain
    # Returns the position of the end effector in homogeneous coordinates
    self.transforms = [Transform(joint, link) for joint, link in zip(self.joints, self.links)]
    
    solution = self.transforms[0].transform
    self.joint_positions = np.zeros((len(self.joints) + 1, 2))

    for i in range(1, len(self.transforms)):
      self.joint_positions[i] = (solution @ ORIGIN)[:2]
      solution = solution @ self.transforms[i].transform

    self.joint_positions[len(self.joints)] = (solution @ ORIGIN)[:2]

  def get_end_effector_position(self):
    return self.joint_positions[-1]

  def get_joint_positions(self):
    return self.joint_positions[:-1]

  def get_joint_angles(self):
    return self.joint_angles.tolist()

  def get_jacobian(self):
    # Calculate the Jacobian of the chain
    # Returns the Jacobian matrix
    self.jacobian = np.zeros((2, len(self.joints)))
    for i in range(len(self.joints)):
      # For each joint, the partial derivative of the end effector position is
      # the vector perpendicular to the vector connecting the joint to the end effector
      d_position = self.joint_positions[-1] - self.joint_positions[i]
      self.jacobian[0, i] = -d_position[1]
      self.jacobian[1, i] = d_position[0]
    return self.jacobian

  def sync_joints_to_joint_angles(self):
    for joint, angle in zip(self.joints, self.joint_angles):
      joint.angle = angle

  def inverse_kinematics_by_gradient_descent(self, target_ee_position: Iterable[float], learning_rate: float = 0.01, num_iterations: int = 1000):
    # Calculate the inverse kinematics of the chain by gradient descent
    # Returns the joint positions that result in the given end effector position
    self.forward_kinematics()
    for _ in range(num_iterations):
      if np.linalg.norm(self.get_end_effector_position() - target_ee_position) < 1e-6:
        break
      self.joint_angles = self.joint_angles - learning_rate * self.get_jacobian().T @ (self.get_end_effector_position() - target_ee_position)
      self.sync_joints_to_joint_angles()
      self.forward_kinematics()

  def inverse_kinematics_by_pseudoinverse(self, target_ee_position: Iterable[float], num_iterations: int = 1000, damping: float = 0.1):
    # Use the Moore-Penrose pseudoinverse of the Jacobian to calculate the joint angles
    # Add dampening to the pseudoinverse to prevent numerical instability
    self.forward_kinematics()
    for _ in range(num_iterations):
      if np.linalg.norm(self.get_end_effector_position() - target_ee_position) < 1e-6:
        break
      self.jacobian = self.get_jacobian()
      J_plus = self.jacobian.T @ np.linalg.inv(self.jacobian @ self.jacobian.T + damping * np.eye(2))
      d_theta = J_plus @ (target_ee_position - self.get_end_effector_position())
      self.joint_angles = self.joint_angles + d_theta
      self.sync_joints_to_joint_angles()
      self.forward_kinematics()

