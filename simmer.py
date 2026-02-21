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