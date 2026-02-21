from typing import Iterable
import numpy as np

class Joint:
  def __init__(self, angle: float = 0.0, radians: bool = True, name: str = None):
    # The angle of the joint defined as the angle between the link and the previous link
    # If radians is False, the angle is in degrees
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
      [[np.cos(joint.angle), -np.sin(joint.angle), link.length * np.cos(joint.angle)],
      [np.sin(joint.angle), np.cos(joint.angle), link.length * np.sin(joint.angle)],
      [0.0, 0.0, 1.0]])

class Chain:
  def __init__(self, origin: Iterable[float] = [0.0, 0.0], links: Iterable[Link] = [], joints: Iterable[Joint] = []):
    # The links of the chain and the joints that connect them
    # The links are connected to the joints in order and vice versa
    # The position of the first joint is defined by the origin 
    # and the position of the last link is the end effector
    self.links = links
    self.joints = joints
    # Represent the origin as a homogeneous coordinate
    self.origin_h = np.array(origin + [1.0])

  def forward_kinematics(self):
    # Calculate the forward kinematics of the chain
    # Returns the position of the end effector in homogeneous coordinates
    self.transforms = [Transform(joint, link) for joint, link in zip(self.joints, self.links)]
    # Initialize the solution as the identity matrix
    solution = np.identity(3)
    for transform in self.transforms:
      solution = solution @ transform.transform
    return solution @ self.origin_h