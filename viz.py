import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simmer import Chain


def plot_chain(chain: Chain, target: np.ndarray = None, ax=None):
  if ax is None:
    _, ax = plt.subplots()

  # Run the forward kinematics to make sure the chain has calculated the joint positions
  chain.forward_kinematics()
  
  pts = chain.joint_positions
  ax.plot(pts[:, 0], pts[:, 1], 'o-', color='steelblue', linewidth=2, markersize=8)
  ax.plot(pts[-1, 0], pts[-1, 1], '*', color='tomato', markersize=14, label='EE')

  if target is not None:
    ax.plot(target[0], target[1], '*', color='limegreen', markersize=14, label='Target')

  ax.set_aspect('equal')
  ax.grid(True)
  ax.legend()
  return ax

