# Forward Kinematics Demo

import simmer
import numpy as np
import time
import matplotlib.pyplot as plt
from viz import plot_chain

# Define the joints and links
joint1 = simmer.Joint(angle=45.0, radians=False, name="shoulder")
link1 = simmer.Link(length=np.sqrt(8), name="upper arm")
joint2 = simmer.Joint(angle=-15.0, radians=False, name="elbow")
link2 = simmer.Link(length=2.0, name="lower arm")

# Define the chain
chain = simmer.Chain(links=[link1, link2], joints=[joint1, joint2])

# Calculate the forward kinematics
chain.forward_kinematics()
end_effector = chain.get_end_effector_position()
joint_positions = chain.get_joint_positions()
print(f"End effector position: {end_effector}")
print(f"Joint positions: {joint_positions}")

# Plot results
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Plot initial configuration
chain.joints[0].angle = 45.0 * np.pi / 180.0
chain.joints[1].angle = -15.0 * np.pi / 180.0
chain.joint_angles = np.array([joint.angle for joint in chain.joints])
chain.forward_kinematics()
plot_chain(chain, target=end_effector, ax=axes[0])
axes[0].set_title('Initial Configuration')

target_ee_position = np.array([1.0, 1.0])

# Plot gradient descent result
chain.inverse_kinematics_by_gradient_descent(target_ee_position, learning_rate=0.01, num_iterations=1000)
print(f"Gradient descent joint angles: {chain.get_joint_angles()}")
print(f"Gradient descent joint positions: {chain.get_joint_positions()}")
chain.forward_kinematics()
plot_chain(chain, target=target_ee_position, ax=axes[1])
axes[1].set_title('Gradient Descent Result')

# Plot pseudoinverse result
chain.joints[0].angle = 45.0 * np.pi / 180.0
chain.joints[1].angle = -15.0 * np.pi / 180.0
chain.joint_angles = np.array([joint.angle for joint in chain.joints])
chain.inverse_kinematics_by_pseudoinverse(target_ee_position, num_iterations=1000, damping=0.1)
print(f"Pseudoinverse joint angles: {chain.get_joint_angles()}")
print(f"Pseudoinverse joint positions: {chain.get_joint_positions()}")
chain.forward_kinematics()
plot_chain(chain, target=target_ee_position, ax=axes[2])
axes[2].set_title('Pseudoinverse Result')

plt.tight_layout()
plt.show()