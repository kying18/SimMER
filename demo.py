# Forward Kinematics Demo

import simmer
import numpy as np

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