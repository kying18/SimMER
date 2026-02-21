# Forward Kinematics Demo

import simmer

# Define the joints and links
joint1 = simmer.Joint(angle=45.0, radians=False, name="shoulder")
link1 = simmer.Link(length=2.0, name="upper arm")
joint2 = simmer.Joint(angle=-15.0, radians=False, name="elbow")
link2 = simmer.Link(length=3.0, name="lower arm")

# Define the chain
chain = simmer.Chain(origin=[0.0, 0.0], links=[link1, link2], joints=[joint1, joint2])

# Calculate the forward kinematics
end_effector = chain.forward_kinematics()
print(f"End effector position: {end_effector[:2]}")