import numpy as np

origin = np.array([0.0, 0.0, 1.0])

theta1 = np.pi/4
l1 = np.sqrt(8)
T1 = np.array([[np.cos(theta1), -np.sin(theta1), l1 * np.cos(theta1)],
              [np.sin(theta1), np.cos(theta1), l1 * np.sin(theta1)],
              [0.0, 0.0, 1.0]])

theta2 = -np.pi/12
l2 = 2
T2 = np.array([[np.cos(theta2), -np.sin(theta2), l2 * np.cos(theta2)],
              [np.sin(theta2), np.cos(theta2), l2 * np.sin(theta2)],
              [0.0, 0.0, 1.0]])

print(T2 @ origin)
print(T1 @ T2 @ origin)
print(2+np.sqrt(3), 3)