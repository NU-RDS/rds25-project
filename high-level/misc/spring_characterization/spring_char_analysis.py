import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

# Load CSV
# Encoder 1 readings correspond to shaft
# Encoder 2 readings correspond to motor
# Loadcell values are measured in grams 
df = pd.read_csv("data/sensor_log_20250511_171159.csv")

# Motor gear ratio
GEAR_RATIO = 36
SCALING_FACTOR = 9.81 * 1e-3

# Radius of pulley (10mm)
RADIUS = 10 * 1e-3

# Degree to radian
DEG2RAD = np.pi/180

# Reference values
encoder1_ref = df["Encoder1"].iloc[0]
encoder2_ref = df["Encoder2"].iloc[0]
loadcell_ref = df["Loadcell"].iloc[0]

# Unwrap encoder2 values
encoder1 = df["Encoder1"].values
encoder2 = np.rad2deg(np.unwrap(np.deg2rad(df["Encoder2"].values)))
loadcell = df["Loadcell"].values

# Compute x and y 
x = ((encoder1 - encoder1_ref) - ((encoder2 - encoder2_ref)/GEAR_RATIO))*DEG2RAD
y = (loadcell) * SCALING_FACTOR * RADIUS

# Reshape 
x = x.reshape(-1, 1)
y = y.reshape(-1, 1)

# Linear Regression
model = LinearRegression()
model.fit(x, y)
y_pred = model.predict(x)

slope = model.coef_[0][0]
intercept = model.intercept_[0]
r_squared = r2_score(y, y_pred)
# print(r_squared)

# Plot 

# # Plot raw encoder and wrapped encoder 
# plt.figure(figsize=(10, 6))
# plt.plot(df["Encoder2"].values, label="Raw")
# plt.plot(encoder2, label="Unwrapped", linestyle='--')
# plt.title("Encoder 2: Raw vs Unwrapped")
# plt.ylabel("Angle [rad]")
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()

plt.figure(figsize=(10, 6))
plt.scatter(x, y, alpha=0.6)
plt.plot(x, y_pred, color='red', label=f"LR: y = {slope:.8f}x + {intercept:.8f}")
plt.xlabel("Theta [rad]")
plt.ylabel("Torque [Nm]")
plt.title("Spring Characterization")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("output/spring_characterization.png", dpi=300)
# plt.show()

