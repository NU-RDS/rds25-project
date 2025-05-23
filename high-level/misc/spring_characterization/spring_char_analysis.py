import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.metrics import r2_score

# Load CSV
# Encoder 2 readings correspond to shaft
# Encoder 1 readings correspond to motor
# Loadcell values are measured in grams 
df = pd.read_csv("data/sensor_log_20250522_202210.csv")

# Motor gear ratio
GEAR_RATIO = 36
SCALING_FACTOR = 9.81 * 1e-3

# Diameter of shaft (10mm)
RADIUS = 5 * 1e-3

# Degree to radian
DEG2RAD = np.pi/180

# Reference values
encoder1_ref = df["Encoder1"].iloc[0]
encoder2_ref = df["Encoder2"].iloc[0]
loadcell_ref = df["Loadcell"].iloc[0]

# Unwrap encoder1 and encoder2 values
encoder2 = np.rad2deg(np.unwrap(np.deg2rad(df["Encoder2"].values)))
encoder1 = np.rad2deg(np.unwrap(np.deg2rad(df["Encoder1"].values)))
loadcell = df["Loadcell"].values

# Compute x and y 
x = abs((encoder2 - encoder2_ref) - ((encoder1 - encoder1_ref)/GEAR_RATIO))
y = (loadcell) * SCALING_FACTOR * RADIUS

# Reshape 
x = x.reshape(-1, 1)
y = y.reshape(-1, 1)

# Linear Regression
model = LinearRegression()
model.fit(x, y)
y_lin_pred = model.predict(x)

# slope = model.coef_[0][0]
# intercept = model.intercept_[0]
r_squared_lin = r2_score(y, y_lin_pred)

print("----------------------------")
print("Linear Regression")
# print(f"slope - {slope}")
# print(f"intercept - {intercept}")
print(f"r_squared - {r_squared_lin}")

# Standardize input
scaler = StandardScaler()
x_scaled = scaler.fit_transform(x)

# Polynomial regression (degree 4)
poly = PolynomialFeatures(degree=4, include_bias=True)
x_poly = poly.fit_transform(x_scaled)

# Fit model
poly_model = LinearRegression()
poly_model.fit(x_poly, y)
y_poly_pred = poly_model.predict(x_poly)

# Extract coefficients
r_squared_poly = r2_score(y, y_poly_pred)

print("----------------------------")
print("Polynomial Regression")
print(f"r_squared - {r_squared_poly}")

# Plot 

# Plot raw encoder and wrapped encoder 
plt.figure(figsize=(10, 6))
plt.plot(df["Encoder1"].values, label="Raw")
plt.plot(encoder1, label="Unwrapped", linestyle='--')
plt.title("Encoder 1: Raw vs Unwrapped")
plt.ylabel("Angle [rad]")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot raw encoder and wrapped encoder 
plt.figure(figsize=(10, 6))
plt.plot(df["Encoder2"].values, label="Raw")
plt.plot(encoder2, label="Unwrapped", linestyle='--')
plt.title("Encoder 2: Raw vs Unwrapped")
plt.ylabel("Angle [rad]")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.scatter(x, y, alpha=0.6, s=15, color='k')
# plt.plot(x, y_lin_pred, color='green', linestyle='-', linewidth=2.0, label=f"Linear: $R^2$ - {r_squared_lin}")
# plt.plot(x, y_poly_pred, color='red', linestyle='-', linewidth=2.0, label=f"Poly: $R^2$ - {r_squared_poly}")
plt.xlabel("Angular Deflection [deg]")
plt.ylabel("Torque [Nm]")
plt.title("Spring Characterization")
# plt.legend()
plt.grid(True)
plt.tight_layout()

output_dir = "output"
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, "spring_characterization.png")
plt.savefig(output_path, dpi=300)

# plt.show()

