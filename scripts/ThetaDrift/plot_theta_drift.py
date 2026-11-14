#!/usr/bin/env python
import re, sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
 
rows = []
for line in open(sys.argv[1]):
	m = re.search(r"\[([\d.]+)\]-(.*)", line)
	if m:
		rows.append({"t": float(m[1]), **{k: float(v) for k, v in re.findall(r"(\w+):\s*(-?[\d.]+)", m[2])}})
 
df = pd.DataFrame(rows).set_index("t")
df.index = df.index - df.index[0]
 
# line of best fit (slope of an angular velocity is in rad/s^2)
slope_wg, intercept_wg = np.polyfit(df.index, df["wg"], 1)
print(f"omega gyro fit:    slope {slope_wg:.6f} rad/s^2, mean {df['wg'].mean():.6f} rad/s")
slope_we, intercept_we = np.polyfit(df.index, df["we"], 1)
print(f"omega encoder fit: slope {slope_we:.6f} rad/s^2, mean {df['we'].mean():.6f} rad/s")
 
fig, ax = plt.subplots()
ax.plot(df.index, df["wg"], "r-", label="wg (gyro)")
ax.plot(df.index, df["we"], "b-", label="we (encoder)")
ax.plot(df.index, slope_wg * df.index + intercept_wg, "g--", label=f"wg fit: {slope_wg:.2e} rad/s²")
ax.plot(df.index, slope_we * df.index + intercept_we, "b--", label=f"we fit: {slope_we:.2e} rad/s²")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angular velocity (rad/s)")
ax.legend(loc="best")
plt.show()