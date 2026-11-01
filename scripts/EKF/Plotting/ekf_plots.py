#!/usr/bin/env python

import argparse
from pathlib import Path
import re
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def main():
	parser = argparse.ArgumentParser(description="A script that processes a CSV file of EKF data and creates plots")
	parser.add_argument("filepath", type=Path, help="The filepath to a CSV dataset")
	args = parser.parse_args()

	if args.filepath.is_file():
		process_file(args.filepath)
	else:
		raise FileNotFoundError(f"File does not exist {args.filepath}")

def process_file(filepath: Path):
	#collect data from file
	LINE_RE = re.compile(r"received message:\s*\[?(?P<ts>\d+(?:\.\d+)?)\]?-\s*(?P<payload>.*)")
	PAIR_RE = re.compile(r"(\w+):\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")
	data = defaultdict(dict)
	with open(filepath, "r") as f:
		for line in f:
			m = LINE_RE.match(line.strip())
			if not m:
				continue
			pairs = PAIR_RE.findall(m["payload"])
			data[m["ts"]].update({k: float(v) for k, v in pairs})


	#put data into pandas dataframe to be plotted
	USE_P_MATRIX = True
	if USE_P_MATRIX:
		FIELDS = ["nV", "nA", "nE", "nG", "x", "y", "v", "a", "o", "w", "Px", "Py", "Pv", "Pa", "Po", "Pw"]  
	else :
		FIELDS = ["nV", "nA", "nE", "nG", "x", "y", "v", "a", "o", "w"]
	df = pd.DataFrame.from_dict(data, orient="index")   # rows = timestamps, columns = fields
	df.index = df.index.astype(float)                   # "84.2" -> 84.2
	df = df.sort_index().reindex(columns=FIELDS)        # sorts rows by timestamp
	df.index = df.index - df.index[0]					# subtracting the first timestamp from all timestamps so time starts at 0

	#computing uncertainty at each timestep
	if USE_P_MATRIX:
		sigma_x = np.sqrt(df["Px"])
		sigma_y = np.sqrt(df["Py"])
		sigma_v = np.sqrt(df["Pv"])
		sigma_a = np.sqrt(df["Pa"])
		sigma_theta = np.sqrt(df["Po"])
		sigma_omega = np.sqrt(df["Pw"])

	### PLOTTING ###
	PLOT_GROUP_NAME = filepath.stem

	#State
	# fig_1, axes_dict = plt.subplot_mosaic([[["Position"], ["Velocity"], ["Acceleration"], ["Heading"], ["Turn_Rate"]], ["Residual"]], sharex=True)
	fig_1, axes_dict = plt.subplot_mosaic(
		[
			["Position",     "Residual"],
			["Velocity",     "Residual"],
			["Acceleration", "Residual"],
			["Heading",      "Stats"],
			["Turn_Rate",    "Stats"],
		],
		sharex=True,
		figsize=(14, 10),
	)
	fig_1.suptitle(PLOT_GROUP_NAME + " States")
	# fig_1.tight_layout()

	#Position plot
	ax_position = axes_dict["Position"]
	ax_position.plot(df.index, df["x"], label="x")
	ax_position.plot(df.index, df["y"], label="y")
	ax_position.set_ylabel("Position (m)")
	ax_position.legend(loc="upper right")
	if USE_P_MATRIX: 
		ax_position.fill_between(df.index, df["x"] - 3*sigma_x, df["x"] + 3*sigma_x, alpha=.1, label = "±3σ")
		ax_position.fill_between(df.index, df["y"] - 3*sigma_y, df["y"] + 3*sigma_y, alpha=.1, label = "±3σ")

	#Velocity
	ax_velocity = axes_dict["Velocity"]
	ax_velocity.plot(df.index, df["v"], label="Velocity")
	ax_velocity.set_ylabel("Velocity (m/s)")
	ax_velocity.legend(loc="upper right")
	if USE_P_MATRIX: 
		ax_velocity.fill_between(df.index, df["v"] - 3*sigma_v, df["v"] + 3*sigma_v, alpha=.1, label = "±3σ")

	#Acceleration
	ax_acc = axes_dict["Acceleration"]
	ax_acc.plot(df.index, df["a"], label="Acceleration")
	ax_acc.set_ylabel("Acceleration ($m/s^2$)")
	ax_acc.legend(loc="upper right")
	if USE_P_MATRIX: 
		ax_acc.fill_between(df.index, df["a"] - 3*sigma_a, df["a"] + 3*sigma_a, alpha=.1, label = "±3σ")

	#Heading
	ax_heading = axes_dict["Heading"]
	ax_heading.plot(df.index, df["o"], label="θ")
	ax_heading.set_ylabel("Heading (rad)")
	ax_heading.legend(loc="upper right")
	if USE_P_MATRIX: 
		ax_heading.fill_between(df.index, df["o"] - 3*sigma_theta, df["o"] + 3*sigma_theta, alpha=.1, label = "±3σ")

	#Turn Rate
	ax_turnRate = axes_dict["Turn_Rate"]
	ax_turnRate.plot(df.index, df["w"], color="tab:orange", label="w")
	ax_turnRate.set_ylabel("Turn Rate (rad/s)")
	ax_turnRate.set_xlabel("Time (s)")
	ax_turnRate.legend(loc="upper right")
	if USE_P_MATRIX: 
		ax_turnRate.fill_between(df.index, df["w"] - 3*sigma_omega, df["w"] + 3*sigma_omega, alpha=.1, color="tab:orange", label = "±3σ")

	# Residual
	ax_residual = axes_dict["Residual"]
	LIM = 6
	cols = {"Velocity":"nV", "Acceleration":"nA", "ω Encoder":"nE", "ω Gyro":"nG"}
	for label, col in cols.items():
		r = df[col]
		pts = ax_residual.scatter(df.index, r.clip(-LIM, LIM), s=10, label=label)
		off = r.abs() > LIM
		ax_residual.scatter(df.index[off], np.sign(r[off]) * LIM, marker="^", s=60,
					color=pts.get_facecolor()[0], edgecolor="k")   # off-scale markers at the edge
		# print(f"{label:14s} n={len(r):4d}  mean={r.mean():+.2f}  std={r.std():.2f}  "
		# 	f"|r|>1: {(r.abs() > 1).mean():.1%}  |r|>2: {(r.abs() > 2).mean():.1%}  "
		# 	f"|r|>3: {(r.abs() > 3).mean():.1%}")
	ax_residual.axhline(3, color="k", ls="--", lw=1)
	ax_residual.axhline(-3, color="k", ls="--", lw=1)
	ax_residual.set_ylim(-LIM - 0.5, LIM + 0.5)
	ax_residual.set_ylabel("Residual / Predicted Residual")
	ax_residual.set_xlabel("Time (s)")
	ax_residual.legend(loc="upper right")
	ax_residual.tick_params(labelbottom=True)

	# Stats
	lines = []
	for label, c in cols.items():
		r = df[c].dropna()
		lines.append(f"{label:14s} n={len(r):4d}  mean={r.mean():+.2f}  std={r.std():.2f}  "
					f"|r|>1: {(r.abs() > 1).mean():5.1%}  |r|>2: {(r.abs() > 2).mean():5.1%}  "
					f"|r|>3: {(r.abs() > 3).mean():5.1%}")

	lines.append(f"{'Ideal':14s} {'':6s}  mean={0:+.2f}  std={1:.2f}  "
				f"|r|>1: {0.317:5.1%}  |r|>2: {0.046:5.1%}  |r|>3: {0.0027:5.1%}")

	text = "\n".join(lines)
	print(text)
	ax_stats = axes_dict["Stats"]
	ax_stats.axis("off")
	ax_stats.text(0.0, 0.5, text, transform=ax_stats.transAxes,
              family="monospace", fontsize=9, va="center", ha="left",
              bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

	
	#P matrix 
	if USE_P_MATRIX: 
		fig_3, axes_dict_2 = plt.subplot_mosaic([["Pxy"], ["Pv"], ["Pa"], ["Po"], ["Pw"]], sharex=True)
		fig_3.suptitle(PLOT_GROUP_NAME + " P Matrix Diagonals")
		ax_pxy = axes_dict_2["Pxy"]
		ax_pxy.plot(df.index, df["Px"], label = "$σ_x^2$")
		ax_pxy.plot(df.index, df["Py"], label = "$σ_y^2$")
		ax_pxy.set_ylabel("$σ^2$ ($m^2$)")
		ax_pxy.legend(loc="upper right")

		ax_pv = axes_dict_2["Pv"]
		ax_pv.plot(df.index, df["Pv"], label = "$σ_v^2$")
		ax_pv.set_ylabel("$σ_v^2 (m/s)^2$")
		ax_pv.legend(loc="upper right")

		ax_pa = axes_dict_2["Pa"]
		ax_pa.plot(df.index, df["Pa"], label = "$σ_a^2$")
		ax_pa.set_ylabel("$σ_a^2 (m/s^2)^2$")
		ax_pa.legend(loc="upper right")
		
		ax_po = axes_dict_2["Po"]
		ax_po.plot(df.index, df["Po"], label = "$σ_{θ}^2$")
		ax_po.set_ylabel("$σ_{θ}^2 (rad^2)$")
		ax_po.legend(loc="upper right")

		ax_pw = axes_dict_2["Pw"]
		ax_pw.plot(df.index, df["Pw"], label = "$σ_ω^2$")
		ax_pw.set_yscale("log")
		ax_pw.set_ylabel("$σ_ω^2 (rad^2/sec^2)$")
		ax_pw.set_xlabel("Time (s)")
		ax_pw.legend(loc="upper right")

	plt.show()

if __name__ == "__main__":
	main()