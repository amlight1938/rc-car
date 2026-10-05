#!/usr/bin/env python3

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
	LINE_RE = re.compile(r"received message:\s*(?P<ts>\d+(?:\.\d+)?)-\s*(?P<payload>.*)")
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
	FIELDS = ["rE", "rG", "x", "y", "o", "w", "Px", "Py", "Po", "Pw"]
	df = pd.DataFrame.from_dict(data, orient="index")   # rows = timestamps, columns = fields
	df.index = df.index.astype(float)                   # "84.2" -> 84.2
	df = df.sort_index().reindex(columns=FIELDS)        # sorts rows by timestamp
	df.index = df.index - df.index[0]					# subtracting the first timestamp from all timestamps so time starts at 0

	#computing uncertainty at each timestep
	sigma_x = np.sqrt(df["Px"])
	sigma_y = np.sqrt(df["Py"])
	sigma_theta = np.sqrt(df["Po"])
	sigma_omega = np.sqrt(df["Pw"])

	### PLOTTING ###
	PLOT_GROUP_NAME = "Small Rectangle"

	#State
	fig_1, axes_dict = plt.subplot_mosaic([["Position"], ["Heading"], ["Turn_Rate"]], sharex=True)
	fig_1.suptitle(PLOT_GROUP_NAME + " States")

	#Position plot
	ax_position = axes_dict["Position"]
	ax_position.plot(df.index, df["x"], label="x")
	ax_position.fill_between(df.index, df["x"] - 3*sigma_x, df["x"] + 3*sigma_x, alpha=.1, label = "±3σ")
	ax_position.plot(df.index, df["y"], label="y")
	ax_position.fill_between(df.index, df["y"] - 3*sigma_y, df["y"] + 3*sigma_y, alpha=.1, label = "±3σ")
	ax_position.set_ylabel("Position (m)")
	ax_position.legend()

	#Heading
	ax_heading = axes_dict["Heading"]
	ax_heading.plot(df.index, df["o"], label="theta")
	ax_heading.fill_between(df.index, df["o"] - 3*sigma_theta, df["o"] + 3*sigma_theta, alpha=.1, label = "±3σ")
	ax_heading.set_ylabel("Heading (rad)")
	ax_heading.legend(loc="upper left")

	#Turn Rate
	ax_turnRate = axes_dict["Turn_Rate"]
	ax_turnRate.plot(df.index, df["w"], color="tab:orange", label="w")
	ax_turnRate.fill_between(df.index, df["w"] - 3*sigma_omega, df["w"] + 3*sigma_omega, alpha=.1, color="tab:orange", label = "±3σ")
	ax_turnRate.set_ylabel("Turn Rate (rad/s)")
	ax_turnRate.set_xlabel("Time (s)")
	ax_turnRate.legend(loc="upper right")

	# Residual
	fig_2, ax_residual = plt.subplots(figsize=(8, 5))
	fig_2.suptitle(PLOT_GROUP_NAME + " Normalized Residual")
	ax_residual.plot(df.index, df["rE"], label="Encoder")
	ax_residual.plot(df.index, df["rG"], label="Gyro")
	ax_residual.set_ylabel("Residual / Predicted Residual")
	ax_residual.set_xlabel("Time (s)")
	ax_residual.legend(loc="upper right")

	#P matrix 
	fig_3, axes_dict_2 = plt.subplot_mosaic([["Pxy"],["Po"], ["Pw"]], sharex=True)
	fig_3.suptitle(PLOT_GROUP_NAME + " P Matrix Diagonals")
	ax_pxy = axes_dict_2["Pxy"]
	ax_pxy.plot(df.index, df["Px"], label = "$σ_x^2$")
	ax_pxy.plot(df.index, df["Py"], label = "$σ_y^2$")
	ax_pxy.set_ylabel("σ ($m^2$)")
	ax_pxy.legend()

	ax_po = axes_dict_2["Po"]
	ax_po.plot(df.index, df["Po"], label = "$σ_{theta}^2$")
	ax_po.set_ylabel("$σ_{theta}^2 (rad^2)$")
	ax_po.legend()

	ax_pw = axes_dict_2["Pw"]
	ax_pw.plot(df.index, df["Pw"], label = "$σ_ω^2$")
	ax_pw.set_yscale("log")
	ax_pw.set_ylabel("$σ_ω^2 (rad^2/sec^2)$")
	ax_pw.set_xlabel("Time (s)")
	ax_pw.legend()

	plt.show()

if __name__ == "__main__":
	main()