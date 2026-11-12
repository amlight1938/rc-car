#!/usr/bin/env python

import argparse
from pathlib import Path
import re
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# this script can either take a single file as input or a folder that contains multiple data files to plot them all on the same axes for comparison

FIELDS = ["PidOutput", "EkfTheta"] # the fields in the data files to read

def main():
	parser = argparse.ArgumentParser(description="A script that processes a CSV file of EKF data and creates plots")
	parser.add_argument("filepath", type=Path, help="The filepath to a CSV dataset or directory containing csv datasets")
	args = parser.parse_args()

	if args.filepath.is_file():
		title, df = process_file(args.filepath)
		plot_single_ax(title, df)
	elif args.filepath.is_dir():
		files = [f for f in args.filepath.iterdir() if f.is_file() and f.suffix == '.txt']
		dfs: list[tuple[str, pd.DataFrame]] = []
		for file in files:
			dfs.append(process_file(file))
		plot_all_in_dir(dfs)
	else:
		raise FileNotFoundError(f"File does not exist {args.filepath}")

# read the data file and place the data in a pandas dataframe to be used for plotting
def process_file(filepath: Path) -> tuple[str, pd.DataFrame]:
	#collect data from file
	LINE_RE = re.compile(r"(?:received message:\s*)?\[?(?P<ts>\d+(?:\.\d+)?)\]?(?:-\s*|\s+)(?P<payload>.*)")
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
	df = pd.DataFrame.from_dict(data, orient="index")   # rows = timestamps, columns = fields
	df.index = df.index.astype(float)                   # "84.2" -> 84.2
	df = df.sort_index().reindex(columns=FIELDS)        # sorts rows by timestamp
	df.index = df.index - df.index[0]					# subtracting the first timestamp from all timestamps so time starts at 0
	return (get_gains_from_filepath(filepath), df)

# plot a single data file
def plot_single_ax(title: str, df: pd.DataFrame):
	ax = df.plot(y=FIELDS[1], kind='line', label="Angle")
	t = 90 * np.pi / 180
	ax.axhline(y= t,  color="red",   linestyle="-",  linewidth=1.5, alpha=0.3, label="Target")
	ax.axhline(y= t*1.1, color="green", linestyle="--", linewidth=1.5, alpha=0.3, label="10% over/undershoot")
	ax.axhline(y= t - t*.1, color="green", linestyle="--", linewidth=1.5, alpha=0.3, label=None)
	ax.legend(loc="lower right")
	ax.set_ylabel("Angle (rad)")
	ax.set_xlabel("Time (s)")
	ax.set_title(title)
	plt.show()

# plot multiple runs on one plot for comparison
def plot_all_in_dir(dfs: list[tuple[str,pd.DataFrame]]):
	fig, ax = plt.subplots()
	for title, df in dfs:
		df.plot(y=FIELDS[1], ax=ax, kind='line', label=title)
	t = 90 * np.pi / 180
	ax.axhline(y= t,  color="red",   linestyle="-",  linewidth=1.5, alpha=0.3, label="Target")
	ax.axhline(y= t*1.1, color="green", linestyle="--", linewidth=1.5, alpha=0.3, label="10% over/undershoot")
	ax.axhline(y= t - t*.1, color="green", linestyle="--", linewidth=1.5, alpha=0.3, label=None)
	ax.legend(loc="lower right")
	ax.set_ylabel("Angle (rad)")
	ax.set_xlabel("Time (s)")
	ax.set_title("Varying Kp, Ki, Kd Gains for a Target Velocity")
	fig.tight_layout()
	plt.show()

def get_gains_from_filepath(filepath: Path) -> str:
	kp, ki, kd = (part.replace("p", ".") for part in filepath.stem.split("_"))
	return f"{kp} Kp, {ki} Ki, {kd} Kd"

if __name__ == "__main__":
	main()