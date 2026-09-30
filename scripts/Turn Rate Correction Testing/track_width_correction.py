#!/usr/bin/env python3

import argparse
from pathlib import Path
import pandas as pd

GYRO_Z_BIAS = -0.724126

def main():
	parser = argparse.ArgumentParser(description="A script that processes a CSV file and calculates variance")
	parser.add_argument("filepath", type=Path, help="The filepath to a CSV dataset")
	args = parser.parse_args()

	if args.filepath.is_file():
		calculate_correction_factor(args.filepath)
	else:
		raise FileNotFoundError(f"File does not exist {args.filepath}")

def calculate_correction_factor(filepath: Path):
	df = pd.read_csv(filepath)

	df["correction_factor]"] = df["calculated_omega"] / (df["gryro_z"] - GYRO_Z_BIAS)
	correction_average = df["correction_factor"].mean()
	print("Average correction factor:")
	print(correction_average)

if __name__ == "__main__":
	main()