#!/usr/bin/env python3

import argparse
from pathlib import Path
import pandas as pd

def main():
	parser = argparse.ArgumentParser(description="A script that processes a CSV file and calculates variance")
	parser.add_argument("filepath", type=Path, help="The filepath to a CSV dataset")
	args = parser.parse_args()

	if args.filepath.is_file():
		process_file(args.filepath)
	else:
		raise FileNotFoundError(f"File does not exist {args.filepath}")

def process_file(filepath: Path):
	df = pd.read_csv(filepath)
	df.columns = df.columns.str.strip()

	### Used for deriving variance of encoder omega. Will need to subtract the GyroZ variance from this to get encoder omega
	if "w" in df.keys() and "GZ" in df.keys():
		df["residual"] = df["w"] - df["GZ"]
		residual_variance = df["residual"].var() 
		print("Residual Variance:")
		print(residual_variance)

	### Used for variance of GyroZ omega, will give variance of encoder omega but do ot trust it. Use code above for that
	# variances = df.var()
	# bias = df.mean()
	# print("Variances:")
	# print(variances)
	# print("Biases:")
	# print(bias)

if __name__ == "__main__":
	main()