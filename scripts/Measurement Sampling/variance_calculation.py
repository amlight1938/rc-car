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

	variances = df.var()
	print("Variances:")
	print(variances)

if __name__ == "__main__":
	main()