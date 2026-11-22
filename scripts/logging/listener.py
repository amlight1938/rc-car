from pathlib import Path
import socket

PORT = 8090
#used to create log file from messages
# script_dir = Path(__file__).resolve().parent

try:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind(("", PORT))
	sock.settimeout(1.0)

	# with open(script_dir.joinpath("log.txt"), mode="w") as file:
	while True:
		try:
			data, address = sock.recvfrom(1024)
			print(f"received message: {data.decode()}", flush=True)
				# file.write(data.decode() + '\n')
				# file.flush()
		except socket.timeout:
			continue  # no packet arrived in timeout, loop and check for Ctrl+C again

except KeyboardInterrupt:
	print("User closed program")