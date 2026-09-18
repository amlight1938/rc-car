## Logging Architecture
The Logger is a singleton which means there is one logging object shared between all classes throughout the entirety of the car's lifecycle. When each class logs a message, that
message will be sent to a queue, belonging to the Logger class and named logQueue. The Logger class creates and registers a logging task with FreeRTOS. Any time other tasks with 
higher priority, like controlTask, are idle, this task will take log messages off of the queue and send them via UDP to a specific IP address and port
(`WifiCredentials.cpp.example` instructions). Since the `WifiCredentials.cpp` will have sensitive information it has been added to the `.gitignore` but care should still be taken when pushing
to the remote repo.

The messages are sent by the car but the receiving IP address must be listening on the designated port in order to receive the logs. This is what `listener.py` does. When running,
it will collect and print out to console any mesages that arrived on the designated port.

## Setting up remote logging

1) Create and complete `WifiCredentials.cpp` by copying and renaming `WifiCredentials.cpp.example and filling out wifi information
2) Make sure the IP address and port number in `listener.py` is correct
3) Run `listener.py` in a command terminal using `python3 <path_to_listener.py>`
4) Turn on the car