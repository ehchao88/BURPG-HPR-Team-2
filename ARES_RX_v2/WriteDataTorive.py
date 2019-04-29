import serial
import sys
import signal

serialport = serial.Serial("/dev/ttyACM2", 9600, timeout = 0.5)

start = False;
start_string = raw_input("type 's' and press enter to start receiving\n")
if(start_string == 's'):
    start = True
f = open("data.txt", "w+")

def signal_handler(signal, frame):
    print("Exiting...\n")
    f.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

while (start == True):
    command = serialport.read()
    print(command)
    if(command == "complete"):
        start = False
    f.write(command)

print("Finished collecting data")
f.close()
