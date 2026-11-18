from dynamixel_sdk import *
import glob

MOTOR_ID = 1        # <- from Wizard on Windows
BAUD = 57600        # <- from Wizard on Windows

port = glob.glob("/dev/cu.usbserial*")[0]
ph = PortHandler(port)
pk = PacketHandler(2.0)
print("open:", ph.openPort(), "baud set:", ph.setBaudRate(BAUD))

model, result, error = pk.ping(ph, MOTOR_ID)
print(pk.getTxRxResult(result), "| model:", model)
ph.closePort()