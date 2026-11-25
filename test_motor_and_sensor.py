from dynamixel_sdk import *
import glob

port = glob.glob("/dev/cu.usbserial*")[0]
ph = PortHandler(port)
pk = PacketHandler(2.0)
print("open:", ph.openPort(), "baud set:", ph.setBaudRate(57600))

model, result, error = pk.ping(ph, 1)
print("ping:", pk.getTxRxResult(result), "| model:", model)

result, error = pk.write1ByteTxRx(ph, 1, 64, 0)     # torque off
print("write:", pk.getTxRxResult(result), "| error:", error)

data, result, error = pk.readTxRx(ph, 1, 70, 1)     # hardware error status
print("read:", pk.getTxRxResult(result), "| data:", data, "| error:", error)
ph.closePort()