import numpy as np
import matplotlib.pyplot as plt
from dynamixel_sdk import *
import pandas as pd


#MOTOR_PORT = "COM6"
MOTOR_PORT = "/dev/cu.usbserial-FT763G3X"
BAUDRATE = 57600
MOTOR_ID = 1

class DynamixelXM430:
    # Control table adresses
    TORQUE_ON_ADD = 64 #OK
    V_LIM_ADD = 44
    V_GOAL_ADD = 104
    I_LIM_ADD = 38
    I_GOAL_ADD = 102 #OK
    POS_GOAL_ADD = 116 #OK
    OP_MOD_ADD = 11 #OK
    PRES_CURR_ADD = 126 #OK
    PRES_V_ADD = 128
    PRES_POS_ADD = 132 #OK


    # Dynamixel constants
    CURR_BASED_POS_MODE = 5

    def __init__(self, port, baudrate, motor_id, homing=True):

        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id

        # setting up the porthandler
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(2.0)

        self.portHandler.openPort()
        self.portHandler.setBaudRate(self.baudrate)

        # setting up the dynamixel 
        self.current_limit = 250 # current limit in undefined dimension, range [0;1193]
        self.current_goal = 100

        self.v_limit = 10 # in range [0;1023]
        self.v_goal = 2

        self.torqueOff()  # make sure motor is off
        self.setCurrentBasedPositionOpMode()
        self.setCurrentLimit(self.current_limit)
        self.setVLim(self.v_limit)

        self.setCurrentGoal(self.current_goal)
        self.setVGoal(self.v_goal)
        #print(f'Written goal velocity {self.getVGoal()}')

        if homing:
            print("**start homing**")
            self.torqueOn()
            _, _, _ = self.reach_positionOrCurrent(0, t_hold=0.5)
            self.torqueOff()
            print("**homing finished**")
            time.sleep(1)
            
    # functionality methods ------------------------------

    def reach_positionOrCurrent(self, position, current=None, t_hold = 3, timeout=60, t_start=None):
        """
        Reach either the goal position or the goal current, whichever happens first
        """
        epsilon_pos = 1
        epsilon_curr = 1
        if current is None:
            current = self.current_goal

        # data containers
        positions = []
        currents = []
        time_stamps = []

        self.torqueOn()
        self.setGoalPos(position)
        self.setCurrentGoal(current)

        if t_start is None: t_start = time.time()
        t_timeout_start = time.time()
        while time.time() - t_timeout_start < timeout:
            curr_position = self.getPosition()
            curr_current = self.getCurrent()

            positions.append(curr_position)
            currents.append(curr_current)
            time_stamps.append(time.time()-t_start)

            if abs(curr_position - position) < epsilon_pos:
                print("position reached")
                if t_hold > 0:
                    t_hold_start = time.time()
                    while time.time()-t_hold_start < t_hold:
                        curr_position = self.getPosition()
                        curr_current = self.getCurrent()
            
                        positions.append(curr_position)
                        currents.append(curr_current)
                        time_stamps.append(time.time()-t_start)
                        time.sleep(0.05)
                break

            if abs(curr_current) >= current:
                print("max current reached")
                if t_hold > 0:
                    t_hold_start = time.time()
                    while time.time()-t_hold_start < t_hold:
                        curr_position = self.getPosition()
                        curr_current = self.getCurrent()
            
                        positions.append(curr_position)
                        currents.append(curr_current)
                        time_stamps.append(time.time()-t_start)
                        time.sleep(0.05)
                break

            time.sleep(0.05)

        if time.time() - t_start > timeout:
            print(f"timed out {time.time() - t_start}")

        return positions, currents, time_stamps
    
    # set methods ------------------------------

    def torqueOn(self):
            self._write1Byte(self.TORQUE_ON_ADD, 1)
    
    def torqueOff(self):
        self._write1Byte(self.TORQUE_ON_ADD, 0)

    def setCurrentLimit(self, current_lim):
        self._write2Bytes(self.I_LIM_ADD, current_lim)

    def setCurrentGoal(self, current_goal):
        self._write2Bytes(self.I_GOAL_ADD, current_goal)

    def setGoalPos(self, goal_pos):
        self._write4Bytes(self.POS_GOAL_ADD, goal_pos)

    def setVLim(self, v_lim):
        self._write4Bytes(self.V_LIM_ADD, v_lim)

    def setVGoal(self, goal_vel):
        self._write4Bytes(self.V_GOAL_ADD, goal_vel)

    def setCurrentBasedPositionOpMode(self):
        self._write1Byte(self.OP_MOD_ADD, self.CURR_BASED_POS_MODE)

    # get methods ---------------------------------------------------------
    def getCurrent(self):
        raw =  self._read2Bytes(self.PRES_CURR_ADD)

        if raw >= 2**15:
            raw -= 2**16
        return raw

    def getGoalCurrent(self):
        raw =  self._read2Bytes(self.I_GOAL_ADD)

        if raw >= 2**15:
            raw -= 2**16
        return raw

    def getPosition(self):
        raw = self._read4Bytes(self.PRES_POS_ADD)

        if raw >= 2**31:
            raw -= 2**32
        return raw

    def getGoalPosition(self):
        raw = self._read4Bytes(self.POS_GOAL_ADD)

        if raw >= 2**31:
            raw -= 2**32
        return raw
    
    def getVelocity(self):
        return self._read4Bytes(self.PRES_V_ADD)

    def getVGoal(self):
        return self._read4Bytes(self.V_GOAL_ADD)

    # private methods ------------------------------------

    def _write1Byte(self, address, value):
        self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_id, address, value)
    
    def _write2Bytes(self, address, value):
        self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_id, address, value)
    
    def _write4Bytes(self, address, value):
        self.packetHandler.write4ByteTxRx(self.portHandler, self.motor_id, address, value)

    def _read1Byte(self, address):
        value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.motor_id, address)

        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))

        return value

    def _read2Bytes(self, address):
        value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_id, address)
        
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))

        return value

    def _read4Bytes(self, address):
        value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.motor_id, address)
                
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))

        return value

# functions --------------------------------------------------------------

def cycical_movement(motor, N_cycles, goal_pos, current_goal, t_hold=0.5, t_pause=0.5):

    currents = []
    positions = []
    time_stamps = []

    motor.torqueOn()

    t_start = time.time()
    for i in range(N_cycles):
        print(f'cycle {i+1}/{N_cycles}')
        temp_pos, temp_curr, temp_time= motor.reach_positionOrCurrent(goal_pos, current_goal, t_hold=t_hold, t_start=t_start)
        currents.extend(temp_curr)
        positions.extend(temp_pos)
        time_stamps.extend(temp_time)

        temp_pos, temp_curr, temp_time= motor.reach_positionOrCurrent(0, current_goal, t_hold=t_hold, t_start=t_start)
        currents.extend(temp_curr)
        positions.extend(temp_pos)
        time_stamps.extend(temp_time)

        t_pause_start = time.time()
      
        while time.time()-t_pause_start < t_pause:
            curr = motor.getCurrent()
            pos = motor.getPosition()

            currents.append(curr)
            positions.append(pos)
            time_stamps.append(time.time()-t_start)

            time.sleep(0.05)

    motor.torqueOff()

    return currents, positions, time_stamps

# core code ------------------------------------------------------------------------------------

motor = DynamixelXM430(MOTOR_PORT, BAUDRATE, MOTOR_ID, homing=True)

#currents, positions, time_stamps = cycical_movement(motor, N_cycles=10, goal_pos=3000, current_goal=250)

data = pd.DataFrame({
    "current": currents, 
    "position": positions,
    "time": time_stamps
})

data.to_excel("FlexionTest_FreeHeatShrink.xlsx", index=False)

fig, ax = plt.subplots(3,1, figsize=(10,8))
ax[0].plot(positions, currents)
ax[0].set_title('position vs current')
ax[0].set_xlabel('position')
ax[0].set_ylabel('current')

ax[1].plot(time_stamps, positions)
ax[1].set_title('position vs time')
ax[1].set_xlabel('time in [s]')
ax[1].set_ylabel('position')

ax[2].plot(time_stamps, currents)
ax[2].set_title('currents vs time')
ax[2].set_xlabel('time in [s]')
ax[2].set_ylabel('current')

plt.show()

