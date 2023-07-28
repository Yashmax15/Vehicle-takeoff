import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time

vehicle= utility.mavlink_connection(device= "127.0.0.1:14560")

vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=1,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

vehicle_disarm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

set_mode_message_Guided = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=4,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

set_mode_message_Stabilize= dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

takeoff_command = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_NAV_TAKEOFF,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=10
)

land_command = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_NAV_LAND,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

vehicle.wait_heartbeat()

vehicle.mav.send(set_mode_message_Stabilize)

vehicle.mav.send(vehicle_arm_message)

while True:
    message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)
    message = message.to_dict()
    print(message["command"])
    if message["result"] == dialect.MAV_RESULT_ACCEPTED and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM:
        break

time.sleep(1) #one sec before takeoff

vehicle.mav.send(set_mode_message_Guided)

vehicle.mav.send(takeoff_command)

while True:
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)
    message = message.to_dict()
    relative_altitude = message["relative_alt"] * 1e-3
    print(relative_altitude)
    if (10-relative_altitude) <=0:
        print(relative_altitude)
        break

for i in range(10):
    time.sleep(1)
    print("Hovering")

vehicle.mav.send(land_command)

while True:
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)
    message = message.to_dict()
    relative_altitude = message["relative_alt"] * 1e-3
    print(relative_altitude)
    if relative_altitude <1:
        break

time.sleep(1) #one sec before disarm

vehicle.mav.send(vehicle_disarm_message)

while True:
    message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)
    message = message.to_dict()
    if message["result"] == dialect.MAV_RESULT_ACCEPTED and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM:
        exit(1)