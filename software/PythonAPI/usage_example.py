from open_micro_stage_api import OpenMicroStageInterface

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM8')

# run this once to calibrate joints
# for i in range(3): oms.calibrate_joint(i, save_result=True)

# home device
# oms.home()
oms.enable_motors(True)
# move and wait
# oms.move_to(x=0, y=0, z=0, f=10)
# # x+ -> x+, y+ -> y+, z+ -> z-
# oms.wait_for_stop()

# # print some info
# oms.read_device_state_info()