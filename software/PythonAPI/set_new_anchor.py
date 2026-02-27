from open_micro_stage_api import OpenMicroStageInterface
from calibration_plotter import calibrate_and_plot

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM8')

oms.enable_motors(False)
input("Richte den neuen Ankerpunkt aus...")
oms.set_anchor_point()

# Aktiviere Motoren
oms.enable_motors(enable=True)

# Move Position to 0 0 0
oms.move_to(0, 0, 0, f=10)
# oms.wait_for_stop()

# # print some info
# oms.read_device_state_info()