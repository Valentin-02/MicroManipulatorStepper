from open_micro_stage_api import OpenMicroStageInterface
import numpy as np

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM8')

positions = [(x, x, x) for x in np.arange(-10, 9.5, 0.1)]
oms.calibrate_angle_mapping(positions_list=positions)

oms.enable_motors(False)
input("Warte...")

oms.enable_motors(True)
oms.move_to_current_angle()
