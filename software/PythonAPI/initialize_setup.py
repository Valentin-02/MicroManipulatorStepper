from open_micro_stage_api import OpenMicroStageInterface
import numpy as np

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM8')

oms.enable_motors(False)
input("Warte...")

oms.enable_motors(True)
oms.move_to_current_angle()
