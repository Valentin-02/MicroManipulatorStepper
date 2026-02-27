from open_micro_stage_api import OpenMicroStageInterface
from calibration_plotter import calibrate_and_plot

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM8')

# Kalibriere die Gelenke und plotte die Ergebnisse
calibrate_and_plot(oms)

# Schalte Motoren aus
oms.enable_motors(enable=False)