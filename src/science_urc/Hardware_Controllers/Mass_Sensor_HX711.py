#####################################################################################################################
# Importing Program Libraries
#   - time:
#       - Adds delays to the program
#   - hx711:
#       - Provides the HX711 class for communicating with the load cell amplifier
#       - Enables reading raw weight data from the sensor through GPIO pins
#####################################################################################################################

import time
from hx711 import HX711




#####################################################################################################################
# Declaring Program Variables
#   - GPIO pin assignments for HX711 communication
#   - DT_PIN (Data pin) receives output from HX711
#   - SCK_PIN (Clock pin) sends timing pulses to HX711
#####################################################################################################################

DT_PIN = 27
SCK_PIN = 17




#####################################################################################################################
# Initializing HX711 Object
#   - Create an HX711 instance using defined GPIO pins
#   - The HX711 interfaces with a load cell to measure force/weight
#####################################################################################################################

hx = HX711(DT_PIN, SCK_PIN)




#####################################################################################################################
# Taring the Load Cell
#   - Removes any offset from the scale by setting the current reading to zero
#   - Ensures accurate measurements after calibration
#####################################################################################################################

print("Taring... Please remove all weight from the load cell.")
hx.tare()
print("Tare complete.")




#####################################################################################################################
# Reading Weight in a Continuous Loop
#   - Collects averaged weight data from the HX711
#   - Prints weight in grams (formatted to two decimal places)
#   - Powers the HX711 down/up between reads to reduce drift
#   - Runs until manually stopped with a keyboard interrupt (CTRL+C)
#####################################################################################################################

try:
    while True:
        weight = hx.get_weight(5)  # Average of 5 samples for stability
        print(f"Weight: {weight:.2f} g")
        hx.power_down()
        hx.power_up()
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting... Program stopped.")