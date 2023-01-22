#IHA'nın anlik verilerini okuma
from dronekit import connect
import time

connection_string="/dev/serial/by-id/usb-ArduPilot_CUAV-X7_320035000751303137343737-if00"
#connection_string="127.0.0.1:14550"



iha=connect(connection_string,wait_ready=True,timeout=100)

while True:

    print(iha.mode)    #Calisma Modu
    print(iha.attitude)    #Pozisyon(Pitch,Yaw,Roll)
    print(iha.battery)   #batarya level
    print("ARM: %s" %iha.armed)   #Arm durumu
    print(iha.location.local_frame)    #Konum(Latitude,Longitude,Altitude)
    print("Airspeed: %s" %iha.airspeed) #Airspeed
    print("Channel values from RC Tx:", iha.channels)
    
    print("\n")
    time.sleep(1)
