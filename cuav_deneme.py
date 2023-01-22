# coding: utf8
#otomatik ucus icin ucus kontrol kartina rota olusturmak

from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import math

try:
	iha = connect("/dev/serial/by-id/usb-ArduPilot_CUAV-X7_320035000751303137343737-if00", wait_ready=True, timeout = 100)
except:
    print("iha ya baglanma hatasi !!!!")
    
def takeoff(irtifa):
    while iha.is_armable is not True:
        print("iha arm edilebilir durumda degil.")
        time.sleep(1)


    print("iha arm edilebilir.")
    
    iha.mode = VehicleMode("GUIDED")

    iha.armed = True

    while iha.armed is not True:
        print("iha arm ediliyor...")
        #iha.armed = True                 İkinci deneme kodunda çalıştır
        time.sleep(1)

    print("iha arm edildi.")

    iha.simple_takeoff(irtifa)
    
    #while iha.location.global_relative_frame.alt < irtifa * 0.9:
    #    print("iha hedefe yukseliyor.")
    #    time.sleep(1)



def gorev_ekle():
	global komut
	komut = iha.commands

	komut.clear()
	time.sleep(1)
    

    # TAKEOFF
    
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 1))
	
	#SPEED
	#komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 0, 10, 0, 0, 0, 0, 0))	
	
    # WAYPOINT
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,   38.7107679 , 35.5356592 , 1))
	print(" irtifa = {}".format(iha.location.global_relative_frame.alt))
	
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,   38.7105712, 35.5355734, 1))
	
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.7104791, 35.5359811 , 1))
	
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  38.7106821, 35.5360615 , 1))
	
    # DOĞRULAMA
	komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

	komut.upload()
	print("Komutlar yukleniyor...")


takeoff(1)

gorev_ekle()


iha.mode = VehicleMode("AUTO")


################## iki nokta arasindaki mesafe fonksiyonu######################################
def get_distance_metres(aLocation1, aLocation2):
 
    xlat = aLocation2.lat - aLocation1.lat
    xlong = aLocation2.lon - aLocation1.lon
    print("Gelecek nokataya son: {} Metre".format(round(math.sqrt((xlat*xlat) + (xlong*xlong)) * 1.113195e5, 2)))
###############################################################################################




######################### aradaki mesafe ##################################

komut.next = 0

while True:
	next_waypoint = komut.next	
	
	print("Siradaki komut {}".format(next_waypoint))
	time.sleep(1)
	# iki nokta arasindaki mesafenin hesaplanmasinda kulllanin değişkenlerin similasyondan çekilmesi
	missionitem=iha.commands[next_waypoint-1] 
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z
	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(iha.location.global_frame, targetWaypointLocation)

	if next_waypoint is 4:
		print("Gorev bitti.")
		break


print("Döngüden cikildi.")
























