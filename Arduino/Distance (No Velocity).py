import math

while True:
    tof = float(input("Input the distance value fo the TOF: "))
    angle_deg = float(input("Input the angle from the IMU in Degrees: "))
    if angle_deg >= 45 and angle_deg <= 180:
        angle_rad = math.radians(angle_deg)
        #print(math.cos(angle_rad))
        #print(angle_rad)
        d = tof / math.cos(angle_rad)
        print ("Distance: ", str(d))
    else:
        break