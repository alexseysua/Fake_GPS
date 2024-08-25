import numpy as np
from math import *
import pymap3d as pm
import socket
import json
import time
# import matplotlib.pyplot as plt
######## function ##########

def simulate_GPS(origin_tag_pos, rotate_theta, lon_ref, lat_ref, height, port):
	# if y-axis rotates to the north direction (new coordinates)
	# then the position origin_pos may be reprsented as (east_ENU, north_ENU, z) on the new coordinates
	x = origin_tag_pos[0]
	y = origin_tag_pos[1]
	up_ENU = origin_tag_pos[2]

	unit_degree = pi/180

	theta = rotate_theta*unit_degree

	east_ENU = x*cos(theta) + y*sin(theta)
	north_ENU = -x*sin(theta) + y*cos(theta)

	tag_lat, tag_lon, __ = pm.enu2geodetic(east_ENU,north_ENU,up_ENU,lat_ref,lon_ref,height)

	# print(x,y)
	# print(east_ENU, north_ENU)
	# print(tag_lat, tag_lon)

	# put the information of lontitude and latitude to the GPS packet
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
	out_addr = ("127.0.0.1", port)

	data = {
	            'time_usec' : 0,                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
	            'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
	            'ignore_flags' : 8 | 16 | 32,           # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
	            'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
	            'time_week' : 0,                        # (uint16_t) GPS week number
	            'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	            'lat' : 354100000,                          # (int32_t) Latitude (WGS84), in degrees * 1E7
	            'lon' : 1394200000,                         # (int32_t) Longitude (WGS84), in degrees * 1E7
	            'alt' : 10,                             # (float) Altitude (AMSL, not WGS84), in m (positive for up)
	            'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
	            'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
	            'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
	            've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
	            'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
	            'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
	            'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
	            'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
	            'satellites_visible' : 7                # (uint8_t) Number of satellites visible.
	      }


	data['lon'] = int(np.round(tag_lon*10**7))
	data['lat'] = int(np.round(tag_lat*10**7))
	data['alt'] = up_ENU
	
	out_data = json.dumps(data)
	s.sendto(out_data.encode(), out_addr)
	print("send tag info: ", data['lon'], data['lat'], data['alt'])
	time.sleep(0.15)

############# test function #############


# test1
def test1(origin_tag_pos, rotate_theta, lon_ref, lat_ref, height, port):
	while(1):

		simulate_GPS(origin_tag_pos, rotate_theta, lon_ref, lat_ref, height, port)

# test2


def test2(rotate_theta, lon_ref, lat_ref, height, port):
	distance = [20.95, 3.21, 20.95, 3.21] # 0 -> 1 -> 3 -> 2 -> 0
	point = [[2.63,0.81, 1.93], [23.58,0.81, 1.93], [23.58, 4.02, 1.93], [2.63,4.02, 1.93], [2.63,0.81, 1.93]]
	pos_send_time = 0.15 # 0.15 s / one pkt is sent to drone 
	x_pos = []
	y_pos = []

	# fig, ax = plt.subplots(figsize=(9, 12))

	# create simulated ENU points of x & y
	for i in range(len(distance)):

	      step = int(np.round(distance[i]/pos_send_time))
	      print(i,i+1,step)

	      x_pos.append(np.linspace(point[i][0], point[i+1][0], step))
	      y_pos.append(np.linspace(point[i][1], point[i+1][1], step))

	lon_ls = []
	lat_ls = []

	for i in range(len(x_pos)):
		print('Start flying on PATH %d'%(i))

		for j in range(len(x_pos[i])): 

			# print(x_pos[i][j])
			# print(y_pos[i][j])

			east_ENU = x_pos[i][j]
			north_ENU = y_pos[i][j]
			up_ENU = 1.93
			origin_tag_pos = np.array([east_ENU, north_ENU, up_ENU])
			print('type:',origin_tag_pos.dtype)
			simulate_GPS(origin_tag_pos, rotate_theta, lon_ref, lat_ref, height, port)
			
			# lat_out, lon_out, __ = pm.enu2geodetic(east_ENU,north_ENU,up_ENU,lat_ref,lon_ref,height)

			# lon_ls.append( int(np.round(lon_out*10**7)) )
			# lat_ls.append( int(np.round(lat_out*10**7)) )

	# plt.plot(lon_ls, lat_ls, 'b-', label= 'python func')
	# ax.legend()
	# plt.savefig('test_ENU.png')

def main():

	########## input data ##########
	# the tag position determined by 2-stage
	origin_tag_pos = np.array([1,2,1])

	# rotation angle of y-axis
	rotate_theta = 322
	# the position of the reference anchor where its ENU coordinates is (0,0,1.1)
	lon_ref = 121.5444161
	lat_ref = 25.0176494
	height = 1.1

	# udp port
	port = 25100
	test2(rotate_theta, lon_ref, lat_ref, height, port)

if __name__ == "__main__":
	main()
