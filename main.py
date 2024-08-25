from loc_method import two_stage_solve_trans
from fakeGPS import simulate_GPS
import serial
import binascii
import numpy as np
import pandas as pd
import collections
import time
import warnings
warnings.filterwarnings('ignore')

def swapEndianness(hexstring):
	ba = bytearray.fromhex(hexstring)
	ba.reverse() 
	return ba.hex()

def UWB_dis():

    COM_PORT = '/dev/ttyUSB0'    
    BAUD_RATES = 57600    
    ser_UWB = serial.Serial(COM_PORT, BAUD_RATES) 
    dis_queue = collections.deque(maxlen = 1)
    t = time.time()
    rx = ser_UWB.read(264)
    rx = binascii.hexlify(rx).decode('utf-8')
    global dis_1
    global dis_2
    global dis_3
    global dis_4
    global dis
    
    if( rx != ' ' and rx.find('0241222000000000') >= 0 and rx.find('0241222000000000') <= (len(rx)-24)):
        
        dis_1_index = rx.find('0241222000000000') 
        dis_1 = rx[dis_1_index + 16 : dis_1_index + 24]
        dis_time1 = rx[dis_1_index - 18 : dis_1_index - 16]
        dis_1 = swapEndianness(dis_1)
        if dis_1 != "":
            dis_1 = int(dis_1,16)
            dis_1 = dis_1/100 
            dis_1 = round(float(dis_1),2)
        else:
           dis_1 = 0
    else:
        dis_1 = 0
    
    if( rx != ' ' and rx.find('0341222000000000') >= 0 and rx.find('0341222000000000') <= (len(rx)-24)):
        
        dis_2_index = rx.find('0341222000000000')
        dis_2 = rx[dis_2_index + 16 : dis_2_index + 24]
        dis_time2 = rx[dis_2_index - 18 : dis_2_index - 16]
        dis_2 = swapEndianness(dis_2)
        
        if dis_2 != "":
            dis_2 = int(dis_2,16)
            dis_2 = round(dis_2/100,2)
        else:
           dis_2 = 0
    else:
        dis_2 = 0
    
    if( rx != ' ' and rx.find('0441222000000000') >= 0 and rx.find('0441222000000000') <= (len(rx)-24)):
        
        dis_3_index = rx.find('0441222000000000')
        dis_3 = rx[dis_3_index + 16 : dis_3_index + 24]
        dis_time3 = rx[dis_3_index - 18 : dis_3_index - 16]
        dis_3 = swapEndianness(dis_3)
        
        if dis_3 != "":
            dis_3 = int(dis_3,16)
            dis_3 = round(dis_3/100,2)
        else:
           dis_3 = 0
    else:
        dis_3 = 0
        
    if( rx != ' ' and rx.find('0541222000000000') >= 0 and rx.find('0541222000000000') <= (len(rx)-24)):
        
        dis_4_index = rx.find('0541222000000000')
        dis_4 = rx[dis_4_index + 16 : dis_4_index + 24]
        dis_time4 = rx[dis_4_index - 18 : dis_4_index - 16]
        dis_4 = swapEndianness(dis_4)
        
        if dis_4 != "":
            dis_4 = int(dis_4,16)
            dis_4 = round(dis_4/100,2)
        else :
           dis_4 = 0
    else:
        dis_4 = 0
        
    dis = np.array([dis_1, dis_2, dis_3, dis_4])
    return dis
def _main():
    data_filename = 'UWB_distance.csv'
    
    ####### anchor info                                    
    anchor_positions = [[0, 0, 1.1],
                        [0, 4.83, 1.19],
                        [27, 4.83, 1.1],
                        [27, 0, 1.11]]
                        
    ####### info of GPS_simulate func
    # rotation angle of y-axis
    rotate_theta = 322
    # the position of the reference anchor where its ENU coordinates is (0,0,1.1)
    lon_ref = 121.5444161
    lat_ref = 25.0176494
    height = 1.1
    # udp port
    port = 25100
    #test2(rotate_theta, lon_ref, lat_ref, height, port)
    
    
    #########
    anchor_positions = np.array(anchor_positions)
    anchor_offset = anchor_positions[0]
    A = anchor_positions[1:] - anchor_offset
    u, s, vh = np.linalg.svd(A, full_matrices=True) 
    time_ls = []
    t = time.time()
    dis_to_tag_ls = []
    try:
        while(1):
            dis_to_tag = UWB_dis()
            # dis_to_tag = [0.2,1,1.414,1]
            a2_value = dis_to_tag[0] + 0.28235
            a3_value = dis_to_tag[1] + 0.20491
            a4_value = dis_to_tag[2] + 0.55185
            a5_value = dis_to_tag[3] + 0.10149
            dis_to_tag = [a2_value, a3_value, a4_value, a5_value]
            if(0 not in dis_to_tag):
                t_two_stage = time.time() - t
                time_ls.append(t_two_stage)
                dis_to_tag_ls.append(dis_to_tag)
                two_stage_result = two_stage_solve_trans(dis_to_tag, anchor_positions,u)
                
                origin_tag_pos = np.float64(two_stage_result)
                #print('type:', origin_tag_pos.dtype)
                simulate_GPS(origin_tag_pos, rotate_theta, lon_ref, lat_ref, height, port)
                # print(two_stage_result)
           
    except KeyboardInterrupt:
        time_ls = np.array(time_ls)
        time_ls = np.transpose(time_ls)
        dis_to_tag_ls = np.array(dis_to_tag_ls)
        dis_to_tag_ls = np.transpose(dis_to_tag_ls)
        df = pd.DataFrame({'time':time_ls.tolist(),
                    'A2':dis_to_tag_ls[:][0].tolist(),
                    'A3': dis_to_tag_ls[:][1].tolist(),
                    'A4': dis_to_tag_ls[:][2].tolist(),
                    'A5':dis_to_tag_ls[:][3].tolist()})
        df.to_csv(data_filename)                         

if __name__ == '__main__':  
    _main()
