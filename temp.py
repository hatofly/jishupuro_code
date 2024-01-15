#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#path: /home/mech-user/enshu_ws/src/robot-programming/dynamixel_7dof_arm/scripts
import sys
from optparse import OptionParser
import time
import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io,dynamixel_const
# dynamixel_ioに全ての入出力関係が入っている

if __name__ == '__main__':
    parser = OptionParser(usage='Usage: %prog [options]', description='Changes the unique ID of a Dynamixel servo motor.')
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-f', '--from-id', metavar='FROM_ID', type="int", default=1,
                      help='from id [default: %default]')
    parser.add_option('-t', '--to-id', metavar='TO_ID', type="int", default=7,
                      help='to id [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 1:
        parser.print_help()
        exit(1)
    options.from_id = 4
    options.to_id = 7
    options.port = "/dev/dynamixel_arm"
    port = options.port
    baudrate = options.baud
    init_pos = [0,1023,1023,0]
    reverse_list = [5,6]
    #init_posは連続回転モードでない場合0~1023でおよそ300度を分割している
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR:', soe)
    else:
        for idx in [x + options.from_id for x in range(options.to_id - options.from_id + 1)]:
            print('Scanning %d...' %(idx), end=' ')
            if dxl_io.ping(idx):
                resp=dxl_io.set_position(idx,init_pos[idx-options.from_id])
                print('The motor %d respond to a ping and set to initial pos' %(idx))
                #resp = dxl_io.set_speed(idx,200)
                # set_speedで設定された速度でset_positionの位置までいくみたい！
                #dxl_io.write(idx, dynamixel_const.DXL_GOAL_POSITION_L, (20, 1))
            else:
                print('ERROR: The specified motor did not respond to id %d.' % idx)

# rosrun dynamixel_7dof_arm temp.py /dev/dynamixel_arm -f 4 -t 7
#　とすることで1~3をスキップできるよ！
    # ここからはinteractive mode
    debug = False
    if debug:
        while True:
            idx,pos = map(int,input().split())
            if not (options.from_id <= idx <= options.to_id):
                print("illegal ID")
                continue
            dxl_io.set_position(idx,pos)
        
    def motion_parser(x,lr):
        # lr-> 1:left 2:right
        if lr == 1:
            thigh = 7
            calf = 6
        elif lr == 2:
            thigh = 4
            calf = 5
        else:
            print("illegal lr")
            return
        
        if x[0]=='t':
            if thigh in reverse_list:
                dxl_io.set_position(thigh,1023-x[1])
            else:
                dxl_io.set_position(thigh,x[1])
        elif x[0]=='c':
            if calf in reverse_list:
                dxl_io.set_position(calf,1023-x[1])
            else:
                dxl_io.set_position(calf,x[1])
        elif x[0]=='s':
            time.sleep(x[1])
        else:
            print("illegal x[0]")


    def walk_controller(times,direction):
        # t: thigh, c: calf, s: sleep (sleep time is in [sec])
        # direction-> 0:front 1:left 2:right
        pattern = [
            ('t',0),
            ('s',1.5),
            ('c',0),
            ('s',1.5),
            ('t',1000),
            ('s',1.5),
            ('c',800),
            ('s',1.5)
        ]
        phase_diff = int(len(pattern)/2)
        # 左右の脚でリストの読込にどういう差を設けるか
        for i in range(times):
            for j in range(len(pattern)):
                if direction == 0:
                    motion_parser(pattern[j],1)
                    motion_parser(pattern[(j+phase_diff)%len(pattern)],2)
                elif direction == 1:
                    motion_parser(pattern[j],1)
                    motion_parser(pattern[-j-1],2)
                elif direction == 2:
                    motion_parser(pattern[j],2)
                    motion_parser(pattern[-j-1],1)
                else:
                    print("illegal direction")
                    return
    times,direction = map(int,input("input walk count and direction \n").split())
    walk_controller(times,direction)
                    
                
        

