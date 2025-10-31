#! /usr/bin/env python
#LDS 距離資訊讀取 安裝空間資訊讀取 Sensor特性讀取
#Sonar 距離資訊讀取 安裝空間資訊讀取
#馬達控制操作
#Odometry 資訊讀取
#電池資訊讀取 電量 是否充電中
#export LD_LIBRARY_PATH=/home/pyuser/client_lib
import math
import signal
import time
import numpy as np
from ctypes import *
from playercpp import *
from playerc import *
import ctypes

########################################################################
IsRunProgram=True;

########################################################################
if __name__ == "__main__":
    # Create a client object and connect it
    robot = PlayerClient('localhost')
    #robot = PlayerClient('172.16.114.217')


    # Create a proxy for laser:0~3
    # Create a proxy for dio:0
    lp0 = LaserProxy(robot, 0);
    lp1 = LaserProxy(robot, 1);
    dp = DioProxy(robot, 0);
    p2d = Position2dProxy(robot, 0);
    power = PowerProxy(robot,0);
    dis=DispatcherProxy(robot,0);

    try:
        print("program start\n");

        # Retrieve the pose of the laser with respect to its parent
        lp0.RequestGeom();
        pose = lp0.GetPose();
        lp0.RequestConfigure();
        print('Laser[%d] MaxAngle=%f MinAngle=%f Count=%d scanRes:%.3f pose:(px=%.3f,py=%.3f,pz=%.3f,proll=%.3f,ppitch=%.3f,pyaw=%.3f)'
              % (0, lp0.GetMaxAngle(), lp0.GetMinAngle(), lp0.GetCount() , lp0.GetScanRes(), pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw))

        lp1.RequestGeom();
        pose = lp1.GetPose();
        lp1.RequestConfigure();
        scanRes = lp1.GetScanRes();
        print('Laser[%d] MaxAngle=%f MinAngle=%f Count=%d scanRes:%.3f pose:(px=%.3f,py=%.3f,pz=%.3f,proll=%.3f,ppitch=%.3f,pyaw=%.3f)'
              % (1, lp1.GetMaxAngle(), lp1.GetMinAngle(), lp1.GetCount() , lp1.GetScanRes(), pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw))



        print("laser on\n")
        #dp.SetOutput(1,1); # LDS power enable
        lp0.Enable(True);  # LDS star
        lp1.Enable(True);  # LDS star
        dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_NOT_CONTROL_CHASSIS).value) 
        while (True):
            if ( robot.Peek(300)==True ):
                robot.Read();
                #check if laser0 data in
                if ( lp0.IsFresh()==True ):
                    lp0.NotFresh()
                    dd=lp0.GetRangeVec()
                    r = np.array(dd)
                    index=np.where(r>0.1)
                    for i in index:
                        r[i]=0
                    #print(r)
                    th=np.linspace(lp0.GetMinAngle(), lp0.GetMaxAngle(), num=lp0.GetCount())
                    print("0",len(r),len(th))
                    x=r*np.cos(th)  
                    y=r*np.sin(th)  
                    
                if ( lp1.IsFresh()==True ):
                    lp1.NotFresh()
                    dd=lp1.GetRangeVec()
                    r = np.array(dd)
                    index=np.where(r>0.1)
                    for i in index:
                        r[i]=0
                    #print(r)
                    th=np.linspace(lp1.GetMinAngle(), lp1.GetMaxAngle(), num=lp1.GetCount())
                    print("1",len(r),len(th))
                    x=r*np.cos(th)  
                    y=r*np.sin(th) 
                    
                if ( lp1.IsFresh()==True ):
                    lp1.NotFresh();
                if ( p2d.IsFresh()==True ):
                    p2d.NotFresh();
                    #print('t=%4.3f X=%f<m> Y=%f<m> theat=%f<rad> VX=%f<m/s> VY=%f<m/s> Vtheat=%f<rad/s> stall=%d lost=%f status=%d' % ( p2d.GetDataTime(), p2d.GetXPos() , p2d.GetYPos(), p2d.GetYaw(), p2d.GetXSpeed(), p2d.GetYSpeed(), p2d.GetYawSpeed(), p2d.GetStall(), p2d.GetLost(), p2d.GetStatus() ) );
                if ( power.IsFresh()==True ):
                    power.NotFresh();
                    #print('time=%f Percent=%3.2f<%%> Voltage=%3.2f<V> watts=%3.2f<w> RunTime=%f<hour>' % ( power.GetDataTime(), power.GetPercent(), power.GetCharge(), power.GetWatts(), power.GetRunTime()/3600.0) );     

            #v <m/s>
            #w <rad/s>
            #vacc <m/s**2>
            #wacc <rad/s**2>
            v=0.0
            w=1.0
            vacc=1.0
            wacc=1.0
            p2d.SetSpeed(v,w,vacc,wacc);



    except KeyboardInterrupt:
        print("program exit\n");


    v=0.0
    w=0.0
    vacc=1.0
    wacc=1.0
    p2d.SetSpeed(v,w,vacc,wacc);
    #dp.SetOutput(1,0); # LDS power disable
    #lp0.Enable(False);  # LDS stop 
    #lp1.Enable(False);  # LDS stop
    # TODO: we can/should proxies and robot explicity?
    del dis
    del power
    del p2d
    del dp
    del lp1
    del lp0
    del robot
