# -*- coding: utf-8 -*-
"""
Created on Sun Sep 22 11:50:20 2019

@author: abhis
"""
import cv2
import numpy as np
import math
from scipy.spatial import distance
import ctypes
import time

SendInput = ctypes.windll.user32.SendInput
def releaseall():
        ReleaseKey(A)
        ReleaseKey(D)
        ReleaseKey(X)

PUL = ctypes.POINTER(ctypes.c_ulong)
class KeyBdInput(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort),
                ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class HardwareInput(ctypes.Structure):
    _fields_ = [("uMsg", ctypes.c_ulong),
                ("wParamL", ctypes.c_short),
                ("wParamH", ctypes.c_ushort)]

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time",ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("ki", KeyBdInput),
                 ("mi", MouseInput),
                 ("hi", HardwareInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong),
                ("ii", Input_I)]

def PressKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def ReleaseKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008 | 0x0002, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))


def coordinates(contour,frame):
    c=contour.copy()
    x, y, w, h = cv2.boundingRect(c)
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    a=box[0]
    b=box[1]
    c=box[2]
    d=box[3]
    cv2.circle(frame,(a[0],a[1]),10,(0,255,0),-1)   
    cv2.drawContours(frame,[box],0,(0,255,255),-1)
    return box

def click_w(angle):
    for i in range(90-angle):
        PressKey(W)
        
def click_a(angle):
    for i in range(angle):
        PressKey(A)

def click_d(angle):
    for i in range(angle):
        PressKey(D)

def keybinding_angle(angle,DK,MK):
        releaseall()
        if(angle<=3):
            releaseall()
            PressKey(MK)
        elif((angle>3) & (angle<=30)):
            PressKey(DK)
            PressKey(MK)
            time.sleep(0.01)
            ReleaseKey(DK)
        elif((angle>30) & (angle<=45)):
            PressKey(DK)
            PressKey(MK)
            time.sleep(0.02)
            ReleaseKey(DK)
        elif((angle>45) & (angle<=65)):
            PressKey(DK)
            PressKey(MK)
            time.sleep(0.03)
            ReleaseKey(DK)
        elif((angle>65) & (angle<=90)):
            PressKey(DK)
            PressKey(MK)
            time.sleep(0.04)
            ReleaseKey(DK)
    

def drive(box,MK):
    a=box[0]
    b=box[1]
#    c=box[2]
    d=box[3]       
    if((distance.euclidean(a,b))>(distance.euclidean(a,d))):
        diff=b[0]-a[0]
        if(diff==0):
            angle=90
        else:
            angle=int(abs(math.degrees(math.atan((b[1]-a[1])/(b[0]-a[0])))))
        DK=D
        if((want_results==1) & ((count%30)==0)):
                global right_side_list
                right_side_list=np.append(right_side_list,angle)
        keybinding_angle(angle,DK,MK)
    
    elif((distance.euclidean(a,b))<(distance.euclidean(a,d))):
        diff=d[0]-a[0]
        if(diff==0):
            angle=90
        else:
            angle=int(abs(math.degrees(math.atan((d[1]-a[1])/(d[0]-a[0])))))
        DK=A

        if((want_results==1) & ((count%30)==0)):
                global left_side_list
                left_side_list=np.append(left_side_list,angle)
        keybinding_angle(angle,DK,MK)




#------------------------------------------------------------------------Hand calculation-----


def normalize_hand(list1):
    temp_list=list1.copy()
    
    for i in range(len(temp_list)):
        angle=temp_list[i]
        if(angle<=5):
            temp_list[i]=5
        elif((angle>5) & (angle<=30)):
            temp_list[i]=30
        elif((angle>30) & (angle<=45)):
            temp_list[i]=45
        elif((angle>45) & (angle<=60)):
            temp_list[i]=60
        elif((angle>60) & (angle<=75)):
            temp_list[i]=75
        elif((angle>75) & (angle<=90)):
            temp_list[i]=90
    return temp_list
            
def normalize_thumb(list1):
    temp_list=list1.copy()
    
    for i in range(len(temp_list)):
        num_of_conts=temp_list[i]
        if(num_of_conts<1):
            temp_list[i]=0
        else:
            temp_list[i]=1
    return temp_list


def check_all(list1):
    temp_list=list1.copy()
    final_list=np.array([])
    
    final_list=np.append(final_list,temp_list[0])
    
    for i in range(len(temp_list)-1):
        if(temp_list[i]!=temp_list[i+1]):
            final_list=np.append(final_list,temp_list[i+1])
    
#    print("final",len(temp_list),len(final_list))
#    print(final_list)
#    
    thumb_count=np.count_nonzero(final_list==1)
    c5=np.count_nonzero(final_list==5)
    c30=np.count_nonzero(final_list==30)
    c45=np.count_nonzero(final_list==45)
    c60=np.count_nonzero(final_list==60)
    c75=np.count_nonzero(final_list==75)
    c90=np.count_nonzero(final_list==90)
    return thumb_count,c5,c30,c45,c60,c75,c90


def everything_hand(left_thumb_list,right_thumb_list,left_side_list,right_side_list,frame,final_results=0):
    
    if(final_results==1):
        if((len(left_thumb_list)==0)):
            left_thumb_list=np.append(left_thumb_list,0)
            
        if((len(right_thumb_list)==0)):
            right_thumb_list=np.append(right_thumb_list,0)
        
        if((len(left_side_list)==0)):
            left_side_list=np.append(left_side_list,0)
        
        if((len(right_side_list)==0)):
            right_side_list=np.append(right_side_list,0)
        
    
        left_thumb_list=left_thumb_list.copy()
        right_thumb_list=right_thumb_list.copy()
        left_side_list=left_side_list.copy()
        right_side_list=right_side_list.copy()
        
        left_thumb_final=normalize_thumb(left_thumb_list)
        right_thumb_final=normalize_thumb(right_thumb_list)
        left_side_final=normalize_hand(left_side_list)
        right_side_final=normalize_hand(right_side_list)
        
        left_thumb,left_5,left_30,left_45,left_60,left_75,left_90=check_all(np.concatenate((left_side_final,left_thumb_final)))
        right_thumb,right_5,right_30,right_45,right_60,right_75,right_90=check_all(np.concatenate((right_side_final,right_thumb_final)))
        
        
        ui_x=1024
        ui_y=512
        
        
        
        header_text="Results"
        header_font_size=1.1
        a=0.1
        offset_x=int(len(header_text)*header_font_size*9)
        
        header_xy=(int(ui_x/2)-offset_x,int(a*1.3*ui_y/2))
        header_line_start=(0,int(a*ui_y))
        header_line_end=(ui_x,int(a*ui_y))
        header_line_color=(255,0,0)
        
        center_line_start=(int(ui_x/2),int(a*ui_y))
        center_line_end=(int(ui_x/2),ui_y)
        center_line_color=(255,0,0)
        
        font=cv2.FONT_HERSHEY_TRIPLEX
    #    ui_frame=np.zeros((ui_y,ui_x,3),np.uint8)
        ui_frame=frame
        
        print(ui_frame.shape)
        cv2.line(ui_frame,header_line_start,header_line_end,header_line_color,2,4)
        cv2.line(ui_frame,center_line_start,center_line_end,center_line_color,2,4)
        
        cv2.putText(ui_frame,header_text,header_xy,font,header_font_size,(255,255,255))
        
        details_lx,details_ly=int(0.1*ui_x),int(a*3*ui_y)
        detail_font=cv2.FONT_HERSHEY_COMPLEX_SMALL
        detail_font_size=1
        offset=detail_font_size*12
        
        def print_left(var_list,name_list):
            
            for idx,var in enumerate(var_list):
                cv2.putText(ui_frame,name_list[idx]+": "+str(var),(details_lx,details_ly+offset*idx*3),detail_font,detail_font_size,(255,255,255))
        
        def print_right(var_list,name_list):
            
            for idx,var in enumerate(var_list):
                cv2.putText(ui_frame,name_list[idx]+": "+str(var),(details_lx+center_line_start[0],details_ly+offset*idx*3),detail_font,detail_font_size,(255,255,255))
            
        
        left_list=[left_thumb,left_5,left_30,left_45,left_60,left_75,left_90]
        name_list=["Thumb Movements   ","Angle between  0 & 5 ", "Angle between  5 & 30", "Angle between 30 & 45", "Angle between 45 & 60", "Angle between 60 & 75", "Angle between 75 & 90"]
        right_list=[right_thumb,right_5,right_30,right_45,right_60,right_75,right_90]
        
        header_line_start2=(0,int(a*ui_y)*2)
        header_line_end2=(ui_x,int(a*ui_y)*2)
        cv2.line(ui_frame,header_line_start2,header_line_end2,header_line_color,2,4)
        cv2.putText(ui_frame,"Left Hand Movements",(int(0.1*ui_x),int(a*1.6*ui_y)),detail_font,detail_font_size,(255,255,255))
        cv2.putText(ui_frame,"Right Hand Movements",(int(0.1*ui_x)+center_line_start[0],int(a*1.6*ui_y)),detail_font,detail_font_size,(255,255,255))
        
        
        footer_line_start=(0,int(0.85*ui_y)*2)
        footer_line_end=(ui_x,int(0.85*ui_y)*2)
        cv2.line(ui_frame,footer_line_start,footer_line_end,header_line_color,2,4)
        cv2.putText(ui_frame,"Total Hand Movements:"+str(left_5+left_30+left_45+left_60+left_75+left_90),(int(0.1*ui_x),int(a*9*ui_y)),detail_font,detail_font_size,(255,255,255))
        cv2.putText(ui_frame,"Total Hand Movements:"+str(right_5+right_30+right_45+right_60+right_75+right_90),(int(0.1*ui_x)+center_line_start[0],int(a*9*ui_y)),detail_font,detail_font_size,(255,255,255))
        
        print_left(left_list,name_list)
        print_right(right_list,name_list)
    
        return frame
    
    
    
    
    else:

        if((len(left_thumb_list)==0)):
            left_thumb_list=np.append(left_thumb_list,0)
            
        if((len(right_thumb_list)==0)):
            right_thumb_list=np.append(right_thumb_list,0)
        
        if((len(left_side_list)==0)):
            left_side_list=np.append(left_side_list,0)
        
        if((len(right_side_list)==0)):
            right_side_list=np.append(right_side_list,0)
        
    
        left_thumb_list=left_thumb_list.copy()
        right_thumb_list=right_thumb_list.copy()
        left_side_list=left_side_list.copy()
        right_side_list=right_side_list.copy()
        
        left_thumb_final=normalize_thumb(left_thumb_list)
        right_thumb_final=normalize_thumb(right_thumb_list)
        left_side_final=normalize_hand(left_side_list)
        right_side_final=normalize_hand(right_side_list)
        
        left_thumb,left_5,left_30,left_45,left_60,left_75,left_90=check_all(np.concatenate((left_side_final,left_thumb_final)))
        right_thumb,right_5,right_30,right_45,right_60,right_75,right_90=check_all(np.concatenate((right_side_final,right_thumb_final)))
        
        ui_x=480
        ui_y=640
        
        detail_font=cv2.FONT_HERSHEY_PLAIN
          
    
    
        a=0.1
        header_line_color=(255,0,0)
        center_line_start=(int(ui_x/2),int(a*ui_y))
        
    
        ui_frame=frame
    
        details_lx,details_ly=int(0.1*ui_x),int(a*3*ui_y)
        
        detail_font_size=1
        offset=detail_font_size*12
        
        def print_left(var_list,name_list):
            
            for idx,var in enumerate(var_list):
                cv2.putText(ui_frame,name_list[idx]+": "+str(var),(details_lx,details_ly+offset*idx*3),detail_font,detail_font_size,(0,0,255))
        
        def print_right(var_list,name_list):
            
            for idx,var in enumerate(var_list):
                cv2.putText(ui_frame,name_list[idx]+": "+str(var),(details_lx+center_line_start[0],details_ly+offset*idx*3),detail_font,detail_font_size,(0,0,255))
            
        
        left_list=[left_thumb,left_5,left_30,left_45,left_60,left_75,left_90]
        name_list=["Thumb Movements   ","Angle between  0 & 5 ", "Angle between  5 & 30", "Angle between 30 & 45", "Angle between 45 & 60", "Angle between 60 & 75", "Angle between 75 & 90"]
        right_list=[right_thumb,right_5,right_30,right_45,right_60,right_75,right_90]
          
        cv2.putText(ui_frame,"Left Hand Movements",(int(0.1*ui_x),int(a*1.6*ui_y)),detail_font,detail_font_size,(0,0,255))
        cv2.putText(ui_frame,"Right Hand Movements",(int(0.1*ui_x)+center_line_start[0],int(a*1.6*ui_y)),detail_font,detail_font_size,(0,0,255))
        
        
        footer_line_start=(0,int(0.85*ui_y)*2)
        footer_line_end=(ui_x,int(0.85*ui_y)*2)
        cv2.line(ui_frame,footer_line_start,footer_line_end,header_line_color,2,4)
        cv2.putText(ui_frame,"Total Hand Movements:"+str(left_5+left_30+left_45+left_60+left_75+left_90),(int(0.1*ui_x),int(a*9*ui_y)),detail_font,detail_font_size,(0,0,255))
        cv2.putText(ui_frame,"Total Hand Movements:"+str(right_5+right_30+right_45+right_60+right_75+right_90),(int(0.1*ui_x)+center_line_start[0],int(a*9*ui_y)),detail_font,detail_font_size,(0,0,255))
        
        print_left(left_list,name_list)
        print_right(right_list,name_list) 
        
        
    
        return frame



#-----------------------------------------------Execution--------------------------------------------


W=0x11
A=0x1E
S=0x1F
D=0x20
X=0x2D
space=0x39



webcam = cv2.VideoCapture(0)
play=1
count=0
want_results=1
left_thumb_list=np.array([])
left_side_list=np.array([])
right_thumb_list=np.array([])
right_side_list=np.array([])



while True:
    
    _, frame = webcam.read()
    count+=1
    
    if(count%1==0):
    
        frame=cv2.flip(frame,1)
        frame_og=frame.copy()
        filter_size=19
        frame_blurred=cv2.blur(frame,(filter_size,filter_size))
        
        hsv_img = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)
        blurred=hsv_img
    
        lower_green = np.array([25,70,70])
        upper_green = np.array([160,200,200])
        mask_green = cv2.inRange(blurred, lower_green, upper_green)
       
        lower_yellow=np.array([10,100,100])
        upper_yellow=np.array([40,255,255])
        mask_yellow=cv2.inRange(hsv_img,lower_yellow,upper_yellow)
    
        lower_blue = np.array([25,70,70])
        upper_blue = np.array([160,255,255])
        mask_blue1 = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        mask_blue=cv2.bitwise_xor(mask_green,mask_blue1)
        
        mask_blue=cv2.erode(mask_blue,None,iterations=1)
        mask_green=cv2.erode(mask_green,None,iterations=1)
        mask_yellow=cv2.erode(mask_yellow,None,iterations=1)
        
        mask=mask_blue+mask_yellow+mask_green
        result = cv2.bitwise_and(frame, frame, mask=mask)
           
        contours_green,_ =  cv2.findContours(mask_green,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue,_ =  cv2.findContours(mask_blue,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow,_ =  cv2.findContours(mask_yellow,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        
        if((want_results==1) & ((count%30)==0)):
            left_thumb_list=np.append(left_thumb_list,len(contours_yellow))
            right_thumb_list=np.append(right_thumb_list,len(contours_blue))

#        print(left_thumb_list,right_thumb_list)
        if((len(contours_blue)==0) & (len(contours_green)==0) & (len(contours_yellow)==0)):
                releaseall()
                ReleaseKey(W)
                ReleaseKey(S)
                
    
        for bc in contours_blue:
            if(cv2.contourArea(bc)>10):
                box=coordinates(bc,frame)
                if(play==1):
                    ReleaseKey(W)
                    drive(box,S)
        
        for gc in contours_green:
            if(len(contours_blue)==0):
                if(cv2.contourArea(gc)>1000):
                    box=coordinates(gc,frame)
                    if(play==1):
                        ReleaseKey(S)
                        drive(box,W)        
                
        for yc in contours_yellow:
            if(cv2.contourArea(yc)>100):
                box=coordinates(yc,frame)
                if(play==1):
                    ReleaseKey(S)
                    PressKey(X)
           
        if((want_results==1)):
            frame=everything_hand(left_thumb_list,right_thumb_list,left_side_list,right_side_list,frame,0)
                    
        cv2.imshow('Original', frame) 
        if (cv2.waitKey(1) == 13): 
            ReleaseKey(W)
            break


webcam.release()
cv2.destroyAllWindows()
results_frame=everything_hand(left_thumb_list,right_thumb_list,left_side_list,right_side_list,np.zeros((512,1024,3),np.uint8),1)
cv2.imshow("Results",results_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()