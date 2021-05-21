#!/usr/bin/python
# ce script detecte le mouvement au centre de la cam
import sys
import time
from math import cos, sin
import cv2.cv as cv
import RPi.GPIO as gpio
import time
import os

gpio.setmode(gpio.BCM) # on utilise le mode BMC
gpio.setup(20, gpio.OUT) # on definit le port 20 comme etant une sortie

CLOCKS_PER_SEC = 1.0
MHI_DURATION = 1
MAX_TIME_DELTA = 0.5
MIN_TIME_DELTA = 0.05
N = 4
buf = range(10)
last = 0
mhi = None # Motion History image
orient = None
mask = None
segmask = None
storage = None
no_mov = 0
temps = 0

def update_mhi(img, dst, diff_threshold):
    global last #on les declare comme globales afin de pouvoir les modifier
    global mhi
    global storage
    global mask
    global orient
    global segmask
    global no_mov
    timestamp = time.clock() / CLOCKS_PER_SEC
    size = cv.GetSize(img)
    idx1 = last
    if not mhi or cv.GetSize(mhi) != size:
        for i in range(N):
            buf[i] = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
            cv.Zero(buf[i])
        mhi = cv.CreateImage(size,cv. IPL_DEPTH_32F, 1)
        cv.Zero(mhi)
        orient = cv.CreateImage(size,cv. IPL_DEPTH_32F, 1)
        segmask = cv.CreateImage(size,cv. IPL_DEPTH_32F, 1)
        mask = cv.CreateImage(size,cv. IPL_DEPTH_8U, 1)

    cv.CvtColor(img, buf[last], cv.CV_BGR2GRAY)
    idx2 = (last + 1) % N
    last = idx2
    silh = buf[idx2]
    cv.AbsDiff(buf[idx1], buf[idx2], silh)
    cv.Threshold(silh, silh, diff_threshold, 1, cv.CV_THRESH_BINARY)
    cv.UpdateMotionHistory(silh, mhi, timestamp, MHI_DURATION)
    cv.CvtScale(mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION)
    cv.Zero(dst)
    cv.Merge(mask, None, None, None, dst)
    cv.CalcMotionGradient(mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3)
    if not storage:
        storage = cv.CreateMemStorage(0)
    seq = cv.SegmentMotion(mhi, segmask, storage, timestamp, MAX_TIME_DELTA)
    for (area, value, comp_rect) in seq:
        if comp_rect[2] + comp_rect[3] > 500: # seuil de rejet petits mouvements
            color = cv.CV_RGB(255, 0,0)
            silh_roi = cv.GetSubRect(silh, comp_rect)
            mhi_roi = cv.GetSubRect(mhi, comp_rect)
            orient_roi = cv.GetSubRect(orient, comp_rect)
            mask_roi = cv.GetSubRect(mask, comp_rect)
            angle = 360 - cv.CalcGlobalOrientation(orient_roi, mask_roi, mhi_roi, timestamp, MHI_DURATION)

            count = cv.Norm(silh_roi, None, cv.CV_L1, None)
            if count < (comp_rect[2] * comp_rect[3] * 0.05):
                continue

            magnitude = 30.
            center = ((comp_rect[0] + comp_rect[2] / 2), (comp_rect[1] + comp_rect[3] / 2))
            cv.Circle(dst, center, cv.Round(magnitude*1.2), color, 3, cv.CV_AA, 0)
            cv.Line(dst,
                    center,
                    (cv.Round(center[0] + magnitude * cos(angle * cv.CV_PI / 180)),
                     cv.Round(center[1] - magnitude * sin(angle * cv.CV_PI / 180))),
                    color,
                    3,
                    cv.CV_AA,
                    0)
            if center[0] > 200 and center[0] < 500 and center[1] > 80 and center[1] < 400:
                color2 = cv.CV_RGB(255, 255,0)
                cv.Circle(dst, (200, 80), cv.Round(magnitude*0.3), color2, 3, cv.CV_AA, 0)
                cv.Circle(dst, (200, 400), cv.Round(magnitude*0.3), color2, 3, cv.CV_AA, 0)
                cv.Circle(dst, (500, 80), cv.Round(magnitude*0.3), color2, 3, cv.CV_AA, 0)
                cv.Circle(dst, (500, 400), cv.Round(magnitude*0.3), color2, 3, cv.CV_AA, 0)
                if no_mov > 3: #seuil temps vehicule doit rester arreter avant beep
                    print ("*** BEEP ***")
                    gpio.output(20, gpio.HIGH)
                    time.sleep(0.5)
                    gpio.output(20, gpio.LOW)
                no_mov = 0 #des qu'il y a un mouvement: on initalise no_mov

if __name__ == "__main__":
    motion = 0
    capture = 0

    if len(sys.argv)==1:
        capture = cv.CreateCameraCapture(0) # flux de la webcam (utiliser VideoCapture(1) pour faire plusieurs flux)
    elif len(sys.argv)==2 and sys.argv[1].isdigit():
        capture = cv.CreateCameraCapture(int(sys.argv[1]))
    elif len(sys.argv)==2:
        capture = cv.CreateFileCapture(sys.argv[1])

    if not capture:
        print "Initialisation capture impossible..."
        sys.exit(-1)

    cv.NamedWindow("Motion", 1)
    while True:
        temps = temps + 1
        if temps > 10: # tout les 10 on initalise temps et on fait no_mov++
            no_mov = no_mov + 1
            print ("pas de mouvement depuis: " + str(no_mov))
            temps = 0
#        if no_mov > 400:
#            no_mov = 0
#            os.system('sudo shutdown -h now') #on eteint car aucun mov durant 10 min donc surement garage
        image = cv.QueryFrame(capture)
        if(image):
            if(not motion):
                    motion = cv.CreateImage((image.width, image.height), 8, 3)
                    cv.Zero(motion)
            update_mhi(image, motion, 30)
            cv.ShowImage("Motion", motion)
            if(cv.WaitKey(10) != -1):
                break
        else:
            break
    cv.DestroyWindow("Motion")
