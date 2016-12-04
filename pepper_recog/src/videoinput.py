#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Simple generic class of reading a sequence of input images.
It can read from a single image/video in a folder or from a sequence of images/video in the folder.
We can specify the the parameters in the initialization and the rescaling factor (rows x cols).
Also, we set the images to be read in a cycle or to finish after the last
element, in which case if would return None.

Note 1: If the initialization parameters are empty then the input image is not scaled and
the normal size of the image is the default size of the input image.

Possible examples of valid initialization values:

Video reading without scaling and repeating at the end (loop):
"/path/to/file.mp4:loop"

Equivalent to the previous one:
"/path/to/file.mp4:rows=0:cols=0:loop"

Similar, but with another type of file:
"/path/to/file.avi"

Reading sequence of images scaling only the widht and looping at the end:
"/path/to/img-???.jpg:rows=0:cols=200:loop"

Reading only one scaled image. If it tries to read the same image for the second time
then it returns None (since we dont have loop):
"/path/to/img-001.jpg:rows=300:cols=400"

Reading  a sequence of images, each one with its orginal size and without looping
(At the end it will return None):
"/path/to/img-*.jpg"

Reading the numbered camera as 0:
"0"

Reading the numbered camera as 1 with re-scaling:
"1:rows=100:cols=200"

To check the usage of the example, check the program ejemplosimple.py:
Usage:
  ./ejemplosimple.py [<fuente de video>]
'''
import glob
import cv2


class VideoInput(object):
    def __init__(self, str):
        largs = map(lambda x: tuple(x.split('=')), str.split(':'))
        self.cols, self.rows = 0, 0
        self.loop = False
        self.type = 'camera'
        self.camera = 0
        for arg in largs:
            if len(arg) == 1:
                if arg[0] == 'loop':
                    self.loop = True
                elif arg[0].lower().endswith(('.jpg', '.jpeg',
                                              '.png', '.gif')):
                    self.type = 'imgfiles'
                    self.imgfiles = glob.glob(arg[0])
                    self.imgfiles.sort()
                    self.curframe = 0
                elif arg[0].lower().endswith(('.mpeg', '.mpg', '.dv', '.wmv',
                                              '.avi', '.mp4', '.webm', '.mkv')):
                    self.type = 'videofile'
                    self.videofile = arg[0]
                    self.cap = cv2.VideoCapture(self.videofile)
                elif arg[0].isdigit():
                    self.type = 'camera'
                    self.camera = int(arg[0])
                    self.cap = cv2.VideoCapture(self.camera)
            else:
                var, val = arg
                if(var == 'cols'):
                    self.cols = int(val)
                elif(var == 'rows'):
                    self.rows = int(val)
        if self.cols != 0 and self.type == 'camera':
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.cols)
        if self.rows != 0 and self.type == 'camera':
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.rows)

    def read(self):
        if self.type in ('camera', 'videofile'):
            flag, frame = self.cap.read()
            if self.loop and frame is None:
                self.cap.set(cv2.CAP_PROP_POS_AVI_RATIO, 0)
                flag, frame = self.cap.read()
        elif self.type in ('imgfiles', ):
            if self.curframe == len(self.imgfiles):
                if self.loop:
                    self.curframe = 0
                else:
                    return None
            frame = cv2.imread(self.imgfiles[self.curframe])
            self.curframe += 1
        if frame is not None:  # Posible escalado
            if self.cols != 0 and self.rows != 0:
                frame = cv2.resize(frame, (self.cols, self.rows))
            elif self.cols != 0:
                frame = cv2.resize(frame, (self.cols, frame.shape[0]))
            elif self.rows != 0:
                frame = cv2.resize(frame, (frame.shape[1], self.rows))
        return frame

    def close(self):
        if self.type in ('videofile', 'camera'):
            # try:
                self.cap.release()
