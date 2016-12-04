#!/usr/bin/env python
# -*- coding: utf-8 -*-



import cv2


# Text in image:
def draw_str(dst, (x, y), s, sz=1.0, th=2):
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, sz, (0, 0, 0),
                thickness=th, lineType=cv2.CV_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, sz, (255, 255, 255),
                thickness=th-1, lineType=cv2.CV_AA)



def fixroi(roi, imshape):
    if roi == ((-1, -1), (-1, -1)):
        rroi = ((0, 0), (imshape[1], imshape[0]))
    else:
        rroi = ((max(0, min(roi[0][0], roi[1][0])),
                 max(0, min(roi[0][1], roi[1][1]))),
                (min(imshape[1], max(roi[0][0], roi[1][0])),
                 min(imshape[0], max(roi[0][1], roi[1][1]))))
    return rroi



def subimg(pimg, proi):
    return pimg[proi[0][1]:proi[1][1], proi[0][0]:proi[1][0]]


def setsubimg(img1, img2, proi):
    img1[proi[0][1]:proi[1][1], proi[0][0]:proi[1][0]] = img2
