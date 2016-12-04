# -*- coding: utf-8 -*-
"""

@Original author: José María Sola Durán, 2015

@Modified by Songyou Peng, Kaisar Kushibar and Paola Ardon, 2016
"""

import cv2
import os
import numpy as np
import utilscv

# A python class has been created called ImageFeature
# This class encloses the necessary information for each
# of the images in the DB to be recognized

class ImageFeature(object):
    def __init__(self, nameFile, shape, imageBinary, kp, desc):
        #Name of the file
        self.nameFile = nameFile
        #Shape of the image
        self.shape = shape
        #Binary data of the image
        self.imageBinary = imageBinary
        #Image keypoints after features detection algorithm has been applied
        self.kp = kp
        #Detected features of the descriptores
        self.desc = desc
        #Matching the image in the DB with the input image in the webcam
        self.matchingWebcam = []
        #Matching the webcam with the actual image in the DB
        self.matchingDatabase = []
    #It allows to empty the previously calculated matches to start storing a new image  
    def clearMatchingMutuos(self):
        self.matchingWebcam = []
        self.matchingDatabase = []
# Function to calculate for each one of the methods the features
# these features are the ones for the images in the folder "modelos"
def loadModelsFromDirectory():
    # Each method returns a dictionary. The key is the features algorithm
    # the value is a list of elements of type ImageFeature where all the data
    # of the images is stored.

    dataBase = dict([('SIFT', []), ('SURF', []), 
                     ('ORB', [])])
    #The number of features is limited to 250
    sift = cv2.SIFT(nfeatures=250)
    #akaze = cv2.AKAZE()
    surf = cv2.SURF(800)
    orb = cv2.ORB(400)
    #brisk = cv2.BRISK()
    for imageFile in os.listdir("/home/cnx/catkin_ws/src/pepper_recog/src/models"):
        #Load the image with OpenCV
        colorImage = cv2.imread("/home/cnx/catkin_ws/src/pepper_recog/src/models/" + str(imageFile))
        if colorImage is None:
            print "wrong input"
        #Convert the image to grayscale
        currentImage = cv2.cvtColor(colorImage, cv2.COLOR_BGR2GRAY)
        #Resizing the image so that both images are the same
        kp, desc = sift.detectAndCompute(currentImage, None)
        #Load the features with SIFT
        dataBase["SIFT"].append(ImageFeature(imageFile, currentImage.shape, colorImage, kp, desc))
        #Se cargan las features con AKAZE
        #kp, desc = akaze.detectAndCompute(currentImage, None)
        #dataBase["AKAZE"].append(ImageFeature(imageFile, currentImage.shape, colorImage, kp, desc))
        #Se cargan las features con SURF
        kp, desc = surf.detectAndCompute(currentImage, None)
        dataBase["SURF"].append(ImageFeature(imageFile, currentImage.shape, colorImage, kp, desc))
        #Loading features with ORB
        kp, desc = orb.detectAndCompute(currentImage, None)
        dataBase["ORB"].append(ImageFeature(imageFile, currentImage.shape, colorImage, kp, desc))
         #Se cargan las features con BRISK
        #kp, desc = brisk.detectAndCompute(currentImage, None)
        #dataBase["BRISK"].append(ImageFeature(imageFile, currentImage.shape, colorImage, kp, desc))
    return dataBase
#Function in charge of calculating the mutual matchings with nesting for loops
#Slow solution because we are not taking advantage of Numpy. Not even a slider
#is used in the method due to the computational cost  

def findMatchingMutuos(selectedDataBase, desc, kp):
    for imgFeatures in selectedDataBase:
        imgFeatures.clearMatchingMutuos()
        for i in range(len(desc)):
            primerMatching = None
            canditatoDataBase = None
            matchingSegundo = None
            candidateWebCam = None
            for j in range(len(imgFeatures.desc)):
                valorMatching = np.linalg.norm(desc[i] - imgFeatures.desc[j])
                if (primerMatching is None or valorMatching < primerMatching):
                    primerMatching = valorMatching
                    canditatoDataBase = j
            for k in range(len(desc)):
                valorMatching = np.linalg.norm(imgFeatures.desc[canditatoDataBase] - desc[k])
                if (matchingSegundo is None or valorMatching < matchingSegundo):
                    matchingSegundo = valorMatching
                    candidateWebCam = k
            if not candidateWebCam is None and i == candidateWebCam:
                imgFeatures.matchingWebcam.append(kp[i].pt)
                imgFeatures.matchingDatabase.append(imgFeatures.kp[canditatoDataBase].pt)
    return selectedDataBase

#Function in charge of calculatinf the mutual matchings in the webcam image
#with the images in the DB. It gets as input the obtained list from the
#feature detection algorithm
def findMatchingMutuosOptimizado(selectedDataBase, desc, kp):
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 7)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    #The algorithm repeats for every image in the DB
    for img in selectedDataBase:
        img.clearMatchingMutuos()

        # Brute force matching

        # for i in range(len(desc)):
        #      #We calculate the norm of the differences of the current descriptor with all
        #      #the image descriptors in the DB. We find them without loops making use of the
        #      #broadcasting of the Numpy, all the distances between the current descriptor with
        #      #all the descriptors in the image
        #      distanceListFromWebCam = np.linalg.norm(desc[i] - img.desc, axis=-1)
        #      #We obtain the candidate with minimum distance from the current descriptor
        #      candidatoDataBase = distanceListFromWebCam.argmin() 
        #      #Double check the mutual matching. Check if the candidatoDatabase
        #      #has the current descriptor as the best matching
        #      distanceListFromDataBase = np.linalg.norm(img.desc[candidatoDataBase] - desc,
        #                                    axis=-1)
        #      candidatoWebCam = distanceListFromDataBase.argmin()
        #      #If the mutual matching is accomplisedh, we store it to deal with it later
        #      if (i == candidatoWebCam):
        #         img.matchingWebcam.append(kp[i].pt)
        #         img.matchingDatabase.append(img.kp[candidatoDataBase].pt)
        # #For convinience we convert into Numpy ND-Array  
        # img.matchingWebcam = np.array(img.matchingWebcam)
        # img.matchingDatabase = np.array(img.matchingDatabase)


        # FLANN matching

        matches = flann.knnMatch(img.desc,desc,k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        matchingWebcam = []
        matchingDatabase = []
        m = []
        if len(good)>30:
            for m in good:

                matchingWebcam.append(kp[m.trainIdx].pt)
                matchingDatabase.append(img.kp[m.queryIdx].pt)

            matchingWebcam = np.array(matchingWebcam)
            matchingDatabase = np.array(matchingDatabase)

            img.matchingWebcam = matchingWebcam
            img.matchingDatabase = matchingDatabase
        else:
            img.matchingWebcam = np.array([])
            img.matchingDatabase = np.array([])
    return selectedDataBase

# This function calculates the best image in terms of inliers
# of each image in the DB with the image in the webcam
def calculateBestImageByNumInliers(selectedDataBase, projer, minInliers):
    if minInliers < 15:
        minInliers = 15
    bestIndex = None
    bestMask = None
    numInliers = 0
    #For each of the images
    for index, imgWithMatching in enumerate(selectedDataBase):
        #we compute the algorithm RANSAC to calculate the number of inliers       
        # print type(imgWithMatching.matchingDatabase)
        if imgWithMatching.matchingDatabase.size is not 0:
            _, mask = cv2.findHomography(imgWithMatching.matchingDatabase, 
                                     imgWithMatching.matchingWebcam, cv2.RANSAC, projer)
            if not mask is None:
                #Double check, from the mask the number of inliers.
                #If the number of inliers is greater than the minimum of of inliers,
                #and is the max (more inliers than the previous image)
                #then we consider is the matching image to the one in the DB
                countNonZero = np.count_nonzero(mask)
                if (countNonZero >= minInliers and countNonZero > numInliers):
                    numInliers = countNonZero
                    bestIndex = index
                    bestMask = (mask >= 1).reshape(-1)
    #If there is an image that has been obtained as the best image, therefore
    #we calculate the keypoints that are the inliers from the obtained mask in
    # findHomography and it is returned as the best image 
    if not bestIndex is None:
        bestImage = selectedDataBase[bestIndex]
        inliersWebCam = bestImage.matchingWebcam[bestMask]
        inliersDataBase = bestImage.matchingDatabase[bestMask]
        return bestImage, inliersWebCam, inliersDataBase
    return None, None, None
                
#This function calculates the affinity matrix and draws the rectangle
#arounf the detected object
def calculateAffinityMatrixAndDraw(bestImage, inliersDataBase, inliersWebCam, imgout):
    #Calculating affinity matrix
    depth = 0

    A, mask = cv2.findHomography(inliersDataBase, inliersWebCam, cv2.RANSAC,5.0)
    if not np.all(A == 0):
        #Calculating the points for the rectangle in the recognized object
        a = np.array([0, 0, 1], np.float)
        b = np.array([bestImage.shape[1], 0, 1], np.float)
        c = np.array([bestImage.shape[1], bestImage.shape[0], 1], np.float)
        d = np.array([0, bestImage.shape[0], 1], np.float)
        centro = np.array([float(bestImage.shape[0])/2, 
           float(bestImage.shape[1])/2, 1], np.float)
           
        #multiply the points in the virtual space to convert them into real points
        #in the image
        a = np.dot(A, a)
        b = np.dot(A, b)
        c = np.dot(A, c)
        d = np.dot(A, d)
        centro = np.dot(A, centro)
        
        areal = (int(a[0]/a[2]), int(a[1]/b[2]))
        breal = (int(b[0]/b[2]), int(b[1]/b[2]))
        creal = (int(c[0]/c[2]), int(c[1]/c[2]))
        dreal = (int(d[0]/d[2]), int(d[1]/d[2]))
        centroreal = (int(centro[0]/centro[2]), int(centro[1]/centro[2]))
        
        #Draw the polygono and the name of the file in the image
        points = np.array([areal, breal, creal, dreal], np.int32) 
        # print(np.sqrt(np.sum(np.square(points[1,:] - points[2,:]))))
        depth = 0.250*bestImage.shape[1]/np.sqrt(np.sum(np.square(points[0,:] - points[1,:]))) - 0.25
        cv2.polylines(imgout, np.int32([points]),1, (0,255,0), thickness=2)
        utilscv.draw_str(imgout, centroreal, bestImage.nameFile.upper().split('.')[0], sz=2.0, th=3)

    return depth, centroreal
