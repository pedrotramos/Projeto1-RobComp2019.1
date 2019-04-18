# Para RODAR
# python object_detection_webcam.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel
# Credits: https://www.pyimagesearch.com/2017/09/11/object-detection-with-deep-learning-and-opencv/

print("Para executar:\npython object_detection_webcam.py --prototxt  --model ")

# import the necessary packages
import numpy as np
import argparse
import cv2

import rospkg
import os
import Projeto1 as P1

contador_mnet = 1000

rospack = rospkg.RosPack()
path = rospack.get_path('p1_ros')
scripts = os.path.join(path,  "scripts")

proto = os.path.join(scripts,"MobileNetSSD_deploy.prototxt.txt")
model = os.path.join(scripts, "MobileNetSSD_deploy.caffemodel")
confianca = 0.2

initBB = None

# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

print('Objetos que podem ser detectados:')
print(CLASSES)
print('Que objeto deve ser identificado?')
detectado = raw_input()
detectado = str(detectado)
detectado = detectado.strip()
detectado = detectado.lower()

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(proto, model)

# load the input image and construct an input blob for the image
# by resizing to a fixed 300x300 pixels and then normalizing it
# (note: normalization is done via the authors of the MobileNet SSD
# implementation)


def detect(frame):
    global detectado
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence


        if confidence > confianca:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            if CLASSES[idx] == detectado:
            # display the prediction
                label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                print("[INFO] {}".format(label))
                cv2.rectangle(image, (startX, startY), (endX, endY),
                    COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))
        if results == []:
            P1.contador = 0
    # show the output image
    return image, results

def track(frame):
    global initBB
    global contador_mnet
    P1.contador = 10
    tracker_type = "kcf"
    # extract the OpenCV version info
    (major, minor) = cv2.__version__.split(".")[:2]

    # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
    # function to create our object tracker
    if int(major) == 3 and int(minor) < 3:
        tracker = cv2.Tracker_create(tracker_type.upper())

    # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
    # approrpiate object tracker constructor:
    else:
        # initialize a dictionary that maps strings to their corresponding
        # OpenCV object tracker implementations
        OPENCV_OBJECT_TRACKERS = {
            "csrt": cv2.TrackerCSRT_create,
            "kcf": cv2.TrackerKCF_create,
            "boosting": cv2.TrackerBoosting_create,
            "mil": cv2.TrackerMIL_create,
            "tld": cv2.TrackerTLD_create,
            "medianflow": cv2.TrackerMedianFlow_create,
            "mosse": cv2.TrackerMOSSE_create
        }

        # grab the appropriate object tracker using our dictionary of
        # OpenCV object tracker objects
        tracker = OPENCV_OBJECT_TRACKERS[tracker_type]()

    if P1.contador == 10:

        result_frame, result_tuples = detect(frame)
        # select initial bounding box
        if result_tuples != []:
            initBB = (result_tuples[0][2][0], result_tuples[0][2][1],
                    result_tuples[0][3][0] - result_tuples[0][2][0], 
                    result_tuples[0][3][1] - result_tuples[0][2][1])

        # start OpenCV object tracker using the supplied bounding box
            tracker.init(frame, initBB)

            P1.contador = 20

    # check to see if we are currently tracking an object
    if P1.contador == 20:
        # grab the new bounding box coordinates of the object
        (success, box) = tracker.update(frame)

        # check to see if the tracking was a success
        if success:

            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                (0, 255, 0), 2)
        else:
            P1.contador = 0

        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            ("Tracker", tracker_type),
            ("Success", "Yes" if success else "No"),
        ]
    contador_mnet = P1.contador




    



