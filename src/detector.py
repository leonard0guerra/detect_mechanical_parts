import cv2 as cv
import numpy as np
from utils import Utils

class Detector:

    #TODO: create config. file
    YOLO_CFG = 'yolo_model/yolo-obj.cfg'
    YOLO_WEIGHTS = 'yolo_model/yolo-obj_6100.weights'
    OBJ_NAMES = 'yolo_model/obj.names'

    CONF_THRESHOLD = 0.3
    NMS_THRESHOLD = 0.4


    def __init__(self):
        self.__init_colors_for_classes()
        self.__init_network()
    
    def __init_colors_for_classes(self):
        self.labels = []
        with open(Utils.absolutePath(Detector.OBJ_NAMES), "r") as f:
            self.labels = [cname.strip() for cname in f.readlines()]
        
        self.bbox_colors = np.random.uniform(low=0, high=255, size=(len(self.labels), 3))
        
    
    def __init_network(self):
        self.net = cv.dnn.readNetFromDarknet(Utils.absolutePath(Detector.YOLO_CFG), \
                                    darknetModel=Utils.absolutePath(Detector.YOLO_WEIGHTS))

        layer_names = self.net.getLayerNames()
        self.__output_layer = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, image):
        blob = cv.dnn.blobFromImage(image, scalefactor=1/255, size=(416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.__output_layer)
        return outputs
    
    def process_image(self, outputs, image):
        boxes, confidences, classIDs = [], [], []

        H, W = image.shape[:2]
                                    
        for out in outputs:
            for detection in out:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                                        
                if confidence > Detector.CONF_THRESHOLD:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)
                                        
        idxs = cv.dnn.NMSBoxes(boxes, confidences, score_threshold=Detector.CONF_THRESHOLD, nms_threshold=Detector.NMS_THRESHOLD)
                                        
        if len(idxs)>0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                                        
                clr = [int(c) for c in self.bbox_colors[classIDs[i]]]
                                        
                cv.rectangle(image, (x, y), (x+w, y+h), clr, 2)
                cv.putText(image, "{}: {:.4f}".format(self.labels[classIDs[i]], confidences[i]), (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, clr, 2)
