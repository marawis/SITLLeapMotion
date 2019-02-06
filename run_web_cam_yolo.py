import cv2
from darkflow.net.build import TFNet
import numpy as np
import time


class VideoCamera(object):
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        self.video = cv2.VideoCapture(0)
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        # self.video = cv2.VideoCapture('video.mp4')
        self.options = {
            # 'model': 'cfg/v1/yolo-tiny.cfg',
            # 'load': 'bin/yolo-tiny.weights',
            # 'model' : 'cfg/yolov3.cfg',
            # 'load' : 'bin/yolov3.weights',
            'model': 'cfg/yolov2-tiny.cfg',
            'load': 'bin/yolov2-tiny.weights',
            'threshold': 0.4,
            'gpu': 1.0
        }

        print("TFNET Setup")
        self.tfnet = TFNet(self.options)
        self.colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]

        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def __del__(self):
        self.video.release()

    def get_frame(self):
        # success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        # while(success):
        #    ret, jpeg = cv2.imencode('.jpg', image)
        #   return jpeg.tobytes()

        stime = time.time()
        ret, frame = self.video.read()
        if ret:
            results = self.tfnet.return_predict(frame)
            for color, result in zip(self.colors, results):
                tl = (result['topleft']['x'], result['topleft']['y'])
                br = (result['bottomright']['x'], result['bottomright']['y'])
                label = result['label']
                confidence = result['confidence']
                text = '{}: {:.0f}%'.format(label, confidence * 100)
                frame = cv2.rectangle(frame, tl, br, color, 5)
                frame = cv2.putText(frame, text, tl, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)

            cv2.imshow('frame', frame)
            ret, jpeg = cv2.imencode('.jpg', frame)
            return jpeg.tobytes()
        # print('FPS {:.1f}'.format(1 / (time.time() - stime)))
# if cv2.waitKey(1) & 0xFF == ord('q'):
#    break
