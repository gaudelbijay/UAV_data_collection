#!/usr/bin/env python

import cv2

class CameraPublisher:
    def __init__(self, rtsp_url):
        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            print("Failed to open the stream!")
            exit(-1)

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read a frame!")
                    break

                cv2.imshow("RTSP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        except Exception as e:
            print(e)
            
        finally:
            self.cap.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    rtsp_url = "rtsp://172.16.0.33:8900/live"
    camera_pub = CameraPublisher(rtsp_url)
