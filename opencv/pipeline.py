import numpy as np
import cv2

camera = cv2.VideoCapture(0) # First webcam (video0)

while camera.isOpened():
    success, frame = camera.read()
    if not success:
        break

    # convert to hsv for identification by hue
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cube_min = np.array([90,200,40], np.uint8)
    cube_max = np.array([130,245,255], np.uint8)
    cube_mask = cv2.inRange(hsv, cube_min, cube_max)

    kernal = np.ones((25,25), "uint8")

    cube_mask = cv2.dilate(cube_mask, kernal)
    res_cube = cv2.bitwise_and(frame, frame, mask=cube_mask)
    #cv2.imshow("Pipeline output", res_cube)

    contours, hierarchy = cv2.findContours(cube_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate (contours):
        area = cv2.contourArea(contour)
        if (area > 28):
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,0),2)

            cv2.putText(frame, "Cube", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,0))

    cone_min = np.array([10,120,150], np.uint8)
    cone_max = np.array([40,255,255], np.uint8)
    cone_mask = cv2.inRange(hsv, cone_min, cone_max)

    cone_mask = cv2.dilate(cone_mask, kernal)
    res_cone = cv2.bitwise_and(frame, frame, mask=cone_mask)
    #cv2.imshow("Pipeline output", res_cone)


    contours, hierarchy = cv2.findContours(cone_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate (contours):
        area = cv2.contourArea(contour)
        if (area > 28):
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0),2)

            cv2.putText(frame, "Cone", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))

    cv2.imshow("Pipeline output", frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break

    # echo poweroff > ~/.bashrc
