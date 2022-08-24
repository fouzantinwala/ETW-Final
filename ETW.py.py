import cv2
import numpy as np
import dlib
from math import hypot
import time
import math

cap = cv2.VideoCapture(0)

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")


def midpoint(p1, p2):
    return int((p1.x + p2.x) / 2), int((p1.y + p2.y) / 2)


font = cv2.FONT_HERSHEY_PLAIN


def get_blinking_ratio(eye_points, facial_landmarks):
    left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))

    # hor_line = cv2.line(frame, left_point, right_point, (0, 255, 0), 2)
    # ver_line = cv2.line(frame, center_top, center_bottom, (0, 255, 0), 2)

    hor_line_length = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    ver_line_length = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))

    ratio = ver_line_length / hor_line_length
    return ratio


def get_gaze_ratio(eye_points, facial_landmarks):
    left_eye_region = np.array([(facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y),
                                (facial_landmarks.part(eye_points[1]).x, facial_landmarks.part(eye_points[1]).y),
                                (facial_landmarks.part(eye_points[2]).x, facial_landmarks.part(eye_points[2]).y),
                                (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y),
                                (facial_landmarks.part(eye_points[4]).x, facial_landmarks.part(eye_points[4]).y),
                                (facial_landmarks.part(eye_points[5]).x, facial_landmarks.part(eye_points[5]).y)],
                               np.int32)
    # cv2.polylines(frame, [left_eye_region], True, (0, 0, 255), 2)

    height, width, _ = frame.shape
    mask = np.zeros((height, width), np.uint8)
    cv2.polylines(mask, [left_eye_region], True, 255, 2)
    cv2.fillPoly(mask, [left_eye_region], 255)
    eye = cv2.bitwise_and(gray, gray, mask=mask)

    min_x = np.min(left_eye_region[:, 0])
    max_x = np.max(left_eye_region[:, 0])
    min_y = np.min(left_eye_region[:, 1])
    max_y = np.max(left_eye_region[:, 1])

    gray_eye = eye[min_y: max_y, min_x: max_x]
    _, threshold_eye = cv2.threshold(gray_eye, 50, 255, cv2.THRESH_BINARY)
    height, width = threshold_eye.shape
    left_side_threshold = threshold_eye[0: height, 0: int(0.55*width)]
    left_side_white = cv2.countNonZero(left_side_threshold)

    right_side_threshold = threshold_eye[0: height, int(0.45*width): width]
    right_side_white = cv2.countNonZero(right_side_threshold)

    if left_side_white == 0:
        gaze_ratio = 0.01
    elif right_side_white == 0:
        gaze_ratio = 100
    else:
        gaze_ratio = left_side_white / right_side_white
    return gaze_ratio

blink_time = -5;
decision = 0;
while True:
    start = time.time()
    _, frame = cap.read()
    new_frame = np.zeros((500, 500, 3), np.uint8)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    for face in faces:
        # x, y = face.left(), face.top()
        # x1, y1 = face.right(), face.bottom()
        # cv2.rectangle(frame, (x, y), (x1, y1), (0, 255, 0), 2)

        landmarks = predictor(gray, face)

        # Detect blinking
        left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)
        blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2

        # Gaze detection
        gaze_ratio_left_eye = get_gaze_ratio([36, 37, 38, 39, 40, 41], landmarks)
        gaze_ratio_right_eye = get_gaze_ratio([42, 43, 44, 45, 46, 47], landmarks)
        gaze_ratio = (gaze_ratio_right_eye + gaze_ratio_left_eye) / 2
        string = str(gaze_ratio)
        # cv2.putText(frame, string, (50, 100), font, 2, (0, 0, 255), 3)
        #print(gaze_ratio)
        count = 0;

        if blinking_ratio < 0.19:
            cv2.putText(frame, "BLINKING", (50, 150), font, 7, (255, 0, 0))
            if (time.time() - blink_time) < 0.75 and (time.time() - blink_time) > 0.25:
                count = 2;
            else:
                blink_time = time.time();
                count = 1;

        if count == 2:
            if decision != 0:
                count = 1;
                decision = 0;
                #cv2.putText(frame, "stop", (50, 100), font, 2, (0, 0, 255), 3)
            elif decision == 0:
                decision = 1;
                count = 1;
                #cv2.putText(frame, "start", (50, 100), font, 2, (0, 0, 255), 3)
        elif gaze_ratio < 0.6 and decision != 0:
            #cv2.putText(frame, "RIGHT", (50, 100), font, 2, (0, 0, 255), 3)
            decision = 4;
        elif 1.8 < gaze_ratio and decision != 0:
            #cv2.putText(frame, "LEFT", (50, 100), font, 2, (0, 0, 255), 3)
            decision = 2;
        elif gaze_ratio > 0.6 and 1.8 > gaze_ratio and decision != 0:
            #cv2.putText(frame, "CENTER", (50, 100), font, 2, (0, 0, 255), 3)
            decision = 3;

        if decision == 0:
            cv2.putText(frame, "Idle", (150, 100), font, 2, (0, 0, 255), 3)
        elif decision == 1:
            cv2.putText(frame, "Operating", (150, 100), font, 2, (0, 0, 255), 3)
        elif decision == 2:
            cv2.putText(frame, "Looking Left", (400, 100), font, 2, (0, 0, 255), 3)
        elif decision == 3:
            cv2.putText(frame, "Centre", (400, 100), font, 2, (0, 0, 255), 3)
        elif decision == 4:
            cv2.putText(frame, "Looking Right", (400, 100), font, 2, (0, 0, 255), 3)



    cv2.imshow("Frame", frame)

    #time_taken = time.time() - start
    #print(count)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()