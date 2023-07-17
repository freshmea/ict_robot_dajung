from zumi.zumi import Zumi
from zumi.util.camera import Camera
from zumi.util.screen import Screen

import time
import cv2
import os
import socket
import numpy as np

zumi = Zumi()

landmarks = {
    0: 'up', 1: 'left', 2: 'right', 3: 'nyc', 4: 'china', 5: 'eiffel',
    6: 'bigben', 7: 'seattle'
    # , 8:'khalifa', 9:'chicago',
    # 10:'intersection', 11:'start'
}


def load_model(demo_name):
    import json
    from keras.models import model_from_json

    with open(os.path.dirname(os.path.abspath(__file__)) + demo_name + '_model.json') as f:
        json_string = json.load(f)

    model = model_from_json(json_string)
    model.load_weights(os.path.dirname(os.path.abspath(__file__)) + demo_name + '_weights.h5')

    print("model is loaded")
    return model


def take_a_bunch_of_pictures(camera, label):
    cnt = 5
    while cnt >= 0:
        print(cnt)
        cnt = cnt - 1
        image = camera.run()

        if label not in landmarks.values():
            print("~invalid label~")
        else:
            file_name = os.path.dirname(os.path.abspath(__file__)) + "/images/" + socket.gethostname() + "." + str(
                time.time()) + "." + label + ".jpg"
            cv2.imwrite(file_name, image)


def get_readable_predictions(predictions):
    stringy = ""
    for index in landmarks:
        stringy = stringy + landmarks.get(index) + " {:.2%}".format(predictions[0][index]) + "  "
    return stringy


def drive_and_stop(direction):
    drive(direction)
    zumi.stop()


def drive_and_continue(direction):
    drive(direction)
    zumi.forward(-1)


def drive(direction):
    if direction == "up":
        zumi.forward(.6)
    elif direction == 'left':
        zumi.turn_left(3)
    elif direction == "right":
        zumi.turn_right(3)
    elif direction != None:
        zumi.forward(.6)


def drive_to_landmark(landmark, model):
    screen = Screen()
    screen.hello()
    camera = Camera(image_w=64, image_h=64, image_d=3, framerate=10)

    try:
        while True:
            frame = camera.run()

            # ask NN to predict control from image
            pred = model.predict(frame[None, :, :, :])

            # get values from pred array
            iArrowDir = np.argmax(pred[0])
            command = landmarks.get(iArrowDir)
            print(get_readable_predictions(pred))
            print(command)

            if iArrowDir > 2 and command != landmark:
                second = 0
                second_idx = 0
                for i in range(8):
                    if i != iArrowDir and pred[0][i] > second:
                        second = pred[0][i]
                        second_idx = i
                iArrowDir = second_idx
                command = landmarks.get(iArrowDir)
                print("second : ", command)

            confidence = pred[0][iArrowDir]
            drive_and_continue(command)

            if command == landmark and confidence > .6:
                zumi.stop()
                screen.draw_text(command + ": " + "{:.1%}".format(confidence))
                return 0
            #                     confidence = pred[0][iArrowDir]
            # #                         print("found " + command + " with confidence: " + str(confidence))
            #                     if command == "china" and confidence < .82:
            #                         x=1
            # #                             print("FAKE CHINA ")
            #                     elif command == "nyc" and confidence < .9:
            # #                             print("FAKE NEW YORK ")  ?
            #                         x=1
            #                     else:
            #                         engine.stop()
            #                         time.sleep(5)
            elif landmark != "a" and landmark == "all" and command not in ["left", "right", "up"]:
                screen.draw_text(command + ": " + "{:.1%}".format(confidence))
                zumi.stop()
                time.sleep(5)
                zumi.hello()
    finally:
        zumi.stop()
        camera.shutdown()