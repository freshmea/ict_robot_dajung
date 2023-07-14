from zumi.util.image_processor import Cropper

import numpy as np
import time
import cv2
import socket
import os


class HandDrawingClassifier():
    class Predictions():
        def __init__(self, outer_instance=None):
            self.outer_instance = outer_instance

        def set_value(self, predictions):
            self.predicts = predictions
            self.pred_index = np.argmax(self.predicts[0])
            self.pred_label = self.outer_instance.labels.get(self.pred_index)
            self.pred_confidence = self.predicts[0][self.pred_index]

        def get_readable_predictions(self):
            string = ""
            for index in self.outer_instance.labels:
                string = string + self.outer_instance.labels.get(index) + " {:.2%}".format(
                    self.predicts[0][index]) + "  "
            return string

    def __init__(self, demo_name, folder, dictionary, color=False, crop=False):
        self.demo_name = demo_name
        self.folder = folder
        self.model = None
        self.labels = {}
        self.commands = {}
        self.set_labels_and_commands(dictionary)
        self.predictions = self.Predictions(self)
        self.current_image = None
        self.color = color
        self.crop = crop

    def set_labels_and_commands(self, dictionary):
        i = 0
        for set in dictionary:
            self.labels[i] = set[0]
            self.commands[set[1]] = i
            i += 1

    def load_model(self):
        import json
        from keras.models import model_from_json
        from keras.models import load_model

        with open(self.folder + "/" + self.demo_name + "_model.json") as f:
            json_string = json.load(f)

        self.model = model_from_json(json_string)
        self.model.load_weights(self.folder + "/" + self.demo_name + "_weights.h5")

    def predict(self, image):
        self.current_image = image
        # if self.color :
        image = image[None, :, :, :]
        # else :
        #    print("gray")
        #    image = Image.fromarray(image)
        #    image = np.expand_dims(image, axis = 0)
        self.predictions.set_value(self.model.predict(image))

        print("found " + self.predictions.pred_label + " with confidence: " + str(self.predictions.pred_confidence))

        return self.predictions

    def save_image(self, image, data_type, command):
        file_name = self.folder + "/images/" + self.demo_name + "/" + data_type + "/" + command + "/" + socket.gethostname() + "." + str(
            time.time()) + "." + command + ".jpg"
        cv2.imwrite(file_name, image)

    def take_images(self, camera):
        command = ""
        cropper = Cropper()
        try:
            while True:
                data_type = input("Which data will you collect? (Type train / test) \n: ")
                if data_type == "train" or data_type == "test":
                    break

            print("Press keyboard commands and Enter for take images.\n")
            while True:
                command = input("")
                image = camera.run()

                if command in self.commands.keys():
                    command = self.labels[self.commands[command]]
                    if self.crop:
                        image = cropper.crop(image)
                        if image is None:
                            print("no image for crop")
                            continue
                    if not self.color:
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                    self.current_image = image
                    self.save_image(image, data_type, command)
                    print(command)
                else:
                    print("# bad command: " + command)
        finally:
            print("Error")
            camera.shutdown()
            exit()

    def create_folders(self):
        try:
            dic = "images/" + self.demo_name

            if not (os.path.isdir("images")):
                os.makedirs(os.path.join("images"))

            if not (os.path.isdir(dic)):
                os.makedirs(os.path.join(dic))
                os.makedirs(os.path.join(dic + "/train"))
                os.makedirs(os.path.join(dic + "/test"))
                for label in self.labels.keys():
                    os.makedirs(os.path.join(dic + "/train/" + self.labels[label]))
                    os.makedirs(os.path.join(dic + "/test/" + self.labels[label]))

        except OSError as e:
            if e.errno != errno.EEXIST:
                print("Failed to create directory!!!!!")
                raise
