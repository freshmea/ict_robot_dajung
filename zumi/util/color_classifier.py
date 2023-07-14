import numpy as np
import cv2
import os
import time
from random import uniform, shuffle
from zumi.util.knn_classifier import KNeighborsClassifier

absPATH = '/home/pi/Data'
DEMO = 'color-classifier'

class ColorClassifier():
    upper_hsv = [220, 255, 255]

    def __init__(self, demo_name="", is_balanced_data=True, path=absPATH, user_name=""):
        self.knn = KNeighborsClassifier()
        self.demo_name = demo_name
        self.label_names = []
        self.feature_names = ['h', 's', 'v']
        self.label_keys = []
        self.label_num = len(self.label_names)
        self.feature_num = len(self.feature_names)
        self.labels = []
        self.features = []
        self.data_cnt = {}
        self.divided_labels = []
        self.divided_features = []
        self.data_file_name = self.demo_name + "_KNN_data"
        self.current_feature_value = ''
        self.current_label = ''
        self.reactions = []
        self.predicts = None
        self.label_cnt = -1
        self.collect_num = 10
        self.pred_hsv_values = ""
        self.absPATH = path
        if user_name != "":
            self.absPATH = '/home/pi/Dashboard/user/'+user_name+'/Data'
        self.is_balanced_data = is_balanced_data
        self.__create_base_folders()

        if demo_name != "":
            self.load_model(demo_name)
   
    def fit(self, values='hsv'):
        features = self.__select_hsv_value(values, self.features)
        if features == None:
            hsv = input("You should input at least one of the following: 'h', 's', and 'v'. Try again.")
            return self.fit(hsv)
        
        self.pred_hsv_values = values
        self.knn.fit(features, self.labels)
            
    def predict(self, pred_features):
        check_hsv = False
        if not isinstance(pred_features, list):
            hsv_list = self.get_hsv_data(pred_features)
            pred_features = []
            if self.pred_hsv_values.find('h') != -1:
                pred_features.append(hsv_list[0])
            if self.pred_hsv_values.find('s') != -1:
                pred_features.append(hsv_list[1])
            if self.pred_hsv_values.find('v') != -1:
                pred_features.append(hsv_list[2])
        
        pred_list = list([pred_features])
        self.predicts = self.knn.predict(pred_list)

        return self.predicts

    def get_accuracy(self):
        zipped_lists = list(zip(self.features, self.labels))
        shuffle(zipped_lists)
        x, y = zip(*zipped_lists)
        x, y = list(x), list(y)

        cut = int(len(x) / 10)
        if cut == 0:
            print("There is not enough data. Collect more images.")
            return

        x_train = x[:-cut]
        y_train = y[:-cut]
        x_test = x[-cut:]
        y_test = y[-cut:]

        self.knn.fit(x_train, y_train)

        pred = self.knn.predict(x_test)

        print("Predict :", pred)
        print("Origin :", y_test)

        n = 0
        for i in range(len(pred)):
            if pred[i] == y_test[i]:
                n += 1

        accuracy = (n / len(y_test)) * 100
        print("Accuracy :" + str(accuracy))

        return accuracy

    def load_model(self, name):
        self.demo_name = name
        if os.path.isdir(os.path.join(self.absPATH, DEMO, self.demo_name)):
            print("Loaded ", name, " successfully")
            self.set_values_from_data()
        else:
            #print("Model ", name, " does not exist")
            raise Exception("This project does not exist.")


    def read_datas(self):
        while True:
            self.demo_name = input("Please enter the project name you want to run: ")

            if os.path.isdir(os.path.join(self.absPATH, DEMO, self.demo_name)):
                print("Okay, start ", self.demo_name)
                self.set_values_from_data()
                break
            else:
                print("This project does not exist.")

    def set_values(self):
        if self.demo_name == "":
            self.demo_name = input("What is the name of your project? : ")

        if os.path.isdir(os.path.join(self.absPATH, DEMO, self.demo_name)):
            if "y" == input("This project already exists. Do you want to test this model? (y/n) : "):
                self.set_values_from_data()
                return False
            else:
                self.demo_name = ""
                return self.set_values()
        else:
            self.data_file_name = self.demo_name + "_KNN_data"
            while True:
                try:
                    self.label_num = int(input("What is the total number of labels? : "))
                except ValueError:
                    print("Oops! Input was not an integer.")
                else:
                    if self.label_num <= 1:
                        print("You must have more than one label.")
                        continue
                    break

            for i in range(self.label_num):
                while True:
                    label = input("Label name (" + str(i + 1) + "/" + str(self.label_num) + ") : ")
                    while self.is_in_labels(label, False):
                        print("This label already exists. Please choose a different label name.")
                        label = input("Label name (" + str(i + 1) + "/" + str(self.label_num) + ") : ")

                    key = 'n'
                    while key == 'n':
                        key = input(
                            "Keyboard command for label (" + str(i + 1) + "/" + str(self.label_num) + ") : ")
                        if self.is_in_labels(key, False):
                            key = 'n'
                            print("This keyboard command already exists. Please choose a different command.")
                    if input("Label:" + label + " Command: " + key + ". Input 'Enter' to confirm or 'n' to rename. ") != "n":
                        self.label_names.append(label)
                        self.label_keys.append(key)
                        break

            return True

    def set_values_from_data(self):
        self.data_file_name = self.demo_name + "_KNN_data"
        file_path = os.path.join(self.absPATH, DEMO, self.demo_name, str(self.data_file_name + ".txt"))

        current_label = ""
        with open(file_path) as f:
            for line in f.readlines():
                line = line.split(' ')

                if line[0] == "Labels":
                    for cursor in line[2:-1]:
                        self.label_names.append(cursor)
                    self.label_num = len(self.label_names)

                elif line[0] == "Features":
                    for cursor in line[2:-1]:
                        self.feature_names.append(cursor)
                    self.feature_num = len(self.feature_names)

                elif line[0] in self.label_names:
                    current_label = line[0]

                elif line[0] == ">":
                    feature = []
                    for cursor in line[1:-1]:
                        feature.append(int(cursor))
                    self.labels.append(current_label)
                    self.features.append(feature)

    def check_enough_datas(self, balance="", min_num=30):
        if balance == "":
            balance = self.is_balanced_data

        self.is_balanced_data = balance

        prev_num = 0
        print("\nData collected: ")
        for label in self.data_cnt.keys():
            prev_num = self.data_cnt[label]
            print(label + ':' + str(self.data_cnt[label]))

        if len(self.data_cnt) < self.label_num:
            print("There is not enough data. Collect more images.")
            return False

        for num in self.data_cnt.values():
            if balance and num != prev_num:
                print("Collect the same amount of images for each label. Try again.")
                return False
            if num < min_num:
                print("There is not enough data. Collect more images.")
                return False

        return True

    def add_out_boundary_datas(self):
        for i in range(len(self.features)):
            feature = self.features[i]
            label = self.labels[i]
            if feature[0] < 40:
                temp = [180 + feature[0], feature[1], feature[2]]
                self.labels.append(label)
                self.features.append(temp)
            elif feature[0] > 180:
                temp = [0 + feature[0], feature[1], feature[2]]
                self.labels.append(label)
                self.features.append(temp)

    def add_data(self, label, feature):
        if not self.is_in_labels(label):
            return
        label = self.label_names[self.label_keys.index(label)]
        self.current_label = label
        self.label_cnt += 1

        if not isinstance(feature, list):
            feature = self.get_hsv_data(feature)

        self.labels.append(label)
        self.features.append(feature)
        if label in self.data_cnt.keys():
            self.data_cnt[label] += 1
        else:
            self.data_cnt[label] = 1

    def add_datas(self, camera, cnt=10):
        self.collect_num = cnt
        while True:
            label = input("Enter your key command for the label: ")

            if label == "q":
                break
            if self.is_in_labels(label):
                if label in self.label_names:
                    label = self.label_keys[self.label_names.index(label)]

                length = len(self.labels)
                for i in range(cnt):
                    image = camera.capture()
                    print("[", i, "]")
                    self.add_data(label, image)
                    time.sleep(0.1)
                if input("If you want to delete data, press 'y'. Else, press 'Enter': ") == 'y':
                    delete_list = []
                    while True:
                        try:
                            num = int(input("Input the data value you wish to delete or input -1 to cancel: "))

                            if num not in delete_list and num >= 0 and num < cnt:
                                delete_list.append(num)
                            if num == -1 or len(delete_list) == cnt:
                                break
                        except ValueError:
                            print("Oops! Input was not an integer.")

                    delete_list.sort(reverse=True)

                    for idx in delete_list:
                        self.data_cnt[self.labels[length + idx]] -= 1
                        del self.labels[length + idx]
                        del self.features[length + idx]
                    print(self.features)

        self.add_out_boundary_datas()

    def save_data_set(self):
        self.__create_folders()
        self.remove_outlier()

        f = open(os.path.join(self.absPATH, DEMO, self.demo_name, str(self.data_file_name + ".txt")), 'w')
        f.write('[' + self.data_file_name + ']\n')
        f.write("Labels : " + " ".join(self.label_names) + " \n")
        f.write(" \n")
        f.write("Features : " + " ".join(self.feature_names) + " \n")

        current_label_name = ""
        for i in range(len(self.labels)):
            if current_label_name != self.labels[i]:
                f.write(self.labels[i] + " ---------- \n")
                current_label_name = self.labels[i]
            f.write("> " + ' '.join(map(str, self.features[i])) + ' \n')

        f.close()

    def get_hsv_data(self, image):
        image = cv2.flip(image, -1)
        height, width, channel = image.shape

        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        rgb = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        h, s, v = cv2.split(hsv)

        top = int(height / 2 - 2 * height / 10)
        bottom = int(height / 2)
        left = int(width / 2 - width / 10)
        right = int(width / 2 + width / 10)

        h = h[top:bottom, left:right]
        s = s[top:bottom, left:right]
        v = v[top:bottom, left:right]

        cv2.rectangle(rgb, (left, top), (right, bottom), (255, 0, 0), 2)

        mean_h = int(np.mean(h)) + 20
        mean_s = int(np.mean(s))
        mean_v = int(np.mean(v))

        #print(mean_h, mean_s, mean_v)
        self.current_feature_value = [mean_h, mean_s, mean_v]

        return self.current_feature_value

    def divide_data(self):
        self.labels, self.features = zip(*sorted(zip(self.labels, self.features)))

        divided_labels = [None] * self.label_num
        divided_features = [None] * self.label_num

        current_label_name = self.labels[0]
        prev_idx = 0

        for i in range(1, len(self.labels)):
            if current_label_name != self.labels[i] or i == len(self.labels) - 1:
                idx = self.label_names.index(self.labels[i - 1])
                if i == len(self.labels) - 1:
                    i = len(self.labels)
                else:
                    current_label_name = self.labels[i]
                divided_labels[idx] = self.labels[prev_idx:i]
                divided_features[idx] = self.features[prev_idx: i]
                prev_idx = i

        self.divided_labels = divided_labels
        self.divided_features = divided_features

    def remove_outlier(self):
        self.divide_data()

        divided_features = []
        divided_labels = []
        for i in range(self.label_num):
            features = list(self.divided_features[i])
            labels = list(self.divided_labels[i])
            length = len(labels)
            features = features[int(length / 5):int(length - length / 5)]
            means = np.array(np.mean(features, axis=0))
            stds = np.array(np.std(features, axis=0))

            for j in range(length - len(features)):
                random_val = uniform(-2, 2)
                stds = np.array(random_val * stds)
                feature = []
                for k in range(self.feature_num):
                    temp = int(means[k] + stds[k])
                    if temp > ColorClassifier.upper_hsv[k]:
                        temp = ColorClassifier.upper_hsv[k]
                    elif temp < 0:
                        temp = 0
                    feature.append(temp)
                features.append(feature)
            divided_features.append(features)
            divided_labels.append(labels)

        self.divided_features = divided_features
        self.divided_labels = divided_labels

        sorted_labels = []
        sorted_features = []

        for i in range(self.label_num):
            sorted_labels += self.divided_labels[i]
            sorted_features += self.divided_features[i]

        self.labels = sorted_labels
        self.features = sorted_features

    def is_in_labels(self, label, is_print=True):
        if label in self.label_names or label in self.label_keys:
            return True
        if is_print:
            print("The label '" + label + "' does not exist.")
        return False

    def __select_hsv_value(self, values, features):
        num = len(values)
        if num == 3:
            return features
        features = np.array(features)

        y = len(features)
        h = features[0:y + 1, 0:1]
        s = features[0:y + 1, 1:2]
        v = features[0:y + 1, 2:3]

        hsv_list = []
        check_hsv = False
        if values.find('h') != -1:
            check_hsv = True
            hsv_list.append(h)
        if values.find('s') != -1:
            check_hsv = True
            hsv_list.append(s)
        if values.find('v') != -1:
            check_hsv = True
            hsv_list.append(v)
            
        if check_hsv == False:
            return None
        if num == 1:
            return hsv_list[0].tolist()
        else:
            return np.hstack((hsv_list[0], hsv_list[1])).tolist()

    def __create_folders(self):
        try:
            if not (os.path.isdir(os.path.join(self.absPATH, DEMO, self.demo_name))):
                os.makedirs(os.path.join(self.absPATH, DEMO, self.demo_name))

        except OSError as e:
            if e.errno != errno.EEXIST:
                print("Failed to create path.")
                raise

    def __create_base_folders(self):
        try:
            if not (os.path.isdir(self.absPATH)):
                os.makedirs(self.absPATH)

            if not (os.path.isdir(os.path.join(self.absPATH, DEMO))):
                os.makedirs(os.path.join(self.absPATH, DEMO))

        except OSError as e:
            if e.errno != errno.EEXIST:
                print("Failed to create path.")
                raise
