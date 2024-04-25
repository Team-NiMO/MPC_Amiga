import numpy as np

class A_star_path:

    def __init__(self, x_list, y_list, yaw_list):
        if len(x_list)>0 and len(y_list) > 0 and len(yaw_list)>0:
            self.x_path = x_list
            self.y_path = y_list
            self.yaw_path = yaw_list
            self.new_path_acq = False
        else:
            self.x_path = []
            self.y_path = []
            self.yaw_path = []
            self.new_path_acq = False

    def update_path(self, x_path, y_path, yaw_path = []):
        self.x_path = x_path
        self.y_path = y_path
        self.yaw_path = yaw_path
        self.new_path_acq = True

    def read_path(self):
        self.new_path_acq = False
        # return self.x_path, self.y_path, self.yaw_path, self.new_path_acq