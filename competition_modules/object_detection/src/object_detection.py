import torch, torchvision
import torch.nn as nn
import torch.optim as optim

from torch.optim import lr_scheduler
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

from torchvision import models
from torchvision.models.vgg import VGG

from sklearn.metrics import confusion_matrix
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import scipy.misc
import os, sys, random, time, cv2

if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')


ranges = {
    'vgg11': ((0, 3), (3, 6),  (6, 11),  (11, 16), (16, 21)),
    'vgg13': ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25)),
    'vgg16': ((0, 5), (5, 10), (10, 17), (17, 24), (24, 31)),
    'vgg19': ((0, 5), (5, 10), (10, 19), (19, 28), (28, 37))
}

cfg = {
    'vgg11': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg13': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'vgg16': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'vgg19': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
}

means     = np.array([103.939, 116.779, 123.68]) / 255. 
h, w      = 480, 640
val_h     = h
val_w     = w
n_class   = 4


class FCN16s(nn.Module):

    def __init__(self, pretrained_net, n_class):
        super().__init__()

        self.n_class = n_class

        self.pretrained_net = pretrained_net

        self.relu    = nn.ReLU(inplace = True)
        self.deconv1 = nn.ConvTranspose2d(512, 512, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn1     = nn.BatchNorm2d(512)
        self.deconv2 = nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn2     = nn.BatchNorm2d(256)
        self.deconv3 = nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn3     = nn.BatchNorm2d(128)
        self.deconv4 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn4     = nn.BatchNorm2d(64)
        self.deconv5 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, dilation=1, output_padding=1)
        self.bn5     = nn.BatchNorm2d(32)
        self.classifier = nn.Conv2d(32, n_class, kernel_size=1)

    def forward(self, x):
        output = self.pretrained_net(x)
        x5 = output['x5']  # size=(N, 512, x.H/32, x.W/32)
        x4 = output['x4']  # size=(N, 512, x.H/16, x.W/16)

        score = self.relu(self.deconv1(x5))               # size=(N, 512, x.H/16, x.W/16)
        score = self.bn1(score + x4)                      # element-wise add, size=(N, 512, x.H/16, x.W/16)
        score = self.bn2(self.relu(self.deconv2(score)))  # size=(N, 256, x.H/8, x.W/8)
        score = self.bn3(self.relu(self.deconv3(score)))  # size=(N, 128, x.H/4, x.W/4)
        score = self.bn4(self.relu(self.deconv4(score)))  # size=(N, 64, x.H/2, x.W/2)
        score = self.bn5(self.relu(self.deconv5(score)))  # size=(N, 32, x.H, x.W)
        score = self.classifier(score)                    # size=(N, n_class, x.H/1, x.W/1)
        
        return score


class VGGNet(VGG):
    def __init__(self, pretrained=True, model='vgg16', requires_grad=True, remove_fc=True, show_params=False):
        super().__init__(make_layers(cfg[model]))
        self.ranges = ranges[model]

        if pretrained:
            exec("self.load_state_dict(models.%s(pretrained=True).state_dict())" % model)

        if not requires_grad:
            for param in super().parameters():
                param.requires_grad = False

        if remove_fc: 
            del self.classifier

        if show_params:
            for name, param in self.named_parameters():
                print(name, param.size())

    def forward(self, x):
        output = {}

        for idx in range(len(self.ranges)):
            for layer in range(self.ranges[idx][0], self.ranges[idx][1]):      
                x = self.features[layer](x)
            output["x%d"%(idx+1)] = x
        return output



def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)

def find_contour(frame , h , w , min_size):
    ret, thresh = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)
    _, tmp_contours, hierarchy = cv2.findContours( thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour = []
    contour_area = []
    contour_center_x_position = []
    contour_center_y_position = []

    for i in range(len(tmp_contours)):
        cnt = tmp_contours[i]
        area = cv2.contourArea(cnt)

        if(area > ((h/min_size)*(w/min_size))):
            contour.append(cnt)
            contour_area.append(area)

            M=cv2.moments(cnt)
            contour_center_x_position.append(int(M['m10']/M['m00']))
            contour_center_y_position.append(int(M["m01"]/M["m00"]))

    return (len(contour) , contour , contour_area , contour_center_x_position , contour_center_y_position)


if __name__ == "__main__":

    vgg_model = VGGNet(requires_grad=True, remove_fc=True)
    fcn_model = FCN16s(pretrained_net=vgg_model, n_class=n_class)

    num_gpu = list(range(torch.cuda.device_count()))

    if torch.cuda.is_available():
        vgg_model = vgg_model.cuda()
        fcn_model = fcn_model.cuda()
        fcn_model = nn.DataParallel(fcn_model, device_ids=num_gpu)

    model_dir = "";
    model_path = os.path.join(model_dir, "FCNs_mini_competition_batch10_epoch9_RMSprop_lr0.0001.pkl")

    fcn_model.load_state_dict(model_path)


    cap = cv2.VideoCapture(0)

    while(cap.isOpened()):

        ret, img = cap.read()

        if (ret != True):
            break

        img = np.transpose(img,(2,0,1))/255.

        img[0] -= means[0]
        img[1] -= means[1]
        img[2] -= means[2]

        img = np.expand_dims(img, axis=0)

        input_img = torch.from_numpy(img).cuda()
        input_img = input_img.float()

        output_img = fcn_model(input_img)
        output_img = output_img.data.cpu().numpy()

        test_N, _, test_h, test_w = output_img.shape
        pred_img = output_img.transpose(0, 2, 3, 1).reshape(-1, n_class).argmax(axis = 1).reshape(test_N, test_h, test_w)

        output_frame = cv2.inRange(test_pred[i], 1, 3)
        count, contours, area, x, y = find_contour(output_frame , output_frame.shape[0] , output_frame.shape[1] , 15)  

        cv2.drawContours(output_frame , contours , -1 , 128 , thickness=2)

        center_position = []
        for items in range(count): 
            cv2.circle(output_frame, (x[items] , y[items]), 2, 128, 5)
            center_position.append((x[items] , y[items]))


        cv2.imshow("output", output_frame)
























