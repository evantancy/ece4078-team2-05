# the neural network structure
# AlexNet is used here
# you may switch to other neural network models provided by torchvision.models

import torch
import torch.nn as nn
import torchvision.models as models
import numpy as np


class BaselineNet(nn.Module):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    def __init__(self, num_classes=3):
        super(BaselineNet, self).__init__()
        #pre_trained_backbone = models.alexnet(pretrained=True)
        pre_trained_backbone = models.vgg16(pretrained=True)
        alex_features = pre_trained_backbone.features
        #self.features = nn.Sequential(*list(
        #            alex_features.children())[:-1])
        self.features = nn.Sequential(*list(
                    alex_features.children())[:-1])
        self.fc = nn.Sequential(
                                nn.Linear(8192, 512),
                                nn.ReLU(),
                                nn.Dropout(p=0.3),
                                nn.Linear(512, 64),
                                nn.ReLU(),
                                nn.Dropout(p=0.3),
                                nn.Linear(64, num_classes))

    def forward(self, x):
        # turns off the backwards gradient to the feature extractor backbone
        with torch.no_grad():
            x = self.features(x)
        x = torch.flatten(x, 1)
        x = self.fc(x)
        return x


if __name__ == '__main__':
    baselinenet = BaselineNet(3)
    print(baselinenet)
