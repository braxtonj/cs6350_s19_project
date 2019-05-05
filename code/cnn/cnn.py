#!/usr/bin/env python
""" Convolutional Neural Net

    CNN Predictive Net for Collision Detection

    Authors: Cade Parkison and Braxton Smith
    
    University of Utah
    CS 6250 - Machine Learning
"""
import pickle
import numpy as np
import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data.sampler import SubsetRandomSampler
from torch.autograd import Variable
import torch.nn.functional as F
import torch.optim as optim
import time

seed = 13
np.random.seed(seed)
torch.manual_seed(seed)

def outputSize(in_size, kernel_size, stride, padding):
    output = int((in_size - kernel_size + 2*(padding)) / stride) + 1
    return(output)

class CNN(torch.nn.Module):
    
    #Our batch shape for input x is (480, 640)
    
    def __init__(self):
        super(CNN, self).__init__()
        
        self.conv1 = torch.nn.Conv2d(1, 32, kernel_size=12, stride=2, padding=1)
        self.conv2 = torch.nn.Conv2d(32, 8, kernel_size=8, stride=2, padding=1)
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        
        # 19660800 input features, 64 output features
        self.fc1 = torch.nn.Linear(19660800, 64)
        
        #64 input features, 10 output features for our 10 defined classes
        self.fc2 = torch.nn.Linear(64, 10)
        
    def forward(self, x):
        
        #Computes the activation of the first convolution
        #Size changes from (1, 480, 640) to (32, 480, 640)
        x = F.relu(self.conv1(x))
        
        #Computes the activation of the second convolution
        #Size changes from (32, 480, 640) to (256, 480, 640)
        x = F.relu(self.conv2(x))
        
        #Size changes from (256, 480, 640) to (256, 240,320 )
        x = self.pool(x)
        
        #Reshape data to input to the input layer of the neural net
        #Size changes from (256*240*320) to (1, 19660800)
        x = x.view(-1, 256*240*320)
        
        #Computes the activation of the first fully connected layer
        #Size changes from (1, 19660800) to (1, 64)
        x = F.relu(self.fc1(x))
        
        #Computes the second fully connected layer (activation applied later)
        #Size changes from (1, 64) to (1, 10)
        x = self.fc2(x)
        return(x)


def get_train_loader(batch_size):
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=batch_size,
                                           sampler=train_sampler, num_workers=2)
    return(train_loader)

def createLossAndOptimizer(net, learning_rate=0.001):
    
    #Loss function
    loss = torch.nn.CrossEntropyLoss()
    
    #Optimizer
    optimizer = optim.Adam(net.parameters(), lr=learning_rate)
    
    return(loss, optimizer)

def trainNet(net, batch_size, n_epochs, learning_rate):
    
    #Print all of the hyperparameters of the training iteration:
    print("== HYPERPARAMETERS ==")
    print("batch_size=", batch_size)
    print("epochs=", n_epochs)
    print("learning_rate=", learning_rate)
    print("=============")
    
    #Get training data
    train_loader = get_train_loader(batch_size)
    n_batches = len(train_loader)
    
    #Create our loss and optimizer functions
    loss, optimizer = createLossAndOptimizer(net, learning_rate)
    
    #Time for printing
    training_start_time = time.time()
    
    #Loop for n_epochs
    for epoch in range(n_epochs):
        
        running_loss = 0.0
        print_every = n_batches // 10
        start_time = time.time()
        total_train_loss = 0
        
        for i, data in enumerate(train_loader, 0):
            
            #Get inputs
            inputs, labels = data
            
            #Wrap them in a Variable object
            inputs, labels = Variable(inputs), Variable(labels)
            
            #Set the parameter gradients to zero
            optimizer.zero_grad()
            
            #Forward pass, backward pass, optimize
            outputs = net(inputs)
            loss_size = loss(outputs, labels)
            loss_size.backward()
            optimizer.step()
            
            #Print statistics
            #running_loss += loss_size.data[0]
            #total_train_loss += loss_size.data[0]
            running_loss += loss_size.item()
            total_train_loss += loss_size.item()
            
            #Print every 10th batch of an epoch
            if (i + 1) % (print_every + 1) == 0:
                print("Epoch {}, {:d}% \t train_loss: {:.2f} took: {:.2f}s".format(
                        epoch+1, int(100 * (i+1) / n_batches), running_loss / print_every, time.time() - start_time))
                #Reset running loss and time
                running_loss = 0.0
                start_time = time.time()
            
        #At the end of the epoch, do a pass on the validation set
        total_val_loss = 0
        for inputs, labels in val_loader:
            
            #Wrap tensors in Variables
            inputs, labels = Variable(inputs), Variable(labels)
            
            #Forward pass
            val_outputs = net(inputs)
            val_loss_size = loss(val_outputs, labels)
            total_val_loss += val_loss_size.item()
            
        print("Validation loss = {:.2f}".format(total_val_loss / len(val_loader)))
        
    print("Training finished, took {:.2f}s".format(time.time() - training_start_time))


if __name__ == "__main__":
    # Unpickle data
    infile = open('data/data.pkl','rb')
    data_dict = pickle.load(infile)
    infile.close()

    # Generate Training and Test data from data_dict
    # train_set = 
    # test_set = 

    #Training
    n_training_samples = 2000
    train_sampler = SubsetRandomSampler(np.arange(n_training_samples, dtype=np.int64))

    #Validation
    n_val_samples = 500
    val_sampler = SubsetRandomSampler(np.arange(n_training_samples, n_training_samples + n_val_samples, dtype=np.int64))

    #Test
    n_test_samples = 500
    test_sampler = SubsetRandomSampler(np.arange(n_test_samples, dtype=np.int64))


    test_loader = torch.utils.data.DataLoader(test_set, batch_size=4, sampler=test_sampler, num_workers=2)
    val_loader = torch.utils.data.DataLoader(train_set, batch_size=128, sampler=val_sampler, num_workers=2)

    CNN = CNN()

    trainNet(CNN, batch_size=32, n_epochs=5, learning_rate=0.001)
