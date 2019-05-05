import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.datasets
from bokeh.plotting import figure
from bokeh.io import show
from bokeh.models import LinearAxis, Range1d
import numpy as np
import glob, os, _pickle

_DBG =    True
_PCSIZE = 256

if not os.path.exists('models'):
    os.mkdir('models')

def readInData(dir='../ws/data',pcSize=512, train=True, dbg=False):
    pc = []
    jc = []
    sd = []
    for file in glob.glob(os.path.join(dir,'{s}x{s}_*.p'.format(s=pcSize))):
        if dbg:
            print( 'Reading in {}'.format(file) )
        tmpData = _pickle.load(open(file, 'rb'))
        if train:
            pc.extend(tmpData['pc'][::2])
            jc.extend(tmpData['jc'][::2])
            sd.extend(tmpData['sd'][::2])
        else:
            pc.extend(tmpData['pc'][1::2])
            jc.extend(tmpData['jc'][1::2])
            sd.extend(tmpData['sd'][1::2])
    pc = torch.from_numpy(np.asarray(pc))
    jc = torch.from_numpy(np.asarray(jc))
    sd = torch.from_numpy(np.asarray(sd))
    #pc.view()
    return {'pc':pc, 'jc':jc, 'sd':sd}

'''
class SD_Dataset( TensorDataset ):
    def __init__(self, transform=None, train=True, dir='..\ws\data', pcSize=512, dbg=False):
        self.transform = transform
        self.train =     train
        self.dir =       dir
        self.pcSize =    512
        self.dbg =       dbg
        self.pc =        None
        self.jc =        None
        self.sd =        None
        self.dataLen =   None
        self.readInData(pcSize)
    def __getitem__(self, idx):
        s = { 'pc':self.pc[idx].copy(),
              'jc':self.jc[idx].copy(),
              'sd':self.sd[idx] }
        if self.transform:
            s = self.transform(s)
        print( '\tidx: {}'.format(idx) )
        return s
    def __len__(self):
        return self.dataLen
    def readInData(self, pcSize=512):
        self.pc = []
        self.jc = []
        self.sd = []
        for file in glob.glob(os.path.join(self.dir,'{s}x{s}_*.p'.format(s=pcSize))):
            if self.dbg:
                print( 'Reading in {}'.format(file) )
            tmpData = _pickle.load(open(file, 'rb'))
            if self.train:
                self.pc.extend(tmpData['pc'][::2])
                self.jc.extend(tmpData['jc'][::2])
                self.sd.extend(tmpData['sd'][::2])
            else:
                self.pc.extend(tmpData['pc'][1::2])
                self.jc.extend(tmpData['jc'][1::2])
                self.sd.extend(tmpData['sd'][1::2])
        self.pc = np.asarray(self.pc)
        self.jc = np.asarray(self.jc)
        self.sd = np.asarray(self.sd)
        pcLen = self.pc.shape[0]; jcLen = self.jc.shape[0]; sdLen = self.sd.shape[0];
        if pcLen != jcLen or pcLen != sdLen:
            failMe = float(1 / 0)
        self.dataLen = pcLen
'''

class CNN( nn.Module ):
    def __init__(self):
        super(CNN,self).__init__()
        self.pc_conv1 = nn.Conv2d( in_channels=1,
                                   out_channels=32,
                                   kernel_size=12,
                                   stride=2, padding=1 )
        self.pc_relu1 = nn.ReLU( inplace=False )
        self.pc_conv2 = nn.Conv2d( in_channels=32,
                                   out_channels=8,
                                   kernel_size=6,
                                   stride=2, padding=1 )
        self.pc_relu2 = nn.ReLU( inplace=False )
        self.pc_maxpool = nn.MaxPool2d(kernel_size=2)

        self.jc_conv1 = nn.Conv1d(in_channels=1,
                                  out_channels=8,
                                  kernel_size=2,
                                  stride=1, padding=1)
        self.jc_relu1 = nn.ReLU(inplace=False)

        self.fc1 = nn.Linear(42000,1024)
        self.fc2 = nn.Linear(1024,512)
        self.fc3 = nn.Linear(512,1)

    def forward(self, x):
        #print( 'pc_0: {}'.format(x['pc'].size()) )
        pc_out = self.pc_conv1(x['pc'])
        #print( 'pc_1: {}'.format(pc_out.size()) )
        pc_out = self.pc_relu1(pc_out)
        #print( 'pc_2: {}'.format(pc_out.size()) )
        pc_out = self.pc_conv2(pc_out)
        #print( 'pc_3: {}'.format(pc_out.size()) )
        pc_out = self.pc_relu2(pc_out)
        #print( 'pc_4: {}'.format(pc_out.size()) )
        pc_out = self.pc_maxpool(pc_out)
        #print( 'pc_5: {}'.format(pc_out.size()) )


        #print( 'jc_0: {}'.format(x['jc'].size()) )
        jc_out = self.jc_conv1(x['jc'])
        #print( 'jc_1: {}'.format(jc_out.size()) )
        jc_out = self.jc_relu1(jc_out)
        #print( 'jc_2: {}'.format(jc_out.size()) )
        jc_out = jc_out.unsqueeze(3)
        #print( 'jc_3: {}'.format(jc_out.size()) )
        jc_out = F.pad(input=jc_out,
                       pad=(0,0,
                            22,0,
                            0,0,
                            0,0),
                       mode='constant',
                       value=0)
        #print( 'jc_4: {}'.format(jc_out.size()) )

        out = torch.cat((pc_out,jc_out),3)

        return out




if __name__ == '__main__':
    num_epochs = 6
    batch_size = 100
    learning_rate = 0.001

    seed = 42 #int(time.time())
    torch.manual_seed(seed)

    train_data = readInData(dir='../ws/data', pcSize=_PCSIZE, train=True, dbg=True)
    test_data  = readInData(dir='../ws/data', pcSize=_PCSIZE, train=False, dbg=True)

    train_dataset = TensorDataset(train_data['pc'],train_data['jc'],train_data['sd'])
    test_dataset  = TensorDataset(test_data['pc'], test_data['jc'], test_data['sd'])
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=False)
    test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

    model = CNN()

    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam( model.parameters(), lr=learning_rate )

    total_step = len(train_loader)
    loss_list = []
    acc_list = []
    for epoch in range(num_epochs):
        for i, (pcTensor, jcTensor, sdTensor) in enumerate(train_loader):
            ftrs = {'pc':pcTensor.unsqueeze(1).float(),'jc':jcTensor.unsqueeze(1).float()}
            lbls = sdTensor.float()
            outputs = model(ftrs)

            loss = criterion(outputs,lbls)
            loss_list.append(loss.item())

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total = lbls.size(0)
            _, predicted = torch.max(outputs.data,1)
            corr = (predicted==lbls).sum().item()
            acc_list.append(corr/float(total))

            if (i+1)%10==0:
                print('Epoch [{}/{}], Step [{}/{}], Loss: {:.4f}, Accuracy: {:.2f}%'
                  .format(epoch + 1, num_epochs, i + 1, total_step, loss.item(),
                          (corr / total) * 100))

model.eval()
with torch.no_grad():
    corr=0
    tot=0
    for data in test_loader:
        ftrs = {k: v for k, v in data.items() if k in ['pc', 'jc']}
        lbls = data['sd']
        outputs = model(ftrs)
        _, predicted = torch.max(outputs.data, 1)
        total += lbls.size(0)
        corr += (predicted == lbls).sum().item()

    print('Test Accuracy of the model on the {} test datum: {} %'.format(len(test_dataset),(corr / total) * 100))

torch.save(model.state_dict(), os.path.join('models','net_model.ckpt'))

p = figure(y_axis_label='Loss', width=850, y_range=(0, 1), title='PyTorch CNN results')
p.extra_y_ranges = {'Accuracy': Range1d(start=0, end=100)}
p.add_layout(LinearAxis(y_range_name='Accuracy', axis_label='Accuracy (%)'), 'right')
p.line(np.arange(len(loss_list)), loss_list)
p.line(np.arange(len(loss_list)), np.array(acc_list) * 100, y_range_name='Accuracy', color='red')
show(p)