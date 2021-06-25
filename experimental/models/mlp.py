from os import path

import tqdm
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

class MlpDataset(Dataset):
    def __init__(self, data):
        self.data_ls = zip(data["inputs"], data["outputs"])

    def __len__(self):
        return len(self.data_ls)

    def __getitem__(self, idx):
        data_entry = self.data_ls[idx]
        return {"x": data_entry[0], "y": data_entry[1]}

class Mlp(torch.nn.Module):
    fc_ls: list

    def __init__(n_in, n_out, n_hiddens):
        self.fc_ls = []

        widths = [n_in] + n_hiddens
        for i in range(len(widths) - 1):
            fc = nn.Linear(widths[i], widths[i+1])
            self.__setattr__(f"fc_{i}", fc)
            self.fc_ls.append(fc)
        
        self.out = nn.Linear(widths[-1], n_out)

    def forward(self, x):
        for fc in self.fc_ls:
            x = F.Relu(fc(x))
        x = self.out(x)

        return x


class MlpTrainer:
    def __init__(self, cfg):
        self.cfg = cfg

        self.net = Mlp(cfg.n_in, cfg.n_out, cfg.layers)
        self.optimizer = torch.optim.Adam(self.net.parameters())
        self.criterion = torch.nn.MSELoss()

    def get_model(self):
        return self.net

    def train(self, data):
        dataset = MlpDataset(data)
        dataloader = DataLoader(dataset, batch_size=self.cfg.batchsize, shuffle=True)


        for epoch in tqdm(range(self.cfg.epochs)):
            for batch in dataloader:
                optimizer.zero_grad()

                y = self.net(batch["x"])
                y_hat = batch["y"]

                loss = self.criterion(y, y_hat)
                loss.backward()
                optimizer.step()


    def save(self, filename):
        torch.save(self.net.state_dict(), filename)

    def load(self, filename):
        self.net.load_state_dict(torch.load(filename))