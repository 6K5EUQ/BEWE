"""IQResNet1D — compact 1D ResNet over raw I/Q (2 x N), ~0.65M params @ C=50."""
import torch
import torch.nn as nn


class ResBlock(nn.Module):
    def __init__(self, cin, cout, stride=1, k=5):
        super().__init__()
        p = k // 2
        self.c1 = nn.Conv1d(cin, cout, k, stride, p, bias=False)
        self.b1 = nn.BatchNorm1d(cout)
        self.c2 = nn.Conv1d(cout, cout, k, 1, p, bias=False)
        self.b2 = nn.BatchNorm1d(cout)
        self.act = nn.SiLU(inplace=True)
        if stride != 1 or cin != cout:
            self.skip = nn.Sequential(nn.Conv1d(cin, cout, 1, stride, bias=False),
                                      nn.BatchNorm1d(cout))
        else:
            self.skip = nn.Identity()

    def forward(self, x):
        h = self.act(self.b1(self.c1(x)))
        h = self.b2(self.c2(h))
        return self.act(h + self.skip(x))


class IQResNet1D(nn.Module):
    EMB = 128

    def __init__(self, n_classes: int):
        super().__init__()
        self.stem = nn.Sequential(nn.Conv1d(2, 32, 7, 1, 3, bias=False),
                                  nn.BatchNorm1d(32), nn.SiLU(inplace=True))
        self.stages = nn.Sequential(
            ResBlock(32, 32), ResBlock(32, 32),
            ResBlock(32, 64, stride=2), ResBlock(64, 64),
            ResBlock(64, 96, stride=2), ResBlock(96, 96),
            ResBlock(96, 128, stride=2), ResBlock(128, 128),
        )
        self.head = nn.Sequential(nn.Dropout(0.3), nn.Linear(256, self.EMB), nn.SiLU(inplace=True))
        self.fc = nn.Linear(self.EMB, n_classes)

    def forward(self, x):                       # x: [B, 2, N]
        h = self.stages(self.stem(x))
        pooled = torch.cat([h.mean(dim=2), h.amax(dim=2)], dim=1)   # [B, 256]
        emb = self.head(pooled)
        return self.fc(emb)

    def embed(self, x):
        h = self.stages(self.stem(x))
        return self.head(torch.cat([h.mean(dim=2), h.amax(dim=2)], dim=1))
