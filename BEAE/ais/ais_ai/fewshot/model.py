"""Open-set prototypical embedding (B): map a residual burst to a metric space
where same-transmitter bursts cluster. Trained episodically (prototypical loss);
at inference a class is a PROTOTYPE (mean embedding), so new vessels enroll from a
few bursts with no retraining, and UNKNOWN is a distance threshold — open-set native.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

from .residual import N_CHANNELS, N_AUX


class ResBlock(nn.Module):
    def __init__(self, cin, cout, stride=1, k=5):
        super().__init__()
        p = k // 2
        self.c1 = nn.Conv1d(cin, cout, k, stride, p, bias=False)
        self.b1 = nn.BatchNorm1d(cout)
        self.c2 = nn.Conv1d(cout, cout, k, 1, p, bias=False)
        self.b2 = nn.BatchNorm1d(cout)
        self.act = nn.SiLU(inplace=True)
        self.skip = (nn.Sequential(nn.Conv1d(cin, cout, 1, stride, bias=False),
                                   nn.BatchNorm1d(cout))
                     if stride != 1 or cin != cout else nn.Identity())

    def forward(self, x):
        h = self.act(self.b1(self.c1(x)))
        h = self.b2(self.c2(h))
        return self.act(h + self.skip(x))


class ProtoEmbed(nn.Module):
    """Residual burst [B, N_CHANNELS, L] (+ aux [B, N_AUX]) -> L2-normalized embedding [B, EMB]."""
    EMB = 64

    def __init__(self):
        super().__init__()
        self.stem = nn.Sequential(nn.Conv1d(N_CHANNELS, 32, 7, 1, 3, bias=False),
                                  nn.BatchNorm1d(32), nn.SiLU(inplace=True))
        self.stages = nn.Sequential(
            ResBlock(32, 32), ResBlock(32, 64, stride=2),
            ResBlock(64, 96, stride=2), ResBlock(96, 128, stride=2),
        )
        self.aux = nn.Sequential(nn.Linear(N_AUX, 16), nn.SiLU(inplace=True))
        self.head = nn.Sequential(nn.Dropout(0.2),
                                  nn.Linear(256 + 16, self.EMB))

    def forward(self, x, aux):
        h = self.stages(self.stem(x))
        pooled = torch.cat([h.mean(dim=2), h.amax(dim=2)], dim=1)   # [B, 256]
        z = self.head(torch.cat([pooled, self.aux(aux)], dim=1))
        return F.normalize(z, dim=1)                                # unit sphere


def sq_dist(a, b):
    """Pairwise squared euclidean [Na, Nb]. On the unit sphere this is 2 - 2*cos."""
    return (a * a).sum(1, keepdim=True) - 2 * a @ b.t() + (b * b).sum(1)[None]


def prototypical_loss(emb, y, n_support):
    """Episodic prototypical loss. emb [C*(S+Q), EMB] ordered class-major, first
    n_support per class = support. Returns (loss, query_acc)."""
    classes = y.unique()
    C = len(classes)
    protos, q_emb, q_lab = [], [], []
    for i, c in enumerate(classes):
        idx = (y == c).nonzero(as_tuple=True)[0]
        protos.append(emb[idx[:n_support]].mean(0))
        q_emb.append(emb[idx[n_support:]])
        q_lab.append(torch.full((len(idx) - n_support,), i, device=emb.device))
    protos = torch.stack(protos)                 # [C, EMB]
    q_emb = torch.cat(q_emb); q_lab = torch.cat(q_lab)
    logits = -sq_dist(q_emb, protos)             # nearest-prototype
    loss = F.cross_entropy(logits, q_lab)
    acc = (logits.argmax(1) == q_lab).float().mean()
    return loss, acc
