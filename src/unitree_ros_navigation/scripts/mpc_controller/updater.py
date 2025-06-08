# updater.py
import torch
import torch.nn.functional as F

def update_control(u_nominal, delta_u, cost, lam):
    # u_nominal: (N,2), delta_u:(M,N,2), cost:(M,)
    # tính w: shape (M,)
    beta = torch.min(cost)
    exp_weights = torch.exp(-(cost - beta)/lam)
    w = exp_weights / exp_weights.sum()
    # weighted sum on dimension 0
    du_weighted = (w.view(-1,1,1) * delta_u).sum(dim=0)
    return u_nominal + du_weighted  # shape (N,2)
