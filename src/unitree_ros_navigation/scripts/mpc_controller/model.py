# model.py (dùng PyTorch cho GPU)
import torch

class DoubleIntegrator:
    def __init__(self, dt, device='cuda'):
        self.dt = dt
        self.device = device

    def propagate(self, x, u):
        # x: (M, 4) tensor, u: (M, 2) tensor for current step
        # state = [x, y, vx, vy]
        # x_next = A x + B u
        pos = x[:, :2] + x[:, 2:]*self.dt + 0.5*u*self.dt**2
        vel = x[:, 2:] + u*self.dt
        return torch.cat([pos, vel], dim=1)
