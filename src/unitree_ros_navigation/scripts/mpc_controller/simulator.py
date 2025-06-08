# simulator.py
import torch

class Simulator:
    def __init__(self, model, N):
        self.model = model
        self.N = N

    def rollout(self, x0, u_seq):
        # x0: (M,4), u_seq: (M,N,2)
        M, N, _ = u_seq.shape
        traj = torch.empty(M, N+1, 4, device=x0.device)
        traj[:, 0] = x0
        x = x0
        for k in range(N):
            u = u_seq[:, k]
            x = self.model.propagate(x, u)
            traj[:, k+1] = x
        return traj  # shape (M, N+1, 4)
