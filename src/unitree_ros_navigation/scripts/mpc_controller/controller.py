# controller.py
import torch
from sampler import Sampler
from simulator import Simulator
from cost import compute_cost
from updater import update_control
from model import DoubleIntegrator
import config

class MPCController:
    def __init__(self, config:config, x0, x_ref_traj):
        self.config = config
        self.M, self.N = config.M, config.N
        self.dt = config.dt
        self.lam = config.lam
        self.device = 'cuda'
        self.model = DoubleIntegrator(self.dt, device=self.device)
        self.sampler = Sampler(self.M, self.N, config.noise_cov, self.device)
        self.sim = Simulator(self.model, self.N)
        self.x_ref_traj = torch.tensor(x_ref_traj, device=self.device)  # (N+1,4)
        self.xT = self.x_ref_traj[-1]
        # initialize u* to zeros: shape (N,2)
        self.u_star = torch.zeros(self.N, 2, device=self.device)
        # weight matrices constant
        self.Q = torch.diag(torch.tensor([200,200,5,5], device=self.device))
        self.R = torch.diag(torch.tensor([0.01,0.01], device=self.device))
        self.P = 10*self.Q

    def step(self, xk):
        # xk: (4,) CPU tensor state at current time
        x0 = xk.to(self.device).repeat(self.M,1)  # (M,4)
        delta_u = self.sampler.sample_noise()      # (M,N,2)
        u_seq = self.u_star.unsqueeze(0) + delta_u  # (M,N,2)
        # clip to bounds
        u_seq = u_seq.clamp(self.config.u_min, self.config.u_max)
        traj = self.sim.rollout(x0, u_seq)          # (M,N+1,4)
        cost = compute_cost(traj, u_seq, self.x_ref_traj, self.xT, self.Q, self.R, self.P)
        self.u_star = update_control(self.u_star, delta_u, cost, self.lam)
        return self.u_star[0]  # return first control (2,)
