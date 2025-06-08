# sampler.py
import torch

class Sampler:
    def __init__(self, M, N, noise_cov, device='cuda'):
        self.M, self.N = M, N
        self.noise_std = noise_cov**0.5
        self.device = device

    def sample_noise(self):
        # Trả về tensor Δu: shape (M, N, 2)
        return torch.randn(self.M, self.N, 2, device=self.device) * self.noise_std
