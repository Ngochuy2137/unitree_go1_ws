# cost.py
import torch

def stage_cost(x, u, x_ref, Q, R):
    # x: (M,4), u:(M,2), x_ref:(4,) or (M,4)
    dx = x - x_ref
    return (dx @ Q * dx).sum(dim=1) + (u @ R * u).sum(dim=1)

def terminal_cost(xN, xT, P):
    dx = xN - xT
    return (dx @ P * dx).sum(dim=1)

def compute_cost(traj, u_seq, x_ref_traj, xT, Q, R, P):
    # traj: (M, N+1,4), u_seq:(M,N,2), x_ref_traj:(N+1,4)
    # trả về cost S: shape (M,)
    M, N, _ = u_seq.shape
    S = torch.zeros(M, device=traj.device)
    for k in range(N):
        S += stage_cost(traj[:, k], u_seq[:, k], x_ref_traj[k], Q, R)
    S += terminal_cost(traj[:, -1], xT, P)
    return S
