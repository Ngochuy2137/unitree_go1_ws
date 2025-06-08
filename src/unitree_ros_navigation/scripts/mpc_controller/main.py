# main.py
import time
import torch
from controller import MPCController
import config

def generate_reference(p0, pT, t0, tT, dt):
    # ... hàm như trước ...
    return x_ref_traj  # list of (4,) for k=0..N

if __name__=="__main__":
    # 1. load p0, pT, t0, tT
    p0 = torch.tensor([0.,0.,0.,0.])
    pT = torch.tensor([2.,3.,0.,0.])
    t0, tT = 0., config.N*config.dt
    x_ref_traj = generate_reference(p0[:2], pT[:2], t0, tT, config.dt)

    # 2. khởi tạo controller
    ctrl = MPCController(config, p0, x_ref_traj)

    # 3. vòng lặp điều khiển
    xk = p0.clone()
    for step in range(config.N):
        u0 = ctrl.step(xk)
        # apply u0 to robot (thực tế) or mô phỏng
        xk = ctrl.sim.model.propagate(xk.to(ctrl.device), u0.to(ctrl.device))
        xk = xk.cpu()
        time.sleep(config.dt)
