# config.py
M = 2000           # số particle
N = 20             # horizon length
dt = 0.05          # bước rời rạc (s)
lam = 1.0          # temperature λ
noise_cov = 0.5    # độ lớn nhiễu (covariance scalar or diag)
# bounds cho mỗi thành phần input
u_x_min, u_x_max = -1, 2   
u_y_min, u_y_max = -1, 1