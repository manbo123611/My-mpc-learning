import time                          # 只要标准库，不用 mujoco
N = 10000                            # 要测多少次
t0 = time.perf_counter()             # 开始计时（高精度）
for _ in range(N):
    ctrl = [0.0, 0.0, 0.0]           # 模拟你的三个力
    for i in range(3):               # 模拟你的 for 下发
        ctrl[i] = 196.2
dt = time.perf_counter() - t0        # 总耗时，单位秒
print("每次(µs):", dt / N * 1e6)     # 换成微秒，好和 2000 µs 比