
import time

import numpy as np
import osqp
from scipy import sparse
from scipy.io import loadmat

data = loadmat("mpc_benchmark.mat", squeeze_me=True)

A = data["Aaug"]
B = data["Baug"]
C = data["C"]
u0 = np.asarray(data["u0"]).reshape(-1)

Np = int(data["Np"])
Nc = int(data["Nc"])

Wy = np.asarray(data["Wy"]).reshape(-1)
Wdu = np.asarray(data["Wdu"]).reshape(-1)

nx, nu = B.shape
ny = C.shape[0]
nv = nu * Nc

# Condensed prediction matrices
F = np.zeros((ny * Np, nx))
G = np.zeros((ny * Np, nv))

Ap = np.eye(nx)
S = np.zeros((nx, nv))

for k in range(Np):
    Ap = A @ Ap
    S = A @ S

    j = min(k, Nc - 1)
    S[:, j*nu:(j+1)*nu] += B

    F[k*ny:(k+1)*ny] = C @ Ap
    G[k*ny:(k+1)*ny] = C @ S

# Control increments
D = np.eye(nv)
for j in range(1, Nc):
    D[j*nu:(j+1)*nu, (j-1)*nu:j*nu] -= np.eye(nu)

wy = np.tile(Wy**2, Np)
wr = np.tile(Wdu**2, Nc)

P = 2 * (G.T @ (wy[:, None] * G)
         + D.T @ (wr[:, None] * D))

P = sparse.csc_matrix(np.triu((P + P.T) / 2))

lb = np.tile(-6 - u0, Nc)
ub = np.tile(12 - u0, Nc)

solver = osqp.OSQP()
solver.setup(
    P=P,
    q=np.zeros(nv),
    A=sparse.eye(nv, format="csc"),
    l=lb,
    u=ub,
    verbose=False,
    warm_starting=True,
    eps_abs=1e-3,
    eps_rel=1e-3,
)

# Benchmark
x = np.zeros(nx)
u_prev = np.zeros(nu)
times = []

for k in range(3000):
    if k % 100 == 0:
        x[0] += 0.02
        x[2] += 0.03

    t0 = time.perf_counter_ns()

    previous = np.zeros(nv)
    previous[:nu] = u_prev

    q = 2 * (
        G.T @ (wy * (F @ x))
        - D.T @ (wr * previous)
    )

    solver.update(q=q)
    result = solver.solve()

    elapsed_ms = (time.perf_counter_ns() - t0) / 1e6

    if result.info.status_val not in (1, 2):
        raise RuntimeError(result.info.status)

    times.append(elapsed_ms)

    u_prev = result.x[:nu].copy()
    x = A @ x + B @ u_prev

times = np.asarray(times)

print(f"Np={Np}, Nc={Nc}")
print(f"Mean: {times.mean():.3f} ms")
print(f"P95:  {np.percentile(times,95):.3f} ms")
print(f"P99:  {np.percentile(times,99):.3f} ms")
print(f"Max:  {times.max():.3f} ms")
print(f">10ms: {np.sum(times > 10)} / {len(times)}")
