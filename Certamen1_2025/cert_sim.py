# Certamen 1 2025: simulación temporal equivalente al modelo Simulink.
# Referencia escalón 1 en t = 0; perturbación escalón 1 (entrada "pert") en t = T_D.
# Mando man saturado a ±LIM; PID ideal en paralelo P + I/s + D·N·s/(s+N) con back-calculation.
import numpy as np, control as ct
s = ct.tf('s')
LIM, T_D, TF, DT = 2.0, 80.0, 160.0, 0.005

class LTI:
    def __init__(self, G):
        m = ct.tf2ss(G); self.A, self.B, self.C, self.D = [np.array(x, float) for x in (m.A, m.B, m.C, m.D)]
        self.x = np.zeros((self.A.shape[0], 1))
    def step(self, u, dt):
        y = (self.C @ self.x + self.D*u).item(); self.x = self.x + dt*(self.A @ self.x + self.B*u); return y

class Delay:
    def __init__(self, L, dt): self.buf = np.zeros(max(int(round(L/dt)), 1)); self.k = 0
    def step(self, u):
        y = self.buf[self.k]; self.buf[self.k] = u; self.k = (self.k+1) % len(self.buf); return y

class PID:
    """P, I sobre el error; D sobre el error o la medición (dmeas); filtro N; saturación y back-calculation Kb."""
    def __init__(self, P, I=0.0, D=0.0, N=10.0, Kb=0.0, lim=None, dmeas=False):
        self.P, self.I, self.D, self.N, self.Kb, self.lim, self.dmeas = P, I, D, N, Kb, lim, dmeas
        self.xi = 0.0; self.xf = 0.0
    def out(self, sp, m, dt):
        e = sp - m; v = -m if self.dmeas else e
        ud = self.N*(self.D*v - self.xf) if self.D else 0.0
        u = self.P*e + self.xi + ud
        us = min(max(u, -self.lim), self.lim) if self.lim else u
        self.xi += dt*(self.I*e + self.Kb*(us - u)); self.xf += dt*ud
        return us, u

def simular(estr, ext, inn=None):
    """estr = 'unico' (ext actúa sobre med1) o 'cascada' (ext -> referencia de inn, inn actúa sobre med2)."""
    n = int(TF/DT)
    g1, g2, g3, gs, gp = LTI(4/(s+2)), LTI(0.5/(2*s+1)), LTI(2/(4*s+1)), LTI(10/(s+10)), LTI(1/(3*s+2))
    d3, ds, dp = Delay(1.5, DT), Delay(0.5, DT), Delay(1.0, DT)
    med1 = med2 = 0.0; T = np.arange(n)*DT; Y = np.zeros(n); U = np.zeros(n); UC = np.zeros(n); M2 = np.zeros(n)
    for k in range(n):
        t = T[k]; r = 1.0; d = 1.0 if t >= T_D else 0.0
        if estr == 'unico': u, uc = ext.out(r, med1, DT)
        else:
            r2, _ = ext.out(r, med1, DT); u, uc = inn.out(r2, med2, DT)
        med2 = g2.step(g1.step(u, DT), DT)
        y = d3.step(g3.step(med2 + dp.step(gp.step(d, DT)), DT))
        med1 = ds.step(gs.step(y, DT))
        Y[k], U[k], UC[k], M2[k] = y, u, uc, med2
    return T, Y, U, UC, M2

def metricas(T, Y, U):
    m = T < T_D; y = Y[m]; t = T[m]
    mp = max(0.0, (y.max()-1)*100)
    tr = t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)] if y.max() >= 0.9 else np.inf
    b = np.where(abs(y-1) > 0.02)[0]; ts = t[b[-1]] if len(b) else 0.0
    md = T >= T_D; dev = Y[md]-1; pk = dev.max()
    b2 = np.where(abs(dev) > 0.02)[0]; tsd = T[md][b2[-1]]-T_D if len(b2) else 0.0
    return dict(Mp=mp, tr=tr, ts=ts, e_ref=1-y[-1], pico=pk, ts_pert=tsd, e_pert=-dev[-1], umax=abs(U).max())

def fila(nombre, T, Y, U, *_):
    m = metricas(T, Y, U)
    print(f"{nombre:46s} Mp={m['Mp']:5.1f}% tr={m['tr']:5.2f} ts={m['ts']:6.2f} e_ref={m['e_ref']:+.3f} | "
          f"pert: pico={m['pico']:+.3f} ts={m['ts_pert']:6.2f} e={m['e_pert']:+.3f} | |u|max={m['umax']:.2f}")
    return m
