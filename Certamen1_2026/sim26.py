# Certamen 1 2026: simulación temporal equivalente al modelo Simulink.
# Diferencia con 2025: la perturbación entra ENTRE G1 y G2 (antes de med2).
# Referencia escalón 1 en t = 0; perturbación escalón 1 en t = T_D. man saturado a ±2.
import numpy as np, control as ct
s = ct.tf('s')
T_D, TF, DT = 80.0, 160.0, 0.005

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
    """PID 2DOF paralelo como el bloque de Simulink:
       u = P(b·r − y) + I∫(r − y) + D·N·s/(s+N)·(c·r − y), con saturación ±lim.
       aw = 'none' | 'back' (back-calculation, ganancia Kb) | 'clamp' (integración condicional)."""
    def __init__(self, P, I=0.0, D=0.0, N=10.0, b=1.0, c=0.0, lim=None, aw='none', Kb=0.0):
        self.P, self.I, self.D, self.N, self.b, self.c = P, I, D, N, b, c
        self.lim, self.aw, self.Kb = lim, aw, Kb; self.xi = 0.0; self.xf = 0.0
    def out(self, r, y, dt):
        e = r - y
        ud = self.N*(self.D*(self.c*r - y) - self.xf) if self.D else 0.0
        u = self.P*(self.b*r - y) + self.xi + ud
        us = min(max(u, -self.lim), self.lim) if self.lim else u
        di = self.I*e
        if self.aw == 'back': di += self.Kb*(us - u)
        elif self.aw == 'clamp' and us != u and np.sign(di) == np.sign(u): di = 0.0
        self.xi += dt*di; self.xf += dt*ud
        return us

def simular(estr, ext, inn=None, amp_r=1.0, amp_d=1.0):
    n = int(TF/DT)
    g1, g2, g3, gs, gp = LTI(4/(s+2)), LTI(0.5/(2*s+1)), LTI(2/(4*s+1)), LTI(10/(s+10)), LTI(1/(3*s+2))
    d3, ds, dp = Delay(1.5, DT), Delay(0.5, DT), Delay(1.0, DT)
    med1 = med2 = 0.0; T = np.arange(n)*DT; Y = np.zeros(n); U = np.zeros(n); M2 = np.zeros(n)
    for k in range(n):
        t = T[k]; r = amp_r; d = amp_d if t >= T_D else 0.0
        if estr == 'unico': u = ext.out(r, med1, DT)
        else:               u = inn.out(ext.out(r, med1, DT), med2, DT)
        med2 = g2.step(g1.step(u, DT) + dp.step(gp.step(d, DT)), DT)
        y = d3.step(g3.step(med2, DT)); med1 = ds.step(gs.step(y, DT))
        Y[k], U[k], M2[k] = y, u, med2
    return T, Y, U, M2

def metricas(T, Y, U, ref=1.0):
    m = T < T_D; y = Y[m]/ref; t = T[m]
    mp = max(0.0, (y.max()-1)*100)
    tr = t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)] if y.max() >= 0.9 else np.inf
    b = np.where(abs(y-1) > 0.02)[0]; ts = t[b[-1]] if len(b) else 0.0
    md = T >= T_D; dev = Y[md]-ref; pk = dev.max(); iae = np.sum(abs(dev))*DT
    b2 = np.where(abs(dev) > 0.02)[0]; tsd = T[md][b2[-1]]-T_D if len(b2) else 0.0
    return dict(Mp=mp, tr=tr, ts=ts, pico=pk, ts_pert=tsd, iae=iae, e=-dev[-1], umax=abs(U).max())

def fila(nombre, T, Y, U, *_ , ref=1.0):
    m = metricas(T, Y, U, ref)
    print(f"{nombre:52s} Mp={m['Mp']:5.1f}% tr={m['tr']:5.2f} ts={m['ts']:6.2f} | pert: pico={m['pico']:+.3f} "
          f"ts={m['ts_pert']:6.2f} IAE={m['iae']:.3f} e={m['e']:+.3f} | |u|max={m['umax']:.2f}")
    return m
