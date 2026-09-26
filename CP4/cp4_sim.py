# CP-4: simulación temporal de las estrategias (equivalente al modelo Simulink)
# Referencia escalón 1 en t=1; perturbación escalón 1 en t=t_d. Integración de Euler.
import numpy as np, control as ct, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
s = ct.tf('s')

class LTI:                       # bloque lineal (tf propia) en espacio de estados
    def __init__(self, G):
        ss = ct.tf2ss(G); self.A, self.B, self.C, self.D = [np.array(m, float) for m in (ss.A, ss.B, ss.C, ss.D)]
        self.x = np.zeros((self.A.shape[0], 1))
    def step(self, u, dt):
        y = (self.C @ self.x + self.D*u).item(); self.x = self.x + dt*(self.A @ self.x + self.B*u); return y

class PID:                       # P + I/s + D·N·s/(s+N), D sobre el error (como el bloque PID de Simulink)
    def __init__(self, P, I=0.0, D=0.0, N=100.0): self.P, self.I, self.D, self.N = P, I, D, N; self.xi = 0.0; self.xf = 0.0
    def step(self, e, dt):
        ud = self.N*(self.D*e - self.xf); u = self.P*e + self.xi + ud
        self.xi += dt*self.I*e; self.xf += dt*ud; return u

def simular(planta, ctrl, tf, t_d, dt=1e-3, L=0.0):
    """planta: 'p1' o 'p2'; ctrl: ('uno', PID) o ('cascada', PIDext, PIDint)."""
    n = int(tf/dt); buf = np.zeros(max(int(round(L/dt)), 1))
    if planta == 'p1': Gv, G1, G2 = LTI(2/(s+2)), LTI(5/(2*s+1)), LTI(3/(5*s+1))
    else:              Gv, G1, G2 = LTI(3/(s+3)), LTI(3/(2*s+1)), LTI(4/s)
    m1 = m2 = 0.0; R = np.zeros(n); Y = np.zeros(n); U = np.zeros(n)
    for k in range(n):
        t = k*dt; r = 1.0 if t >= 1 else 0.0; d = 1.0 if t >= t_d else 0.0
        if ctrl[0] == 'uno': u = ctrl[1].step(r - m2, dt)
        else:                u = ctrl[2].step(ctrl[1].step(r - m2, dt) - m1, dt)
        if planta == 'p1':
            a = Gv.step(u, dt) + d; m1 = G1.step(a, dt); b = G2.step(m1, dt)
            if L: m2 = buf[k % len(buf)]; buf[k % len(buf)] = b
            else: m2 = b
        else:
            m1 = G1.step(Gv.step(u, dt), dt) + d; m2 = G2.step(m1, dt)
        R[k], Y[k], U[k] = r, m2, u
    return np.arange(n)*dt, R, Y, U

def metricas(t, r, y, u, t_d):
    m = (t >= 1) & (t < t_d); yy = y[m]; tt = t[m]
    mp = max(0.0, (yy.max()-1)*100); b = np.where(abs(yy-1) > 0.02)[0]; ts = tt[b[-1]]-1 if len(b) else 0.0
    md = t >= t_d; dev = y[md]-1; pk = dev[np.argmax(abs(dev))]
    b2 = np.where(abs(dev) > 0.02)[0]; tsd = t[md][b2[-1]]-t_d if len(b2) else 0.0
    return dict(Mp=mp, ts=ts, e_ref=1-yy[-1], pico_pert=pk, ts_pert=tsd, e_pert=-dev[-1], umax=abs(u).max())

def grafica(nombre, archivo, t, r, y, u, t_d):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    ax[0].plot(t, r, 'k--', label='referencia'); ax[0].plot(t, y, label='salida (med2)'); ax[0].legend()
    ax[1].plot(t, u, 'C1'); ax[2].plot(t, r - y, 'C3')
    q = u[(t < 1) | (t > 1.5)]; pad = 0.15*(q.max()-q.min()+1e-9)
    ax[1].set_ylim(q.min()-pad, q.max()+pad)   # el pico derivativo en t=1 queda fuera de escala (ver |u|max)
    ax[0].set_title(f'{nombre}: referencia y salida'); ax[1].set_title('Acción de control u'); ax[2].set_title('Error r − med2')
    for a in ax: a.grid(True); a.axvline(t_d, color='gray', ls=':', lw=1)
    ax[2].set_xlabel(f't [s]   (escalón en la referencia en t = 1; escalón en la perturbación en t = {t_d})')
    plt.tight_layout(); plt.savefig(archivo, dpi=85); plt.close()

if __name__ == '__main__':
    casos = [
      ('Ej1a: PID ZN lazo único', 'p1', lambda: ('uno', PID(0.3025, 0.0498, 0.2941)), 120, 60, 1.0, 'ej1a.png'),
      ('Ej1b: cascada PI(MO) + PID(ZN)', 'p1', lambda: ('cascada', PID(1.2264, 0.2687, 0.8956), PID(0.4, 0.2)), 120, 60, 1.0, 'ej1b.png'),
      ('Ej2a: MS lazo único (PID)', 'p2', lambda: ('uno', PID(0.3125, 0.09375, 0.25)), 30, 15, 0.0, 'ej2a.png'),
      ('Ej2 (notas): MO lazo único (PD)', 'p2', lambda: ('uno', PID(0.125, 0.0, 0.25)), 30, 15, 0.0, 'ej2_mo.png'),
      ('Ej2b: cascada PI(MO) + PI(MS)', 'p2', lambda: ('cascada', PID(0.1875, 0.07031), PID(1.0, 0.5)), 30, 15, 0.0, 'ej2b.png'),
      ('Ej2 (notas): cascada PI(MO) + P(MO)', 'p2', lambda: ('cascada', PID(0.1875), PID(1.0, 0.5)), 30, 15, 0.0, 'ej2_momo.png'),
    ]
    for nombre, pl, mk, tf, td, L, arch in casos:
        t, r, y, u = simular(pl, mk(), tf, td, L=L)
        m = metricas(t, r, y, u, td); grafica(nombre, arch, t, r, y, u, td)
        print(f"{nombre:38s} Mp={m['Mp']:5.1f}% ts={m['ts']:6.2f}s e_ref={m['e_ref']:+.4f} | pert: pico={m['pico_pert']:+.4f} "
              f"ts={m['ts_pert']:6.2f}s e_ss={m['e_pert']:+.4f} | |u|max={m['umax']:.2f}")
