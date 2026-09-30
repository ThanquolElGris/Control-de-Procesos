# CP-3, ejercicios 3 y 4: simulación no lineal (saturación, anti-windup, perturbación en el mando)
# PID según el diagrama de la guía: P e I sobre el error, D sobre la medición con filtro N, back-calculation Kb.
import numpy as np

class PID:
    def __init__(s, P, I=0.0, D=0.0, N=100.0, Kb=0.0, lim=None, dmeas=True):
        s.P, s.I, s.D, s.N, s.Kb, s.lim, s.dmeas = P, I, D, N, Kb, lim, dmeas; s.xi = 0.0; s.xf = 0.0
    def out(s, sp, med, dt):
        e = sp - med
        vd = -med if s.dmeas else e                 # señal que se deriva
        ud = s.N*(s.D*vd - s.xf) if s.D else 0.0
        u = s.P*e + s.xi + ud
        us = min(max(u, -s.lim), s.lim) if s.lim else u
        s.xi += dt*(s.I*e + s.Kb*(us - u))
        if s.D: s.xf += dt*ud
        return us

def lag(x, u, T, dt): return x + dt*(u - x)/T

def metr(t, y, ref, t_d):
    m = t < t_d; yy = y[m]; tt = t[m]
    b = np.where(abs(yy-ref) > 0.02*ref)[0]
    return (yy.max()-ref)/ref*100, (tt[b[-1]] if len(b) else 0)

# ---------------- Ejercicio 3 ----------------
# U -> 1/(0.01s+1) -> 1/(0.1s+1) -> V -> 1/(0.1s+1) -> Y
def ej3(estr, lim=None, aw=False, dmeas=True, d=1.0, t_d=1.0, tf=2.0, dt=1e-5):
    n = int(tf/dt); x1 = v = y = 0.0; Y = np.zeros(n); U = np.zeros(n)
    if estr == 'uno':
        c = PID(10, 50, 0.5, N=1000, Kb=(50/10 if aw else 0), lim=lim, dmeas=dmeas)
    else:
        c2 = PID(2.5, 25, Kb=0)                                   # externo (sin límite)
        c1 = PID(5, 50, Kb=(50/5 if aw else 0), lim=lim)          # interno (mando)
    for k in range(n):
        t = k*dt
        if estr == 'uno': u = c.out(1.0, y, dt)
        else: u = c1.out(c2.out(1.0, y, dt), v, dt)
        dd = d if t >= t_d else 0.0
        x1 = lag(x1, u + dd, 0.01, dt); v = lag(v, x1, 0.1, dt); y = lag(y, v, 0.1, dt)
        Y[k] = y; U[k] = u
    return np.arange(n)*dt, Y, U

# ---------------- Ejercicio 4 ----------------
# U -> 1/(0.1s+1) -> 1/(0.01s+1) -> V(lazo interno) -> 1/(2s+1) -> 1/s -> C
def ej4(gc2, lim=None, aw=False, dmeas=True, d=1000.0, t_d=2.0, tf=4.0, dt=1e-5, N2=None):
    n = int(tf/dt); x1 = v = w = c = 0.0; Y = np.zeros(n); U = np.zeros(n)
    c1 = PID(5, 50, Kb=(10 if aw else 0), lim=lim)
    if gc2 == 'MO': c2 = PID(25, 0, 50, N=N2 or 1000, dmeas=dmeas)
    else:           c2 = PID(650, 312.5, 50, N=N2 or 1000, Kb=0, dmeas=dmeas)
    for k in range(n):
        t = k*dt
        u = c1.out(c2.out(1.0, c, dt), v, dt)
        dd = d if t >= t_d else 0.0
        x1 = lag(x1, u + dd, 0.1, dt); v = lag(v, x1, 0.01, dt); w = lag(w, v, 2.0, dt); c += dt*w
        Y[k] = c; U[k] = u
    return np.arange(n)*dt, Y, U

if __name__ == '__main__':
    def fila(nom, t, y, u, td):
        mp, ts = metr(t, y, 1.0, td); m = t >= td
        dev = abs(y[m]-1).max(); b = np.where(abs(y[m]-1) > 0.02)[0]; tsd = t[m][b[-1]]-td if len(b) else 0
        print(f"{nom:44s} Mp={mp:6.1f}% ts={ts:.3f}s |u|max={abs(u).max():9.1f} | pert: desv.max={dev:.4f} ts={tsd:.3f}s y(fin)={y[-1]:.4f}")
    print("=== Ejercicio 3 (ref 1 en t=0, perturbación +1 en el mando en t=1) ===")
    for dm in (False, True):
        for lim, aw in ((None, False), (10, False), (10, True)):
            fila(f"PID único dmeas={dm} lim={lim} aw={aw}", *ej3('uno', lim, aw, dm), 1.0)
    for lim, aw in ((None, False), (10, False), (10, True)):
        fila(f"Cascada (2 PI) lim={lim} aw={aw}", *ej3('cascada', lim, aw), 1.0)
    print("\n=== Ejercicio 4 (ref 1 en t=0, perturbación 1000 en el mando en t=2) ===")
    for gc2 in ('MO', 'MS'):
        for dm in (False, True):
            for lim, aw in ((None, False), (5000, False), (5000, True)):
                fila(f"Gc2={gc2} dmeas={dm} lim={lim} aw={aw}", *ej4(gc2, lim, aw, dm), 2.0)
