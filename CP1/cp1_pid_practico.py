# Simulación del PID práctico (parte Simulink de CP-1) en Python, con integración de Euler
import numpy as np, control as ct
s = ct.tf('s')
Gp = 1.652/(0.2*s+1) * 1.183/((8.34*s+1)*(0.502*s+1))      # M -> T
Gf = -3.34*(0.524*s+1)/((8.34*s+1)*(0.502*s+1))            # F -> T
H  = 1/(0.75*s+1)                                          # T -> C
# Sistemas u -> C y F -> C (cada uno en serie con el sensor H)
Pc = ct.ss(ct.series(ct.tf2ss(Gp), ct.tf2ss(H)))
Fc = ct.ss(ct.series(ct.tf2ss(Gf), ct.tf2ss(H)))

P, I, D = 8.1449, 2.8182, 3.7663     # ZN-Ku convertido a paralelo
Kb = 0.8650                          # 1/sqrt(Ti*Td)
dt, tf = 0.001, 40.0

def sim(case, r_step=1.0, f_step=0.0, sat=None, dmeas=False, N=1000.0, aw=False):
    A1,B1,C1,_ = [np.array(x) for x in (Pc.A,Pc.B,Pc.C,Pc.D)]
    A2,B2,C2,_ = [np.array(x) for x in (Fc.A,Fc.B,Fc.C,Fc.D)]
    x1 = np.zeros((A1.shape[0],1)); x2 = np.zeros((A2.shape[0],1))
    ui = 0.0; xd = 0.0   # integral y estado del filtro derivativo
    n = int(tf/dt); ys = np.zeros(n); us = np.zeros(n)
    for k in range(n):
        t = k*dt
        r = r_step if t >= 1 else 0.0
        f = f_step if t >= 1 else 0.0
        c = (C1@x1 + C2@x2).item()
        e = r - c
        vd = -c if dmeas else e              # señal que se deriva
        ud = D*N*(vd - xd)                   # D*N*s/(s+N) implementado como realimentación de integrador
        u = P*e + ui + ud
        us_ = np.clip(u, -sat, sat) if sat else u
        ui += dt*(I*e + (Kb*(us_ - u) if aw else 0.0))
        xd += dt*N*(vd - xd)
        x1 += dt*(A1@x1 + B1*us_); x2 += dt*(A2@x2 + B2*f)
        ys[k] = c; us[k] = us_
    t = np.arange(n)*dt
    return t, ys, us

def rep(name, t, y, u, ref):
    m = t >= 1
    if ref:
        mp = (y.max()-1)*100; band = np.where(np.abs(y-1) > 0.02)[0]
        print(f"{name:42s} Mp={mp:5.1f}%  ts={t[band[-1]]-1:5.2f} min  IAE={np.trapezoid(abs(1-y[m]),t[m]):5.2f}  |u|max={abs(u).max():7.2f}")
    else:
        pk = y[np.argmax(abs(y))]; band = np.where(np.abs(y) > 0.02*abs(pk))[0]
        print(f"{name:42s} pico={pk:6.3f}  ts={t[band[-1]]-1:5.2f} min  IAE={np.trapezoid(abs(y[m]),t[m]):5.2f}  |u|max={abs(u).max():7.2f}")

cases = [
 ("1 lineal, D sobre error (N=1000)",      dict(N=1000.0)),
 ("2 + saturación ±3",                     dict(N=1000.0, sat=3)),
 ("3 + filtro en la derivada (N=20)",      dict(N=20.0, sat=3)),
 ("4 + derivada de la medición",           dict(N=20.0, sat=3, dmeas=True)),
 ("5 + anti-windup back-calc (Kb=0.865)",  dict(N=20.0, sat=3, dmeas=True, aw=True)),
]
print("Escalón unitario en la referencia (t=1 min):")
for n_,kw in cases: rep(n_, *sim(n_, **kw), ref=True)
print("\nEscalón unitario en la perturbación F (t=1 min):")
for n_,kw in cases: rep(n_, *sim(n_, r_step=0, f_step=1.0, **kw), ref=False)
