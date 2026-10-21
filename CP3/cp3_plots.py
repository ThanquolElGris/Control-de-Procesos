# Gráficas de CP-3
import numpy as np, control as ct, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from cp3_sim import ej3, ej4
s = ct.tf('s')
# ---- Ejercicio 2 ----
for nombre, Gp, Tu, tf in (('Gp1', 100/(s*(s+10)), 0.1, 4), ('Gp2', 2/(s*(0.8*s+1)*(s+0.5)), 0.8, 40)):
    mo = ct.minreal(1/(2*Tu*s*(Tu*s+1)*Gp), verbose=False)
    ms = ct.minreal((4*Tu*s+1)/(8*Tu**2*s**2*(Tu*s+1)*Gp), verbose=False)
    t = np.linspace(0, tf, 8001); fig, ax = plt.subplots(3, 1, figsize=(8, 9))
    for k, Gc in (('MO', mo), ('MS', ms)):
        Tr = ct.feedback(Gc*Gp, 1)
        ax[0].plot(t, ct.step_response(Tr, t).outputs, label=k)
        ax[1].plot(t, t - ct.forced_response(Tr, t, t).outputs, label=k)
        ax[2].plot(t, ct.step_response(ct.feedback(Gp, Gc), t).outputs, label=k)
    ax[0].set_title(f'{nombre}: escalón unitario en la referencia (salida)')
    ax[1].set_title('Rampa unitaria en la referencia: error r - y')
    ax[2].set_title('Escalón unitario en la perturbación (entrada de la planta): salida')
    for a in ax: a.grid(True); a.legend(); a.set_xlabel('t')
    plt.tight_layout(); plt.savefig(f'ej2_{nombre}.png', dpi=90); plt.close()
# ---- Ejercicio 3 ----
fig, ax = plt.subplots(2, 1, figsize=(9, 8))
casos = [('PID único (D sobre e), sin límite', dict(estr='uno', dmeas=False)),
         ('PID único, ±10 sin antiwindup', dict(estr='uno', lim=10, dmeas=False)),
         ('PID único, ±10 con antiwindup', dict(estr='uno', lim=10, aw=True, dmeas=False)),
         ('Cascada 2 PI, sin límite', dict(estr='cascada')),
         ('Cascada 2 PI, ±10 sin antiwindup', dict(estr='cascada', lim=10)),
         ('Cascada 2 PI, ±10 con antiwindup', dict(estr='cascada', lim=10, aw=True))]
for n, kw in casos:
    t, y, u = ej3(**kw, dt=2e-5); ax[0].plot(t, y, label=n); ax[1].plot(t, u, label=n)
ax[0].set_title('Ej. 3: salida (ref = 1 en t=0; perturbación +1 en el mando en t=1 s)')
ax[1].set_title('Acción de control u'); ax[1].set_ylim(-15, 30)
for a in ax: a.grid(True); a.legend(fontsize=7); a.set_xlabel('t [s]')
plt.tight_layout(); plt.savefig('ej3.png', dpi=90); plt.close()
# ---- Ejercicio 4 ----
fig, ax = plt.subplots(2, 1, figsize=(9, 8))
for i, gc2 in enumerate(('MO', 'MS')):
    for n, kw in (('sin límite', {}), ('±5000 sin antiwindup', dict(lim=5000)),
                  ('±5000 con antiwindup', dict(lim=5000, aw=True)), ('D sobre la medición', dict(dmeas=True))):
        kw.setdefault('dmeas', False)
        t, y, u = ej4(gc2, **kw, t_d=5, tf=10, dt=2e-5); ax[i].plot(t, y, label=n)
    ax[i].set_title(f'Ej. 4: Gc1 = MO, Gc2 = {gc2}  (ref = 1 en t=0; perturbación 1000 en el mando en t=5 s)')
    ax[i].grid(True); ax[i].legend(fontsize=8); ax[i].set_xlabel('t [s]')
plt.tight_layout(); plt.savefig('ej4.png', dpi=90); plt.close()
