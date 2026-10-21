# Gráficas del certamen 2026 (mismos casos que main26.m)
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from sim26 import PID, simular, fila
def pid(Kc, Ti, Td, b, aw='back', lim=2.0):
    return PID(Kc, Kc/Ti, Kc*Td, N=10, b=b, c=0.0, lim=lim, aw=aw, Kb=(1/np.sqrt(Ti*Td) if Td else 1/Ti))
A_ref, A_pert = (0.5868, 7.3513, 1.2431), (0.8427, 5.0075, 1.5147)
B_ref, B_zn = (0.5512, 5.6924, 1.0168), (1.0647, 6.3406, 1.0145)
inn = lambda aw='back': PID(2.0, 1.0, lim=2.0, aw=aw, Kb=0.5)

# 1) Estrategias elegidas y efecto de b
casos = [('Único ITAE-ref, b=1', 'unico', lambda: pid(*A_ref, 1), None),
         ('Único ITAE-pert, b=1', 'unico', lambda: pid(*A_pert, 1), None),
         ('Único ITAE-pert, b=0 (elegido)', 'unico', lambda: pid(*A_pert, 0), None),
         ('Cascada: PI MO + PID ITAE-ref, b=1 (elegido)', 'cascada', lambda: pid(*B_ref, 1), inn)]
fig, ax = plt.subplots(3, 1, figsize=(9, 10))
for i, (nom, estr, e, n) in enumerate(casos):
    T, Y, U, M2 = simular(estr, e(), n() if n else None); fila(nom, T, Y, U)
    ax[0].plot(T, Y, color=f'C{i}', label=nom); ax[1].plot(T, U, color=f'C{i}', label=nom)
    m = (T > 75) & (T < 125); ax[2].plot(T[m], Y[m]-1, color=f'C{i}', label=nom)
ax[0].axhline(1, color='k', ls='--', lw=0.8); ax[0].set_title('Salida: ref. 1 en t = 0, pert. 1 en t = 80 s')
ax[1].set_title('Mando man (límite ±2)'); ax[1].set_ylim(-0.6, 2.2)
ax[2].set_title('Zoom: desviación de la salida por la perturbación'); ax[2].set_xlabel('t [s]')
for a in ax: a.grid(True); a.legend(fontsize=7)
plt.tight_layout(); plt.savefig('fig_estrategias.png', dpi=85); plt.close()

# 2) Anti-windup con referencia 3 (man_ss = 1.5, el transitorio satura)
fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
for col, (nom, estr, p, b) in enumerate((('Único ITAE-pert, b=1', 'unico', A_pert, 1), ('Cascada ITAE-ref, b=1', 'cascada', B_ref, 1))):
    for j, aw in enumerate(('none', 'back', 'clamp')):
        T, Y, U, M2 = simular(estr, pid(*p, b, aw), inn(aw) if estr == 'cascada' else None, amp_r=3.0)
        fila(f'{nom} r=3 aw={aw}', T, Y, U, ref=3.0)
        m = T < 60; ax[0, col].plot(T[m], Y[m], color=f'C{j}', label=aw); ax[1, col].plot(T[m], U[m], color=f'C{j}', label=aw)
    ax[0, col].axhline(3, color='k', ls='--', lw=0.8); ax[0, col].set_title(f'{nom}: salida (r = 3)')
    ax[1, col].set_title('Mando man'); ax[1, col].set_xlabel('t [s]')
for a in ax.ravel(): a.grid(True); a.legend(title='anti-windup', fontsize=8)
plt.tight_layout(); plt.savefig('fig_antiwindup.png', dpi=85); plt.close()
