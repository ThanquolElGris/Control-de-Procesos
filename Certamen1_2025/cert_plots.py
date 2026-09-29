# Gráficas del Certamen 1 2025 (salen de cert_sim.py)
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from cert_sim import PID, simular, fila, T_D

A = dict(P=0.5868, I=0.0798, D=0.7294, Kb=0.3308)          # lazo único: PID ITAE-referencia
B = dict(P=0.5512, I=0.0968, D=0.5605, Kb=0.4155)          # cascada: PID externo ITAE-referencia
def unico(lim, aw, dm): return simular('unico', PID(A['P'], A['I'], A['D'], N=10, Kb=A['Kb'] if aw else 0, lim=lim, dmeas=dm))
def cascada(lim, aw, dm):
    ext = PID(B['P'], B['I'], B['D'], N=10, Kb=B['Kb'] if aw else 0, lim=lim, dmeas=dm)
    inn = PID(2.0, 1.0, lim=lim, Kb=0.5 if aw else 0)
    return simular('cascada', ext, inn)

variantes = [('lineal (sin límite)', None, False, False), ('±2, sin anti-windup', 2.0, False, False),
             ('±2, con anti-windup', 2.0, True, False), ('±2, anti-windup + D sobre la medición', 2.0, True, True)]

def figura(nombre, fn, archivo):
    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for i, (et, lim, aw, dm) in enumerate(variantes):
        T, Y, U, UC, M2 = fn(lim, aw, dm); fila(f"{nombre}: {et}", T, Y, U)
        ax[0].plot(T, Y, color=f'C{i}', label=et); ax[1].plot(T, U, color=f'C{i}', label=et)
    ax[0].axhline(1, color='k', ls='--', lw=0.8); ax[0].axhline(1.1, color='gray', ls=':', lw=0.8)
    ax[1].axhline(2, color='r', ls=':', lw=0.8); ax[1].axhline(-2, color='r', ls=':', lw=0.8); ax[1].set_ylim(-2.5, 4)
    ax[0].set_title(f'{nombre}: salida (ref. 1 en t = 0, pert. 1 en t = {T_D:.0f} s)'); ax[1].set_title('Mando man (la línea roja es el límite ±2)')
    for a in ax: a.grid(True); a.legend(fontsize=8)
    ax[1].set_xlabel('t [s]'); plt.tight_layout(); plt.savefig(archivo, dpi=85); plt.close()

if __name__ == '__main__':
    figura('Lazo único (PID ITAE-ref)', unico, 'fig_unico.png')
    figura('Cascada (PI MO + PID ITAE-ref)', cascada, 'fig_cascada.png')
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for i, (nom, fn) in enumerate((('Lazo único', unico), ('Cascada', cascada))):
        T, Y, U, UC, M2 = fn(2.0, True, True)
        ax[0].plot(T, Y, color=f'C{i}', label=nom); ax[1].plot(T, U, color=f'C{i}', label=nom); ax[2].plot(T, M2, color=f'C{i}', label=nom)
    ax[0].axhline(1, color='k', ls='--', lw=0.8); ax[0].axhline(1.1, color='gray', ls=':', lw=0.8)
    ax[0].set_title('Diseños finales (±2, anti-windup, D sobre la medición): salida')
    ax[1].set_title('Mando man'); ax[2].set_title('med2 (salida de G2)')
    for a in ax: a.grid(True); a.legend()
    ax[2].set_xlabel('t [s]'); plt.tight_layout(); plt.savefig('fig_comparacion.png', dpi=85); plt.close()

    # Efecto del anti-windup con una sintonía agresiva (Cohen-Coon, lazo único, D sobre el error)
    cc = dict(P=0.9631, I=0.9631/7.5411, D=0.9631*1.2615, Kb=1/np.sqrt(7.5411*1.2615))
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for i, (et, lim, aw) in enumerate((('lineal', None, False), ('±2 sin anti-windup', 2.0, False), ('±2 con anti-windup', 2.0, True))):
        T, Y, U, UC, M2 = simular('unico', PID(cc['P'], cc['I'], cc['D'], N=10, Kb=cc['Kb'] if aw else 0, lim=lim))
        fila(f"CC lazo único: {et}", T, Y, U)
        m = T < 60; ax[0].plot(T[m], Y[m], color=f'C{i}', label=et); ax[1].plot(T[m], UC[m], color=f'C{i}', label=et + ' (salida calculada del PID)')
    ax[0].axhline(1, color='k', ls='--', lw=0.8); ax[1].axhline(2, color='r', ls=':'); ax[1].set_ylim(-3, 6)
    ax[0].set_title('Windup con una sintonía agresiva (Cohen-Coon, lazo único)'); ax[1].set_title('Salida del PID antes de la saturación')
    for a in ax: a.grid(True); a.legend(fontsize=8)
    ax[1].set_xlabel('t [s]'); plt.tight_layout(); plt.savefig('fig_windup.png', dpi=85); plt.close()
