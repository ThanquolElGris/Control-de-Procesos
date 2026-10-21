# Gráficas de CP-2: (1g) respuesta del mezclador y (2) controladores PI
import numpy as np, control as ct, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
s = ct.tf('s'); K, T = 0.0125, 2.5
G = 0.05/(10*s+4); Gd = 1/(10*s+4)

# ---- Parte 1g: modelo NO lineal vs lineal, escalones de 10 % ----
def nolineal(F1, x2, tf=25, dt=1e-3):
    n = int(tf/dt); x3 = 0.55; out = np.zeros(n)
    for k in range(n):
        tt = k*dt; f1 = F1 if tt >= 1 else 3.0; xx2 = x2 if tt >= 1 else 0.4
        x3 += dt*(0.6*f1 + xx2*1 - x3*(f1+1))/10; out[k] = x3
    return np.arange(n)*dt, out
fig, ax = plt.subplots(3, 1, figsize=(8, 9))
t, y = nolineal(3.3, 0.4); ax[0].plot(t, y, label='no lineal')
ax[0].plot(t, 0.55 + 0.00375*(1-np.exp(-np.clip(t-1,0,None)/2.5)), '--', label='lineal')
ax[0].axhline(0.55375, color='gray', lw=.6); ax[0].axvline(11, color='gray', lw=.6)
ax[0].set_title('x3 ante +10 % en F1 (3 → 3.3): x3 → 0.55375, ts ≈ 10 min'); ax[0].legend()
ax[1].step([0,1,25],[4,4.3,4.3], where='post'); ax[1].set_title('F3 ante +10 % en F1: salta de 4 a 4.3 (sin dinámica)')
t, y = nolineal(3.0, 0.44); ax[2].plot(t, y, label='no lineal')
ax[2].plot(t, 0.55 + 0.01*(1-np.exp(-np.clip(t-1,0,None)/2.5)), '--', label='lineal')
ax[2].axhline(0.56, color='gray', lw=.6); ax[2].axvline(11, color='gray', lw=.6)
ax[2].set_title('x3 ante +10 % en x2 (0.4 → 0.44): x3 → 0.56, ts ≈ 10 min (F3 no cambia)'); ax[2].legend()
for a in ax: a.grid(True); a.set_xlabel('t [min]')
plt.tight_layout(); plt.savefig('p1g_respuestas.png', dpi=100)
print('no lineal x3 final F1+10%:', nolineal(3.3,0.4)[1][-1], ' x2+10%:', nolineal(3.0,0.44)[1][-1])

# ---- Parte 2: tss mínimo del PI 2ºO + prefiltro con |u|<=2 ----
t = np.linspace(0, 40, 20001)
def dis(tss, z=0.7):
    wn = 4/(z*tss); Kp = (2*z*wn*T-1)/K; Ti = Kp*K/(T*wn**2); return Kp, Ti
def umax(tss):
    Kp, Ti = dis(tss); Gc = Kp*(Ti*s+1)/(Ti*s)
    return abs(ct.step_response(0.02*ct.minreal(Gc/(1+Gc*G)/(Ti*s+1), verbose=False), t).outputs).max()
lo, hi = 8, 10
for _ in range(30):
    m = (lo+hi)/2; lo, hi = (lo, m) if umax(m) <= 2 else (m, hi)
tss_min = hi; Kp2, Ti2 = dis(tss_min)
print(f"tss mínimo 2ºO+prefiltro: {tss_min:.3f}  Kp={Kp2:.2f} Ti={Ti2:.3f} umax={umax(tss_min):.3f}")

# Diseños finales
disenos = {'A: cancelación (Kp=100, Ti=2.5)': (100*(T*s+1)/(T*s), 1),
           f'B: PI 2ºO + prefiltro (tss=9.5 → Kp={dis(9.5)[0]:.1f}, Ti={dis(9.5)[1]:.3f})':
               (dis(9.5)[0]*(dis(9.5)[1]*s+1)/(dis(9.5)[1]*s), 1/(dis(9.5)[1]*s+1))}
fig, ax = plt.subplots(3, 1, figsize=(8, 9))
for n, (Gc, Pf) in disenos.items():
    ax[0].plot(t, ct.step_response(0.02*ct.minreal(Pf*Gc*G/(1+Gc*G), verbose=False), t).outputs, label=n)
    ax[1].plot(t, ct.step_response(0.02*ct.minreal(Pf*Gc/(1+Gc*G), verbose=False), t).outputs, label=n)
    ax[2].plot(t, ct.step_response(0.01*ct.minreal(Gd/(1+Gc*G), verbose=False), t).outputs, label=n)
    ax[2].set_title('Δx3 ante escalón 0.01 en la perturbación x2')
ax[2].plot(t, ct.step_response(0.01*Gd, t).outputs, 'k:', label='lazo abierto (0.0025)')
ax[0].set_title('Δx3 ante escalón 0.02 en la referencia'); ax[1].set_title('ΔF1 (acción de control) ante escalón 0.02 en la referencia')
ax[1].axhline(2, color='r', ls='--', lw=.8)
for a in ax: a.grid(True); a.legend(fontsize=8); a.set_xlabel('t [min]')
plt.tight_layout(); plt.savefig('p2_control.png', dpi=100)
