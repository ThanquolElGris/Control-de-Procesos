# Verificación numérica de CP-2 (Python / python-control)
import numpy as np, control as ct
s = ct.tf('s')
G  = 0.05/(10*s+4)      # X3/F1  -> K=0.0125, T=2.5
Gd = 1/(10*s+4)         # X3/X2  -> 0.25/(2.5s+1)
K, T = 0.0125, 2.5
t = np.linspace(0, 40, 40001)

def ts2(y, tt, ref):
    fin = y[-1]; b = np.where(np.abs(y-fin) > 0.02*abs(ref))[0]
    return tt[b[-1]] if len(b) else 0.0

def evalua(nombre, Gc, Pf=1):
    Yr = ct.minreal(Pf*Gc*G/(1+Gc*G), verbose=False)
    Ur = ct.minreal(Pf*Gc/(1+Gc*G), verbose=False)
    Yd = ct.minreal(Gd/(1+Gc*G), verbose=False)
    _, yr = ct.step_response(0.02*Yr, t); _, ur = ct.step_response(0.02*Ur, t)
    _, yd = ct.step_response(0.01*Yd, t)
    mp = max(0, (yr.max()-0.02)/0.02*100)
    pk = yd[np.argmax(abs(yd))]
    bd = np.where(np.abs(yd) > 0.02*abs(pk))[0]
    print(f"{nombre:34s} ref: ts={ts2(yr,t,0.02):5.2f} Mp={mp:4.1f}% umax={abs(ur).max():5.3f} u_ss={ur[-1]:.3f} | "
          f"pert: pico={pk:.5f} ts={t[bd[-1]]:5.2f} IAE={np.trapezoid(abs(yd),t):.4f}")
    return t, yr, ur, yd

print("Lazo abierto: X3 ante 10% en F1 (0.3):", 0.0125*0.3, " ante 10% en x2 (0.04):", 0.25*0.04)
res = {}
# a) PI por cancelación: Ti = T, Kp máximo sin saturar
for Kp in (50, 80, 100):
    res[f'cancel Kp={Kp}'] = evalua(f"Cancelación Kp={Kp}, Ti=2.5", Kp*(T*s+1)/(T*s))
# b) PI 2do orden (zeta=0.7), con y sin prefiltro, barrido de tss
for tss in (4, 5, 6, 8, 10):
    z = 0.7; wn = 4/(z*tss); Kp = (2*z*wn*T-1)/K; Ti = Kp*K/(T*wn**2)
    Gc = Kp*(Ti*s+1)/(Ti*s)
    print(f"  tss={tss}: wn={wn:.3f} Kp={Kp:.2f} Ti={Ti:.3f}")
    evalua(f"  PI 2ºO tss={tss} sin prefiltro", Gc)
    r = evalua(f"  PI 2ºO tss={tss} con prefiltro", Gc, 1/(Ti*s+1))
    if tss == 5: res['2ºO tss=5 + prefiltro'] = r
np.save('res.npy', {k:v for k,v in res.items()}, allow_pickle=True)
