# Certamen 1 2025: identificación y sintonía (lazo único y cascada)
import numpy as np, control as ct
s = ct.tf('s')
G1 = 4/(s+2); G2 = 0.5/(2*s+1); G3 = 2/(4*s+1); Gs = 10/(s+10); Gp = 1/(3*s+2)
L3, Ls, Lp = 1.5, 0.5, 1.0
def pade(L, n=10): num, den = ct.pade(L, n); return ct.tf(num, den)
E = pade(L3+Ls)

def kutu(G):
    gm, pm, wcg, wcp = ct.margin(G); return gm, 2*np.pi/wcg, wcg
def port(G, tf=80):
    t = np.linspace(0, tf, 80001); y = ct.step_response(G, t).outputs; K = y[-1]
    t28 = t[np.argmax(y >= 0.283*K)]; t63 = t[np.argmax(y >= 0.632*K)]
    T = 1.5*(t63-t28); L = t63-T; return K, T, L, t28, t63

# ---------- Lazo único: man -> med1 ----------
Gm = G1*G2*G3*Gs*E
Ku, Tu, wu = kutu(Gm); K, T, L, t28, t63 = port(Gm)
print(f"Lazo único: Ku={Ku:.4f} wu={wu:.4f} Tu={Tu:.3f} | PORT K={K:.3f} t28={t28:.3f} t63={t63:.3f} T={T:.3f} L={L:.3f} L/T={L/T:.3f}")

# ---------- Cascada: interno man -> med2 por MO ----------
Gi = G1*G2
Gc1 = ct.minreal(1/(2*0.5*s*(0.5*s+1)*Gi), verbose=False)
print("Gc1 (MO) =", Gc1)
Gcl1 = ct.minreal(ct.feedback(Gc1*Gi, 1), verbose=False); print("Lazo interno cerrado =", Gcl1)
Go = Gcl1*G3*Gs*E
Ku2, Tu2, wu2 = kutu(Go); K2, T2, L2, a2, b2 = port(Go)
print(f"Externo: Ku={Ku2:.4f} wu={wu2:.4f} Tu={Tu2:.3f} | PORT K={K2:.3f} t28={a2:.3f} t63={b2:.3f} T={T2:.3f} L={L2:.3f} L/T={L2/T2:.3f}")
