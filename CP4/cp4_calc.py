# CP-4: cálculo de controladores (Python / python-control)
import numpy as np, control as ct
from scipy.optimize import brentq
s = ct.tf('s')

def ku_tu(G, L):
    """Ku y Tu de G(s)·e^{-Ls}: frecuencia donde la fase total cruza -180°."""
    ws = np.logspace(-3, 2, 20000)
    fase = np.unwrap(np.angle(G(1j*ws))) - ws*L          # fase continua [rad]
    i = np.argmax(fase <= -np.pi)
    f = lambda w: np.interp(w, ws, fase) + np.pi
    wu = brentq(f, ws[i-1], ws[i]); Ku = 1/abs(G(1j*wu)); return Ku, 2*np.pi/wu, wu

def port(G, L, tf=200):
    t = np.linspace(0, tf, 200001); _, y = ct.step_response(G, t); K = y[-1]
    t28 = t[np.argmax(y >= 0.283*K)] + L; t63 = t[np.argmax(y >= 0.632*K)] + L
    T = 1.5*(t63-t28); return K, t28, t63, T, t63-T

def zn(Ku, Tu):
    kc_, ti_, td_ = Ku/1.6, Tu/2, Tu/8                   # PID serie
    kc = kc_*(1+td_/ti_); ti = ti_+td_; td = ti_*td_/(ti_+td_)  # ideal
    return (kc_, ti_, td_), (kc, ti, td), (kc, kc/ti, kc*td)   # paralelo P, I, D

if __name__ == '__main__':
    print("================ EJERCICIO 1 ================")
    Gv = 2/(s+2); G1 = 5/(2*s+1); G2 = 3/(5*s+1); L = 1.0
    Ga = Gv*G1*G2
    Ku, Tu, wu = ku_tu(Ga, L); print(f"a1) Ku={Ku:.4f}  wu={wu:.4f} rad/s  Tu={Tu:.4f} s")
    K, t28, t63, T, Lp = port(Ga, L); print(f"    PORT: K={K:.3f} t28={t28:.3f} t63={t63:.3f} T={T:.3f} L={Lp:.3f} L/T={Lp/T:.3f}")
    ser, idl, par = zn(Ku, Tu)
    print(f"a2) ZN serie: K'c={ser[0]:.4f} T'i={ser[1]:.4f} T'd={ser[2]:.4f}")
    print(f"    ideal: Kc={idl[0]:.4f} Ti={idl[1]:.4f} Td={idl[2]:.4f} | paralelo: P={par[0]:.4f} I={par[1]:.4f} D={par[2]:.4f}")
    # b) interno MO sobre Gv*G1 = 5/((0.5s+1)(2s+1)), Tu=0.5
    Gi = Gv*G1; Gc1 = ct.minreal(1/(2*0.5*s*(0.5*s+1)*Gi), verbose=False); print("b1) Gc1 =", Gc1)
    Gcl1 = ct.minreal(ct.feedback(Gc1*Gi, 1), verbose=False); print("    lazo interno cerrado:", Gcl1)
    Gb = Gcl1*G2
    Ku2, Tu2, wu2 = ku_tu(Gb, L); print(f"b3) Ku={Ku2:.4f} wu={wu2:.4f} Tu={Tu2:.4f}")
    K2, t282, t632, T2, Lp2 = port(Gb, L); print(f"    PORT: K={K2:.3f} t28={t282:.3f} t63={t632:.3f} T={T2:.3f} L={Lp2:.3f} L/T={Lp2/T2:.3f}")
    ser2, idl2, par2 = zn(Ku2, Tu2)
    print(f"b4) ZN serie: K'c={ser2[0]:.4f} T'i={ser2[1]:.4f} T'd={ser2[2]:.4f}")
    print(f"    ideal: Kc={idl2[0]:.4f} Ti={idl2[1]:.4f} Td={idl2[2]:.4f} | paralelo: P={par2[0]:.4f} I={par2[1]:.4f} D={par2[2]:.4f}")

    print("\n================ EJERCICIO 2 ================")
    Gv = 3/(s+3); G1 = 3/(2*s+1); G2 = 4/s
    Gp = Gv*G1*G2; Tu = 1/3
    ms = ct.minreal((4*Tu*s+1)/(8*Tu**2*s**2*(Tu*s+1)*Gp), verbose=False); mo = ct.minreal(1/(2*Tu*s*(Tu*s+1)*Gp), verbose=False)
    print("a) Gc MS lazo único =", ms, "\n   Gc MO lazo único =", mo)
    Gi = Gv*G1; Gcin = ct.minreal(1/(2*Tu*s*(Tu*s+1)*Gi), verbose=False); print("b1) Gc interno MO =", Gcin)
    print("   lazo interno cerrado:", ct.minreal(ct.feedback(Gcin*Gi, 1), verbose=False))
    Tu2 = 2/3; Go = 1/(Tu2*s+1)*G2
    exms = ct.minreal((4*Tu2*s+1)/(8*Tu2**2*s**2*(Tu2*s+1)*Go), verbose=False); exmo = ct.minreal(1/(2*Tu2*s*(Tu2*s+1)*Go), verbose=False)
    print("b3) Gc externo MS =", exms, "\n   Gc externo MO =", exmo)
