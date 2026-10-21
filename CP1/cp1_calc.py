# Verificación numérica de CP-1 (equivalente Python del script MATLAB)
import numpy as np, control as ct
s = ct.tf('s')
Gv = 1.652/(0.2*s+1)
Gs = 1.183/((8.34*s+1)*(0.502*s+1))
Gf = -3.34*(0.524*s+1)/((8.34*s+1)*(0.502*s+1))
H  = 1/(0.75*s+1)
G1 = Gv*Gs*H; G2 = Gf*H

# 1) Ku, Tu
gm, pm, wcg, wcp = ct.margin(G1)
Ku, Tu = gm, 2*np.pi/wcg
print(f"Ku={Ku:.4f}  wu={wcg:.4f} rad/min  Tu={Tu:.4f} min")

# 2) PORT
t = np.linspace(0, 60, 60001)
_, y = ct.step_response(G1, t)
K = y[-1]
t28 = t[np.argmax(y >= 0.283*K)]; t63 = t[np.argmax(y >= 0.632*K)]
T = 1.5*(t63-t28); L = t63-T
print(f"K={K:.4f} t28={t28:.3f} t63={t63:.3f} T={T:.3f} L={L:.3f} L/T={L/T:.3f}")
r = L/T

tun = {}
tun['ZN-Ku']  = (Ku/1.6, Tu/2, Tu/8)
tun['ZN-PORT']= (1.2*T/(K*L), 2*L, 0.5*L)
tun['CC']     = (T/(K*L)*(4/3+r/4), L*(32+6*r)/(13+8*r), 4*L/(11+2*r))
tun['IAE-ref']= (1.086/K*r**-0.869, T/(0.740-0.130*r), 0.348*T*r**0.914)
tun['IAE-pert']=(1.435/K*r**-0.921, T/0.878*r**0.749, 0.482*T*r**1.137)
tun['ITAE-ref']=(0.965/K*r**-0.855, T/(0.796-0.147*r), 0.308*T*r**0.929)
tun['ITAE-pert']=(1.357/K*r**-0.947, T/0.842*r**0.738, 0.381*T*r**0.995)

def real(kc,ti,td): return kc*(1+1/(ti*s))*(td*s+1)
def ideal(kc,ti,td): return kc*(1+1/(ti*s)+td*s)

def metrics(sys, tt, ref=True):
    _, y = ct.step_response(sys, tt)
    fin = y[-1]
    if ref:
        mp = (y.max()-fin)/abs(fin)*100
        band = np.where(np.abs(y-fin) > 0.02*abs(fin))[0]
        ts = tt[band[-1]] if len(band) else 0
        iae = np.trapezoid(np.abs(1-y), tt)
        return f"Mp={mp:5.1f}%  ts={ts:5.2f}  IAE={iae:5.2f}"
    else:
        pk = y[np.argmax(np.abs(y))]
        band = np.where(np.abs(y) > 0.02*abs(pk))[0]
        ts = tt[band[-1]] if len(band) else 0
        iae = np.trapezoid(np.abs(y), tt)
        return f"pico={pk:6.3f}  ts={ts:5.2f}  IAE={iae:5.2f}"

tt = np.linspace(0, 40, 40001)
print()
for name,(kc,ti,td) in tun.items():
    print(f"{name:10s} Kc={kc:.3f} Ti={ti:.3f} Td={td:.3f}")
    forms = [('real',real),('ideal',ideal)] if name in ('ZN-Ku','ZN-PORT','CC') else [('ideal',ideal)]
    for fn,f in forms:
        Gc = f(kc,ti,td)
        R = ct.minreal(Gc*G1/(1+Gc*G1), verbose=False)
        D = ct.minreal(G2/(1+Gc*G1), verbose=False)
        print(f"   {fn:5s} ref: {metrics(R,tt)} | pert: {metrics(D,tt,False)}")

# ZN-Ku real -> ideal -> paralelo
kcp,tip,tdp = tun['ZN-Ku']
kc = kcp*(1+tdp/tip); ti = tip+tdp; td = tip*tdp/(tip+tdp)
print(f"\nZN-Ku serie -> ideal: Kc={kc:.4f} Ti={ti:.4f} Td={td:.4f}")
print(f"paralelo: P={kc:.4f} I={kc/ti:.4f} D={kc*td:.4f}  Kb(PID)={1/np.sqrt(ti*td):.4f}")
