# CP-3, ejercicios 1 y 2: síntesis simbólica y validación lineal
import numpy as np, sympy as sp, control as ct
# ---------- Ejercicio 1 (simbólico) ----------
s, T1, T2, T3, K1, K2, K3 = sp.symbols('s T1 T2 T3 K1 K2 K3', positive=True)
plantas = {'a) K1K2/(s(T1s+1))': K1*K2/((T1*s+1)*s),
           'b) K1K2/((T1s+1)(T2s+1))': K1*K2/((T1*s+1)*(T2*s+1)),
           'c) K1K2K3/(s(T1s+1)(T2s+1))': K1*K2*K3/((T1*s+1)*(T2*s+1)*s),
           'd) K1K2K3/((T1s+1)(T2s+1)(T3s+1))': K1*K2*K3/((T1*s+1)*(T2*s+1)*(T3*s+1))}
for n, Gp in plantas.items():
    mo = sp.simplify(1/(2*T1*s*(T1*s+1)*Gp))
    ms = sp.simplify((4*T1*s+1)/(8*T1**2*s**2*(T1*s+1)*Gp))
    print(n); print('   MO =', sp.factor(mo), ' ->', sp.collect(sp.expand(sp.apart(mo, s)), s))
    print('   MS =', sp.factor(ms), ' ->', sp.collect(sp.expand(sp.apart(ms, s)), s))
# ---------- Ejercicio 2 (numérico) ----------
s = ct.tf('s')
def disena(Gp, Tu):
    return (ct.minreal(1/(2*Tu*s*(Tu*s+1)*Gp), verbose=False),
            ct.minreal((4*Tu*s+1)/(8*Tu**2*s**2*(Tu*s+1)*Gp), verbose=False))
t = np.linspace(0, 20, 20001)
for nombre, Gp, Tu in [('Gp1', 100/(s*(s+10)), 0.1), ('Gp2', 2/(s*(0.8*s+1)*(s+0.5)), 0.8)]:
    mo, ms = disena(Gp, Tu)
    print(f"\n{nombre}: MO = {mo}   MS = {ms}")
    for k, Gc in (('MO', mo), ('MS', ms)):
        Tr = ct.minreal(Gc*Gp/(1+Gc*Gp), verbose=False)
        Td = ct.minreal(Gp/(1+Gc*Gp), verbose=False)       # perturbación a la entrada de la planta
        _, y = ct.step_response(Tr, t); _, yr = ct.forced_response(Tr, t, t); _, yd = ct.step_response(Td, t)
        fin = y[-1]; b = np.where(abs(y-1) > 0.02)[0]
        print(f"  {k}: Mp={100*(y.max()-1):5.1f}%  ts={t[b[-1]]:5.2f}  e_ss escalón={1-y[-1]:.4f}  "
              f"e_ss rampa={t[-1]-yr[-1]:.4f}  y_ss pert={yd[-1]:.4f}  pico pert={yd.max():.4f}")
        print(f"      polos LC: {np.round(ct.poles(Tr),3)}  MF={ct.margin(Gc*Gp)[1]:.1f}°")
