import numpy as np
from sim26 import PID, simular, fila
def zn(Ku, Tu): Kp, Ti, Td = Ku/1.6, Tu/2, Tu/8; return Kp*(1+Td/Ti), Ti+Td, Ti*Td/(Ti+Td)
def cc(K,T,L): r=L/T; return T/(K*L)*(4/3+r/4), L*(32+6*r)/(13+8*r), 4*L/(11+2*r)
def itae_ref(K,T,L): r=L/T; return 0.965/K*r**-0.855, T/(0.796-0.147*r), 0.308*T*r**0.929
def itae_pert(K,T,L): r=L/T; return 1.357/K*r**-0.947, T/0.842*r**0.738, 0.381*T*r**0.995
def pi_itae_pert(K,T,L): r=L/T; return 0.859/K*r**-0.977, T/0.674*r**0.680, 0.0
casos = {'unico': dict(Ku=1.5860, Tu=12.834, K=2.0, T=4.992, L=3.971),
         'cascada': dict(Ku=1.3628, Tu=10.145, K=2.0, T=3.815, L=3.265)}
def mk(p, b=1.0, lim=None, aw='none'):
    Kc, Ti, Td = p; Kb = (1/np.sqrt(Ti*Td) if Td else 1/Ti)
    return PID(Kc, Kc/Ti, Kc*Td, N=10, b=b, c=0.0, lim=lim, aw=aw, Kb=Kb)
if __name__ == '__main__':
    for estr, c in casos.items():
        print(f"\n===== {estr} =====")
        reglas = {'ZN': zn(c['Ku'], c['Tu']), 'CC': cc(c['K'],c['T'],c['L']), 'ITAE-ref': itae_ref(c['K'],c['T'],c['L']),
                  'ITAE-pert': itae_pert(c['K'],c['T'],c['L']), 'PI ITAE-pert': pi_itae_pert(c['K'],c['T'],c['L'])}
        for nom, p in reglas.items():
            print(f"-- {nom}: Kc={p[0]:.4f} Ti={p[1]:.4f} Td={p[2]:.4f}")
            for b in (1.0, 0.5, 0.0):
                ext = mk(p, b, lim=None if estr=='cascada' else 2.0, aw='back')
                inn = PID(2.0, 1.0, lim=2.0, aw='back', Kb=0.5) if estr=='cascada' else None
                fila(f"   b={b} ±2 back", *simular(estr, ext, inn))
