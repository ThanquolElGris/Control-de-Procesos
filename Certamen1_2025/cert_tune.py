# Barrido de sintonías (ZN, CC, criterios integrales) para ambas estrategias
import numpy as np
from cert_sim import PID, simular, fila, LIM
def zn(Ku, Tu):
    Kp, Ti, Td = Ku/1.6, Tu/2, Tu/8; return Kp*(1+Td/Ti), Ti+Td, Ti*Td/(Ti+Td)
def cc(K, T, L):
    r = L/T; return T/(K*L)*(4/3+r/4), L*(32+6*r)/(13+8*r), 4*L/(11+2*r)
def itae_ref_pid(K, T, L): r=L/T; return 0.965/K*r**-0.855, T/(0.796-0.147*r), 0.308*T*r**0.929
def itae_ref_pi(K, T, L):  r=L/T; return 0.586/K*r**-0.916, T/(1.03-0.165*r), 0.0
def itae_pert_pid(K, T, L): r=L/T; return 1.357/K*r**-0.947, T/0.842*r**0.738, 0.381*T*r**0.995
def iae_ref_pid(K, T, L): r=L/T; return 1.086/K*r**-0.869, T/(0.740-0.130*r), 0.348*T*r**0.914

def ctrl(p, lim=None, aw=False, dmeas=False):
    Kc, Ti, Td = p; P, I, D = Kc, Kc/Ti, Kc*Td
    Kb = (1/np.sqrt(Ti*Td) if Td else 1/Ti) if aw else 0.0
    return PID(P, I, D, N=10.0, Kb=Kb, lim=lim, dmeas=dmeas)

casos = {
 'unico':   dict(Ku=1.5860, Tu=12.834, K=2.0, T=4.992, L=3.971),
 'cascada': dict(Ku=1.3628, Tu=10.145, K=2.0, T=3.815, L=3.265),
}
if __name__ == '__main__':
    for estr, c in casos.items():
        print(f"\n===== {estr} =====")
        reglas = {'ZN-Ku': zn(c['Ku'], c['Tu']), 'CC': cc(c['K'], c['T'], c['L']),
                  'IAE-ref PID': iae_ref_pid(c['K'], c['T'], c['L']), 'ITAE-ref PID': itae_ref_pid(c['K'], c['T'], c['L']),
                  'ITAE-ref PI': itae_ref_pi(c['K'], c['T'], c['L']), 'ITAE-pert PID': itae_pert_pid(c['K'], c['T'], c['L'])}
        for nom, p in reglas.items():
            print(f"-- {nom}: Kc={p[0]:.4f} Ti={p[1]:.4f} Td={p[2]:.4f} -> P={p[0]:.4f} I={p[0]/p[1]:.4f} D={p[0]*p[2]:.4f}")
            for lim, aw, dm in ((None, False, False), (LIM, False, False), (LIM, True, False), (LIM, True, True)):
                ext = ctrl(p, None if estr == 'cascada' else lim, aw, dm)
                inn = PID(2.0, 1.0, lim=lim, Kb=(1.0/2.0 if aw else 0.0)) if estr == 'cascada' else None
                fila(f"   lim={lim} aw={aw} dmeas={dm}", *simular(estr, ext, inn))
