import numpy as np
from sim26 import PID, simular, fila
from tune26 import itae_pert, itae_ref, zn
pu = itae_pert(2.0, 4.992, 3.971)
def U(b, lim=2.0, aw='back', p=pu):
    Kc,Ti,Td=p; return PID(Kc,Kc/Ti,Kc*Td,N=10,b=b,c=0,lim=lim,aw=aw,Kb=1/np.sqrt(Ti*Td))
print('--- lazo único ITAE-pert, barrido de b')
for b in (0.0, 0.1, 0.2, 0.3):
    fila(f'b={b}', *simular('unico', U(b)))
pc = itae_ref(2.0, 3.815, 3.265)
def C(b=1.0, lim=2.0, aw='back', p=pc):
    Kc,Ti,Td=p
    return PID(Kc,Kc/Ti,Kc*Td,N=10,b=b,c=0,lim=lim,aw=aw,Kb=1/np.sqrt(Ti*Td)), PID(2,1,lim=lim,aw=aw,Kb=0.5)
print('--- anti-windup, escalón de referencia 1 y 3 (pert 1)')
for amp in (1.0, 3.0):
    for aw in ('none','back','clamp'):
        fila(f'unico ITAE-pert b=0 r={amp} aw={aw}', *simular('unico', U(0.0, aw=aw), amp_r=amp), ref=amp)
        fila(f'unico ITAE-pert b=1 r={amp} aw={aw}', *simular('unico', U(1.0, aw=aw), amp_r=amp), ref=amp)
        e,i=C(aw=aw); fila(f'cascada ITAE-ref r={amp} aw={aw}', *simular('cascada', e, i, amp_r=amp), ref=amp)
        e,i=C(aw=aw, p=zn(1.3628,10.145)); fila(f'cascada ZN r={amp} aw={aw}', *simular('cascada', e, i, amp_r=amp), ref=amp)
