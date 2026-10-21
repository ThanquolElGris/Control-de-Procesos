# Gráficas de comparación de los 7 controladores (forma ideal) de CP-1
import numpy as np, control as ct, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
s = ct.tf('s')
G1 = 1.652/(0.2*s+1)*1.183/((8.34*s+1)*(0.502*s+1))/(0.75*s+1)
G2 = -3.34*(0.524*s+1)/((8.34*s+1)*(0.502*s+1))/(0.75*s+1)
tun = {'ZN-Ku (real)':(6.516,2.312,0.578,'r'),'ZN-PORT (real)':(3.471,2.958,0.739,'r'),'CC':(3.984,3.392,0.521,'i'),
       'IAE-ref':(2.504,11.649,0.597,'i'),'IAE-pert':(3.620,2.601,0.562,'i'),
       'ITAE-ref':(2.171,10.847,0.515,'i'),'ITAE-pert':(3.581,2.765,0.568,'i')}
t = np.linspace(0,40,4001)
fig,ax = plt.subplots(2,1,figsize=(9,8))
for n,(kc,ti,td,f) in tun.items():
    Gc = kc*(1+1/(ti*s))*(td*s+1) if f=='r' else kc*(1+1/(ti*s)+td*s)
    ax[0].plot(t, ct.step_response(ct.minreal(Gc*G1/(1+Gc*G1),verbose=False),t).outputs, label=n)
    ax[1].plot(t, ct.step_response(ct.minreal(G2/(1+Gc*G1),verbose=False),t).outputs, label=n)
ax[0].set_title('Escalón unitario en la referencia R'); ax[1].set_title('Escalón unitario en la perturbación F')
for a in ax: a.grid(True); a.set_xlabel('t [min]'); a.legend(fontsize=8)
ax[0].set_ylabel('C [%TO]'); ax[1].set_ylabel('C [%TO]')
plt.tight_layout(); plt.savefig('comparacion.png', dpi=110)
