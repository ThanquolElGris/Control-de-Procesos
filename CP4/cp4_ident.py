# CP-4: gráficas de identificación (a1 y b3): PORT con t28/t63 y Ku/Tu en el diagrama de Bode
import numpy as np, control as ct, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from cp4_calc import ku_tu, port
s = ct.tf('s'); L = 1.0
Gv = 2/(s+2); G1 = 5/(2*s+1); G2 = 3/(5*s+1)
Gi = Gv*G1; Gc1 = 0.4 + 0.2/s
plantas = {'a1_lazo_unico': Gv*G1*G2, 'b3_lazo_externo': ct.minreal(ct.feedback(Gc1*Gi, 1), verbose=False)*G2}
for nom, G in plantas.items():
    Ku, Tu, wu = ku_tu(G, L); K, t28, t63, T, Lp = port(G, L)
    t = np.linspace(0, 40, 4001); _, y = ct.step_response(G, t); td = t + L
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    ax[0].plot(td, y, label='planta (con retardo)')
    ax[0].plot(td, np.where(td > Lp, K*(1-np.exp(-(td-Lp)/T)), 0), '--', label=f'PORT: {K:.0f}e^(-{Lp:.2f}s)/({T:.2f}s+1)')
    for tt, frac in ((t28, 0.283), (t63, 0.632)):
        ax[0].plot([tt, tt, 0], [0, frac*K, frac*K], 'r:'); ax[0].plot(tt, frac*K, 'ro')
        ax[0].annotate(f't{int(round(frac*100))}={tt:.2f}', (tt, frac*K), xytext=(5, -12), textcoords='offset points')
    ax[0].set_title('Respuesta al escalón unitario en lazo abierto'); ax[0].grid(True); ax[0].legend(fontsize=8); ax[0].set_xlabel('t [s]')
    w = np.logspace(-2, 1, 2000); H = G(1j*w)
    fase = np.degrees(np.unwrap(np.angle(H)) - w*L)
    ax[1].semilogx(w, fase, label='fase [°]'); ax[1].axhline(-180, color='r', ls=':')
    ax[1].axvline(wu, color='r', ls=':'); ax[1].plot(wu, -180, 'ro')
    ax[1].annotate(f'ωu={wu:.3f} rad/s\n|G(jωu)|={1/Ku:.3f} → Ku={Ku:.3f}\nTu=2π/ωu={Tu:.2f} s', (wu, -180), xytext=(10, 20), textcoords='offset points')
    ax[1].set_title('Fase de G(jω)·e^(-jω) (cruce por −180°)'); ax[1].grid(True, which='both'); ax[1].set_xlabel('ω [rad/s]')
    plt.tight_layout(); plt.savefig(f'{nom}.png', dpi=85); plt.close()
    print(nom, 'ok')
