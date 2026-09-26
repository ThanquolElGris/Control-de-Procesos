# Control de Procesos — Guía de estudio (Conferencias 1, 2 y 3)

> Ramo: Control de Procesos · Dr. Angel E. Rubio R. · DIEE-UBB
> Cubre las conferencias 1 (Introducción y PID), 2 (Sintonía de controladores) y 3 (Control basado en modelo).
> Estas conferencias corresponden al **Tema 1: Control de procesos SISO**, que se evalúa en el **Certamen 1 (C1)**.

---

## Índice

1. [Mapa general del ramo](#1-mapa-general-del-ramo)
2. [Conf. 1 — Conceptos importantes](#2-conf-1--conceptos-importantes)
3. [Conf. 1 — El controlador PID](#3-conf-1--el-controlador-pid)
4. [Conf. 2 — Caracterización dinámica (modelo PORT)](#4-conf-2--caracterización-dinámica-modelo-port)
5. [Conf. 2 — Sintonía: Ziegler-Nichols y Cohen-Coon](#5-conf-2--sintonía-ziegler-nichols-y-cohen-coon)
6. [Conf. 2 — Criterios integrales del error](#6-conf-2--criterios-integrales-del-error)
7. [Conf. 2 — Selección del tipo de controlador](#7-conf-2--selección-del-tipo-de-controlador)
8. [Conf. 3 — Control de plantas de 1er orden](#8-conf-3--control-de-plantas-de-1er-orden)
9. [Conf. 3 — Módulo óptimo](#9-conf-3--módulo-óptimo)
10. [Conf. 3 — Módulo simétrico](#10-conf-3--módulo-simétrico)
11. [Conf. 3 — Control en cascada](#11-conf-3--control-en-cascada)
12. [Ejemplo integrador resuelto (PORT → ZN, CC, ITAE)](#12-ejemplo-integrador-resuelto)
13. [Trampas típicas y checklist para el certamen](#13-trampas-típicas-y-checklist-para-el-certamen)
14. [Formulario resumen](#14-formulario-resumen)

---

## 1. Mapa general del ramo

| Tema | Contenido | Evaluación |
|---|---|---|
| **T1: Control SISO** | ZN, CC, criterios integrales; control basado en modelo; cascada; feedforward; sobrecorrida y selectivo; relación y promediante | **C1** |
| T2: Compensadores | Predictor de Smith (retardo), compensador de respuesta inversa, control inferencial | C2 |
| T3: Control MIMO | Desacopladores, arreglo de Bristol (RGA), control de 1, 2 y 3 impulsos | C3 |

Nota final = promedio de los certámenes. Bibliografía: Ogata (Ing. de Control Moderna), Corripio & Smith (*Principles and Practice of Automatic Process Control*), Stephanopoulos.

**Hilo conductor de las 3 conferencias:** dado un proceso, (1) entender su comportamiento (tipo, signo de ganancia, errores en estado estable), (2) obtener un modelo simple (PORT), (3) elegir y sintonizar un controlador, ya sea con reglas empíricas (ZN, CC), por optimización (ISE/IAE/ITAE) o por síntesis basada en el modelo (cancelación, módulo óptimo/simétrico, cascada).

---

## 2. Conf. 1 — Conceptos importantes

### 2.1 ¿Qué es controlar un proceso?
Garantizar ciertos índices de desempeño en la respuesta de la planta **ante cambios en la referencia** (seguimiento / *servo*) y mantener sus variables dentro del rango deseado **ante perturbaciones** (regulación). Para hacerlo bien se necesita conocer cómo varían las salidas (variables controladas) en función de las entradas (manipulables y perturbadoras): idealmente, el **modelo dinámico**.

### 2.2 Acción directa e inversa
- El signo de la **ganancia estática** del modelo indica si el proceso es de **acción directa** (entrada ↑ ⇒ salida ↑) o **inversa** (entrada ↑ ⇒ salida ↓).
- Ejemplo (intercambiador de calor): flujo de refrigerante vs. apertura de válvula → directa; temperatura de salida vs. flujo de refrigerante → inversa.
- **Por qué importa:** el controlador también se configura como directo o inverso para que la realimentación **siempre sea negativa**. Si queda positiva, el sistema se vuelve inestable.

### 2.3 Tipo de un sistema
- **Tipo** = número de **polos en el origen** (integradores puros) de la FT.
- Ejemplo: velocidad de una cinta transportadora vs. voltaje del motor → tipo 0; **posición** de un objeto sobre la cinta → tipo 1 (la posición es la integral de la velocidad).
- Lo que importa es el tipo del **lazo abierto** $C(s)G(s)$: de él depende el error en estado estable, y por lo tanto si hace falta un integrador en el controlador.

### 2.4 Error en estado estable ante cambios en la **referencia**
Con $H = 1$:

$$G_x(s)=\frac{Y}{X}=\frac{CG}{1+CG},\qquad E = X - Y = \frac{1}{1+CG}\,X$$

$$e_{ss}=\lim_{s\to 0} sE(s)=\lim_{s\to 0}\frac{sX(s)}{1+C(s)G(s)}$$

| Tipo de $CG$ | Escalón $X=1/s$ | Rampa $X=1/s^2$ |
|---|---|---|
| 0 | $e_{ss}=\dfrac{1}{1+K_p}$ (constante), con $K_p=\lim_{s\to0} CG = b_0/a_0$ | $\infty$ |
| 1 | $0$ | $\dfrac{1}{K_v}$ (constante), $K_v=\lim_{s\to0} s\,CG$ |
| 2 | $0$ | $0$ |

**Idea clave:** frente a la referencia, **no importa si el integrador está en C o en G**: basta con que esté en la trayectoria directa.

### 2.5 Error en estado estable ante **perturbaciones**
Con la perturbación entrando antes de la planta ($D$ en la entrada de $G$) y referencia = 0:

$$G_d(s)=\frac{Y}{D}=\frac{G}{1+CG},\qquad e_{ss}=-\lim_{s\to0} s\,\frac{G}{1+CG}\,D(s)$$

Para $D=1/s$: $\;e_{ss}=-\lim_{s\to0}\dfrac{G}{1+CG}$.

- **Integradores en G no garantizan error cero** ante perturbación. Si $G\to\infty$ (G tipo 1) y C es tipo 0 con ganancia $K_c$: $e_{ss}\to -1/K_c \neq 0$.
- **Un integrador en C sí garantiza error cero** ante perturbación escalón (si $C\to\infty$, entonces $e_{ss}\to 0$).

> **Diferencia clave (pregunta típica de certamen):** para la referencia sirve un integrador en *cualquier* parte del lazo directo. Para la perturbación el integrador tiene que estar **en el controlador**, es decir, *antes* del punto donde entra la perturbación.

Resumen de la diapositiva 43 (ref = 0, perturbación escalón):

| C | G | Error ante perturbación escalón |
|---|---|---|
| tipo 0 | tipo 0 | constante ≠ 0 |
| tipo 0 | tipo 1 | constante ≠ 0 |
| tipo 1 | tipo 0 | **0** |

---

## 3. Conf. 1 — El controlador PID

### 3.1 Las tres acciones

| Acción | Ley | FT | Ventaja | Desventaja |
|---|---|---|---|---|
| **P** | $u=K_p e$ | $K_p$ | Rápida y fácil de ajustar | No garantiza $e_{ss}=0$ en plantas tipo 0 (si $e=0$, $u=0$) |
| **I** | $u=K_i\int e\,dt$ | $K_i/s$ | $e_{ss}=0$ ante escalón: la integral "recuerda" un valor y mantiene la acción aunque $e=0$ (sube el tipo del lazo en 1) | Lenta (un escalón en $e$ da una rampa en $u$). Junto con la inercia de la planta puede volver el sistema **oscilatorio** |
| **D** | $u=K_d\,\dfrac{de}{dt}$ | $K_d s$ | Se opone a los cambios: **amortigua** oscilaciones, incluidas las que introduce la acción I | **Amplifica el ruido** (alta frecuencia ⇒ derivada grande) |

### 3.2 Formas del PID (¡muy importante para la sintonía!)

**PID paralelo:** $\;K_p + \dfrac{K_i}{s} + K_d s$

**PID ideal (ISA / estándar):**
$$\frac{U}{E}=K_c\left(1+\frac{1}{T_i s}+T_d s\right)$$

**PID serie (interactivo, "real"):**
$$\frac{U}{E}=K'_c\left(1+\frac{1}{T'_i s}\right)\frac{T'_d s+1}{\alpha T'_d s+1},\qquad \alpha\in[0.05,\,0.2]\ \text{(filtro)}$$

**Conversión serie → ideal:**
$$K_c=K'_c\left(1+\frac{T'_d}{T'_i}\right),\qquad T_i=T'_i+T'_d,\qquad T_d=\frac{T'_i\,T'_d}{T'_i+T'_d}$$

> ⚠️ P, PI y PD son **iguales** en ambas formas. Solo el **PID** cambia.
> ⚠️ **Ziegler-Nichols** entrega parámetros para el **PID serie**; **Cohen-Coon y los criterios integrales** los entregan para el **PID ideal**.
> En Simulink: "Ideal" = $P(1+I/s+Ds)$, "Parallel" = $P+I/s+Ds$.

### 3.3 PID práctico (tres mejoras)

1. **Anti-windup.**
   - *Windup:* cuando el actuador se satura, la integral sigue acumulando un valor que el actuador no puede entregar. Cuando el error cambia de signo, el controlador tarda en "desintegrar" ese valor, lo que produce un sobrepaso grande y una respuesta lenta.
   - *Back-calculation:* se resta a la integral la diferencia $(u - u_s)$ entre la acción calculada y la saturada, multiplicada por una ganancia $K_b$:
     - PI: $K_b = 1/T_i$
     - PID: $K_b = 1/\sqrt{T_i T_d}$
   - *Clamping:* se detiene la integración mientras el actuador está saturado.
2. **Filtro en la derivada.** Se implementa como derivada con filtro pasa-bajos de 1er orden:
   $$\frac{u_d}{sp-m}=\frac{D N s}{s+N}=\frac{D s}{\frac{1}{N}s+1},\qquad N = 2\pi F_c \ (\text{frecuencia de corte del filtro})$$
3. **Derivada sobre la medición.** La acción D se calcula solo sobre la medición $m$ y no sobre el error, para evitar el "pulso" (*derivative kick*) que produciría derivar un escalón en la referencia $sp$.

**Conclusión de la Conf. 1:** el comportamiento en lazo cerrado (y en particular $e_{ss}$) depende de si se analiza frente a la **referencia** o frente a la **perturbación**, y del **tipo** de la planta y del controlador. Eso es lo que guía la elección del controlador.

---

## 4. Conf. 2 — Caracterización dinámica (modelo PORT)

Muchos procesos de orden superior (multicapacitivos, sobreamortiguados) se aproximan bien con un modelo de **P**rimer **O**rden con **R**etardo de **T**iempo:

$$\text{PORT: } G(s)=\frac{K e^{-Ls}}{Ts+1}\qquad\qquad \text{SORT: } \frac{K e^{-Ls}}{(\tau_1 s+1)(\tau_2 s+1)}\ \text{ o }\ \frac{K e^{-Ls}}{\tau^2 s^2+2\phi\tau s+1}$$

### Método de los dos puntos (28 % y 63 %), a partir de la respuesta al escalón
1. **Ganancia:** $\displaystyle K=\frac{Y_{ss}-Y_0}{\Delta U}$ (ojo con el signo y las unidades).
2. Medir $T_{28}$ (instante en que la salida alcanza el 28,3 % del cambio total) y $T_{63}$ (63,2 %).
3. Resolver:
$$T_{28}=L+\frac{T}{3},\qquad T_{63}=L+T$$
$$\boxed{T=\frac{3}{2}\,(T_{63}-T_{28}),\qquad L=T_{63}-T}$$

> ⚠️ $T_{28}$ y $T_{63}$ se miden **desde el instante en que se aplica el escalón**, así que **incluyen el retardo**.

---

## 5. Conf. 2 — Sintonía: Ziegler-Nichols y Cohen-Coon

Ambos métodos buscan una respuesta rápida con **razón de decrecimiento de ¼** (cada pico de oscilación es ¼ del anterior).

### 5.1 ZN en lazo cerrado (ganancia crítica). Parámetros para el **PID serie**
Procedimiento: en lazo cerrado, solo con acción P, se sube la ganancia hasta que aparecen **oscilaciones sostenidas**. Ahí se leen la **ganancia crítica** $K_{cu}$ y el **período de oscilación** $T_u$.

| Regulador | $K'_c$ | $T'_i$ | $T'_d$ |
|---|---|---|---|
| P | $K_{cu}/2$ | — | — |
| PI | $K_{cu}/2.2$ | $T_u/1.2$ | — |
| PID | $K_{cu}/1.6$ | $T_u/2$ | $T_u/8$ |

En MATLAB: `[Gm,Pm,Wcg,Wcp] = margin(Gv*Gs*H)`, con $K_u = G_m$ (en veces, **no en dB**) y $T_u = 2\pi/W_{cg}$. Hay que incluir válvula, proceso y sensor.

- Respuesta a la referencia: `step(feedback(Gc*Gv*Gs, H))`
- Respuesta a la perturbación: `step(Gf*feedback(1, H*Gc*Gv*Gs))`

### 5.2 ZN en lazo abierto (a partir del PORT). Parámetros para el **PID serie**. Válido si $0.1 \le L/T \le 0.5$

| Regulador | $K'_c$ | $T'_i$ | $T'_d$ |
|---|---|---|---|
| P | $\dfrac{T}{KL}$ | — | — |
| PI | $0.9\,\dfrac{T}{KL}$ | $3.33\,L$ | — |
| PID | $1.2\,\dfrac{T}{KL}$ | $2L$ | $0.5L$ |

Se usa cuando **no se puede** llevar la planta a oscilación sostenida (por seguridad o por producción).

### 5.3 Cohen-Coon (a partir del PORT). Parámetros para el **PID ideal**
Similar a ZN (¼ RD), pero **menos sensible a la razón $L/T$**. Con $\tau=T$:

| Regulador | $K_c$ | $T_I$ | $T_D$ |
|---|---|---|---|
| P | $\dfrac{\tau}{KL}\left(1+\dfrac{L}{3\tau}\right)$ | — | — |
| PI | $\dfrac{\tau}{KL}\left(0.9+\dfrac{L}{12\tau}\right)$ | $L\,\dfrac{30+3L/\tau}{9+20L/\tau}$ | — |
| PID | $\dfrac{\tau}{KL}\left(\dfrac{4}{3}+\dfrac{L}{4\tau}\right)$ | $L\,\dfrac{32+6L/\tau}{13+8L/\tau}$ | $L\,\dfrac{4}{11+2L/\tau}$ |

**Intuición:** $K_c \propto \dfrac{\tau}{KL}$. A mayor retardo relativo ($L/\tau$) hay que ser **más conservador** (menos ganancia), y a mayor ganancia del proceso $K$, menos ganancia en el controlador.

---

## 6. Conf. 2 — Criterios integrales del error

ZN y CC fijan una cifra puntual (¼ RD). Los criterios integrales miran **toda la respuesta** de $t=0$ a $\infty$ y **minimizan un índice**, con $e(t)=y_{sp}(t)-y(t)$:

$$ISE=\int_0^\infty e^2dt\quad IAE=\int_0^\infty |e|\,dt\quad ITAE=\int_0^\infty t\,|e|\,dt\quad ITSE=\int_0^\infty t\,e^2dt$$

| Criterio | Qué penaliza | Cuándo usarlo |
|---|---|---|
| **IAE** | Todos los errores por igual | Suprimir errores **pequeños** |
| **ISE** | Los errores **grandes** (al cuadrado), es decir, los iniciales | Reducir fuertemente el error inicial (tiende a respuestas oscilatorias) |
| **ITAE / ITSE** | Los errores que **persisten en el tiempo** (multiplica por $t$) | Eliminar errores que duran |

- Cada criterio da un ajuste distinto, y el ajuste óptimo **depende de la entrada** (referencia o perturbación). Por eso siempre hay que especificar para qué entrada se sintoniza.
- Hacerlo analíticamente es muy engorroso (ejemplo de clase: $G_p=20/(1+s)$ con un PI; el lazo cerrado queda de 2º orden con $\tau=\sqrt{T_I/(20K_c)}$, y hay que resolver $\partial ISE/\partial\phi = \partial ISE/\partial\tau=0$). Por eso se usan **correlaciones tabuladas** (Smith, Murrill et al., U. de Louisiana).
- Válidas para entradas escalón, especialmente con $0.1\le L/T\le 1.0$. Parámetros para el **PID ideal**. El $T_I$ depende más de $\tau$ y menos de $L$ que en ZN/CC.

### 6.1 Para cambios en la **perturbación**
$$K_c=\frac{a_1}{K}\left(\frac{L}{\tau}\right)^{b_1}\qquad T_I=\frac{\tau}{a_2}\left(\frac{L}{\tau}\right)^{b_2}\qquad T_D=a_3\,\tau\left(\frac{L}{\tau}\right)^{b_3}$$

| Reg. | Par. | ISE | IAE | ITAE |
|---|---|---|---|---|
| **P** | a | 1.411 | 0.902 | 0.490 |
| | b | −0.917 | −0.985 | −1.084 |
| **PI** | a1 | 1.305 | 0.984 | 0.859 |
| | b1 | −0.954 | −0.986 | −0.977 |
| | a2 | 0.492 | 0.608 | 0.674 |
| | b2 | 0.739 | 0.707 | 0.680 |
| **PID** | a1 | 1.495 | 1.435 | 1.357 |
| | b1 | −0.945 | −0.921 | −0.947 |
| | a2 | 1.101 | 0.878 | 0.842 |
| | b2 | 0.771 | 0.749 | 0.738 |
| | a3 | 0.560 | 0.482 | 0.381 |
| | b3 | 1.006 | 1.137 | 0.995 |

### 6.2 Para cambios en la **referencia** (Rovira)
Se descarta el P (no da $e_{ss}=0$) y el ISE (respuestas muy oscilatorias). **Ojo: cambia la fórmula de $T_I$.**
$$K_c=\frac{a_1}{K}\left(\frac{L}{\tau}\right)^{b_1}\qquad T_I=\frac{\tau}{a_2+b_2\,(L/\tau)}\qquad T_D=a_3\,\tau\left(\frac{L}{\tau}\right)^{b_3}$$

| Reg. | Par. | IAE | ITAE |
|---|---|---|---|
| **PI** | a1 | 0.758 | 0.586 |
| | b1 | −0.861 | −0.916 |
| | a2 | 1.02 | 1.03 |
| | b2 | −0.323 | −0.165 |
| **PID** | a1 | 1.086 | 0.965 |
| | b1 | −0.869 | −0.855 |
| | a2 | 0.740 | 0.796 |
| | b2 | −0.130 | −0.147 |
| | a3 | 0.348 | 0.308 |
| | b3 | 0.914 | 0.929 |

---

## 7. Conf. 2 — Selección del tipo de controlador

- **P** (siempre que se pueda): con una ganancia adecuada da exactitud aceptable y buena respuesta. En procesos **integradores** (nivel, presión) el P ya da error cero ante cambios en la referencia.
- **PI** (si se exige alta exactitud): muy usado en **flujo de líquidos**, donde los sensores son ruidosos y por eso no conviene la D. Rara vez se usa en nivel y presión.
- **PID** (dinámica compleja): grandes retardos, respuesta inversa, procesos lentos multicapacitivos o multivariables interactivos. Típico en **temperatura y composición**.

**Conclusiones de la Conf. 2:** el PORT basta para analizar y diseñar en ingeniería. El criterio ¼ RD es muy común, pero no siempre da el desempeño esperado, y por eso existen los criterios integrales. En todos los casos los valores calculados son una **primera aproximación** que después se afina en planta. La realimentación simple no siempre basta, lo que motiva estrategias más elaboradas.

---

## 8. Conf. 3 — Control de plantas de 1er orden

Planta: $G_p=\dfrac{K}{Ts+1}$. Controlador: PI $\;G_c=K_p\dfrac{T_i s+1}{T_i s}$.

### 8.1 PI por cancelación ($T_i = T$)
El cero del PI cancela el polo de la planta:
$$G_{LC}(s)=\frac{K_pK}{Ts+K_pK}=\frac{1}{\frac{T}{K_pK}s+1}\quad\Rightarrow\quad T_{LC}=\frac{T}{K_pK},\qquad \boxed{K_p=\frac{T}{T_{LC}\,K},\quad T_i=T}$$

- Con el criterio del 2 %: $t_{ss}\approx 4T_{LC}$.
- Para que responda de forma lineal, $K_p$ debe ser lo más grande posible **sin saturar** la acción de control ante el máximo cambio de referencia.
- ❌ **Ante perturbaciones no hay cancelación.** Queda un polo más (el de la planta) y la respuesta es **más lenta**:
$$\frac{Y}{D}=\frac{\frac{T}{K_p}s}{(Ts+1)\left(\frac{T}{K_pK}s+1\right)}$$
- ❌ Es sensible a la incertidumbre en $T$ (una cancelación imperfecta deja un par polo-cero cercano, con una "cola" lenta).

**Ejemplo 1:** $G_p=1/(s+1)$, se quiere $t_{ss}=2$ (en lazo abierto es $4T=4$). Entonces $T_{LC}=2/4=0.5$, $K_p=1/(0.5\cdot1)=2$, $T_i=1$. Ante la referencia: $1/(0.5s+1)$. Ante la perturbación: $\dfrac{0.5s}{(0.5s+1)(s+1)}$ (más lento).

### 8.2 PI con prefiltro (se impone un 2º orden deseado)
No se cancela nada: se eligen $K_p$ y $T_i$ para que el lazo cerrado tenga los $\zeta$ y $\omega_n$ deseados:
$$G_{LC}=\frac{\omega_n^2(T_is+1)}{s^2+2\zeta\omega_n s+\omega_n^2},\qquad 2\zeta\omega_n=\frac{1+K_pK}{T},\quad \omega_n^2=\frac{K_pK}{T_iT}$$

$$\boxed{\omega_n=\frac{4}{\zeta\,t_{ss}},\qquad K_p=\frac{2\zeta\omega_nT-1}{K},\qquad T_i=\frac{K_pK}{T\omega_n^2}}$$

- $\zeta=0.7 \Rightarrow M_p\approx4\%$ y $\zeta=0.4\Rightarrow M_p\approx25\%$.
- Para que $K_p>0$ se necesita $2\zeta\omega_nT>1$, es decir, $t_{ss}<8T$.
- El **cero** $(T_is+1)$ acelera la respuesta y mejora el rechazo a perturbaciones, pero **aumenta el sobrepaso** por encima del que da $\zeta$. Se compensa con un **prefiltro** $\dfrac{1}{T_is+1}$ en la referencia. Como $T_i$ se conoce exactamente, esa cancelación **sí es exacta**.
  - $5T < t_{ss} < 8T$: **mejor sin prefiltro** (el cero queda lejos de los polos y afecta poco).
  - $t_{ss} < 5T$: **usar el prefiltro**. Ojo: a menor $t_{ss}$, mayor acción de control y riesgo de saturación.
- ✅ Como no se basa en cancelación, la **ecuación característica es la misma** ante la referencia y ante la perturbación: responde **igual de rápido** en ambos casos y es menos sensible a la incertidumbre.

**Ejemplo 2:** $G_p=1/(s+1)$, $t_{ss}=2$, $M_p=4\%$. Entonces $\zeta=0.7$, $\omega_n=4/(0.7\cdot2)=2.86$, $K_p=2\cdot0.7\cdot2.86\cdot1-1=3$, $T_i=3/2.86^2=0.36$.
Ante la referencia: $\dfrac{8.16}{s^2+4s+8.16}$ (con prefiltro). Ante la perturbación: $\dfrac{s}{s^2+4s+8.16}$ (mismos polos, igual de rápida).

---

## 9. Conf. 3 — Módulo óptimo

**Objetivos:**
1. Compensar (cancelar) las **constantes de tiempo más grandes** (lentas) para lograr una respuesta rápida.
2. Poner un **integrador** en la trayectoria directa para que el lazo sea tipo 1 ($e_{ss}=0$ ante escalón en la referencia).
3. Que el lazo cerrado sea un **2º orden típico con $\zeta=0.707$**: lo más rápido posible con sobrepaso mínimo (≈ 4 %), que es óptimo según ITAE.

**Lazo directo deseado** (con $K_r$ = ganancia de la realimentación/sensor y $T_u$ = **la menor** constante de tiempo, la única que **no** se compensa):
$$G_cG_p=\frac{1}{K_r\,a\,T_u s\,(T_u s+1)}$$

En lazo cerrado:
$$G_{LC}=\frac{\frac{1}{K_r a T_u^2}}{s^2+\frac{1}{T_u}s+\frac{1}{aT_u^2}}\ \Rightarrow\ \omega_n=\frac{1}{T_u\sqrt a},\quad \zeta=\frac{\sqrt a}{2}$$
Con **$a=2$** se obtiene $\zeta=\sqrt2/2=0.707$. Despejando:
$$\boxed{G_c(s)=\frac{1}{K_r\,2T_u s\,(T_us+1)\,G_p(s)}}$$

**Ejemplo 3:** $G_p=\dfrac{K_1K_2}{(T_1s+1)(T_2s+1)}$ con $T_1<T_2$, así que $T_u=T_1$:
$$G_c=\frac{T_2s+1}{2T_1K_rK_1K_2\,s}=\frac{T_2s+1}{Ts}=P+\frac{I}{s},\quad T=2T_1K_rK_1K_2,\ P=\frac{T_2}{T},\ I=\frac1T$$
Es un **PI cuyo cero cancela la constante de tiempo mayor**. Con $K_1=2,T_1=1,K_2=3,T_2=2,K_r=1$: $T=12$, $P=0.167$, $I=0.083$.
Como en toda cancelación, la respuesta ante la perturbación resulta **más lenta**.

**Regla práctica:** si la planta tiene 1 constante lenta, el MO da un **PI**; si tiene 2 constantes lentas, un **PID** (dos ceros); si la planta ya tiene integrador, un **P** (o PD).

> ⚠️ Si el polo en el origen lo pone **la planta**, el controlador no lo tendrá, y entonces **habrá error ante perturbaciones** (ver sección 2.5).

---

## 10. Conf. 3 — Módulo simétrico

**Objetivos:** compensar las constantes lentas (igual que en MO), alcanzar el estado estacionario rápido y tener buena estabilidad relativa. Se diseña en **respuesta en frecuencia** (Bode).

**Lazo directo deseado:**
$$G_cG_p=\frac{4T_us+1}{K_r\,8T_u^2\,s^2\,(T_us+1)}
\qquad\Rightarrow\qquad
\boxed{G_c=\frac{4T_us+1}{K_r\,8T_u^2 s^2(T_us+1)\,G_p(s)}}$$

Lectura del Bode:
- **Dos polos en el origen**: pendiente de −40 dB/dec en bajas frecuencias, lo que da ganancia alta y lleva rápido al estado estacionario. Lazo **tipo 2**.
- El **cero en $1/(4T_u)$** sube la pendiente a −20 dB/dec, y el **polo en $1/T_u$** la devuelve a −40 dB/dec.
- La ganancia se elige para que el **cruce por 0 dB ocurra a −20 dB/dec** (en $\omega_c=1/(2T_u)$, justo entre el cero y el polo; de ahí lo "simétrico"), lo que da buena estabilidad relativa.
- *Dato complementario (teoría estándar, no está en las diapositivas):* margen de fase ≈ 37° y sobrepaso ≈ 43 % ante escalón en la referencia sin prefiltro. Por eso suele usarse con un prefiltro $1/(4T_us+1)$.

**Ventaja frente al MO:** $e_{ss}=0$ ante **escalón y rampa** en la referencia **y escalón en la perturbación**, salvo que sea la planta la que aporta los dos integradores.

**Ejemplo 4:** $G_p=\dfrac{100}{s(s+10)}$. Primero hay que llevarla a la **forma típica** (constantes de tiempo): $G_p=\dfrac{10}{s(0.1s+1)}$, así que $T_u=0.1$.
- **MS:** $G_c=\dfrac{0.4s+1}{0.08s^2(0.1s+1)}\cdot\dfrac{s(0.1s+1)}{10}=\dfrac{0.4s+1}{0.8s}=0.5+\dfrac{1.25}{s}$ → **PI** ($K_p=0.5$, $K_i=1.25$).
- **MO:** $G_c=\dfrac{1}{0.2s(0.1s+1)}\cdot\dfrac{s(0.1s+1)}{10}=0.5$ → **P** ($K_p=0.5$). No tiene integrador, así que queda con error ante la perturbación. El MS sí la rechaza.

---

## 11. Conf. 3 — Control en cascada

**Motivación:** con MO/MS, cuanto más compleja es la planta, más complejo es el regulador (PID, PIDD…). La cascada (**mando subordinado**) usa **más sensores y lazos, pero reguladores más simples** (típicamente PI).

**Estructura:** planta $\dfrac{K_1}{T_1s+1}\cdot\dfrac{K_2}{T_2s+1}\cdot\dfrac{K_3}{T_3s+1}$ con $T_1<T_2<T_3$. Se mide una variable **intermedia** (la salida de $K_2/(T_2s+1)$) para cerrar un **lazo interno**, y el **lazo externo** controla la salida final y entrega la referencia del interno.

**Síntesis (de adentro hacia afuera):**
1. **Lazo interno** por MO, con $T_1$ como constante no compensable:
   $$G_{reg1}=\frac{T_2s+1}{T's},\qquad T'=2T_1K_{real1}K_1K_2$$
   Queda $G_{LC1}=\dfrac{1/K_{real1}}{2T_1^2s^2+2T_1s+1}\approx\dfrac{1/K_{real1}}{2T_1s+1}$ (el término $2T_1^2$ se desprecia si $T_1$ es pequeño).
2. **Lazo externo** por MO, visto como planta $\dfrac{1/K_{real1}}{2T_1s+1}\cdot\dfrac{K_3}{T_3s+1}$ y con **$2T_1$** como nueva constante no compensable:
   $$G_{reg}=\frac{T_3s+1}{T''s},\qquad T''=\frac{4K_3K_{real}T_1}{K_{real1}}$$

**Ejemplo 5:** $G_p=\dfrac{1}{0.1s+1}\cdot\dfrac{2}{s+1}\cdot\dfrac{3}{10s+1}$, con $K_{real}=K_{real1}=1$.
- **Un solo lazo por MO** ($T_u=0.1$): $G_c=\dfrac{(s+1)(10s+1)}{1.2s}=8.333\,\dfrac{(s+1)(s+0.1)}{s}$ → **PID**.
- **Cascada:**
  - Interno: $G_{c,in}=\dfrac{s+1}{0.4s}=2.5\,\dfrac{s+1}{s}$ (PI). Su lazo cerrado: $\dfrac{50}{s^2+10s+50}=\dfrac{1}{0.02s^2+0.2s+1}\approx\dfrac{1}{0.2s+1}$.
  - Externo ($T_u=0.2$): $G_{c,out}=\dfrac{10s+1}{1.2s}=0.8333\,\dfrac{10s+1}{s}$ (PI).
- **Resultado:** dos PI (más fáciles de implementar) en vez de un PID, y las **perturbaciones internas las corrige el lazo interno** casi sin que el externo tenga que intervenir.

**Condición práctica:** el lazo interno debe ser **bastante más rápido** que el externo.

**Conclusiones de la Conf. 3:**
- PI por cancelación: sencillo, pero lento ante perturbaciones y sensible a la incertidumbre de los parámetros.
- PI con 2º orden impuesto: más rápido ante perturbaciones y más robusto.
- MO: compensa la dinámica lenta, $e_{ss}=0$ ante escalón en la referencia, $\zeta=0.707$.
- MS: además $e_{ss}=0$ ante rampa y ante perturbación escalón; buena estabilidad relativa.
- Cascada: reguladores más simples y mejor rechazo de perturbaciones.
- Siempre hay que vigilar la **saturación** de la acción de control.
- ⚠️ Estos métodos **no sirven solos para plantas con retardo**, porque requieren el **inverso** de $G_p$ y $e^{+Ls}$ no es realizable. Esto motiva el Tema 2 (predictor de Smith).

---

## 12. Ejemplo integrador resuelto

*(Ejercicio de práctica propio, no está en las diapositivas.)* Se aplica un escalón $\Delta U=2$ y la salida pasa de 0 a 10. Se mide $T_{28}=3.2$ s y $T_{63}=6.5$ s.

1. **Modelo PORT:** $K=10/2=5$. $T=1.5(6.5-3.2)=4.95$ s. $L=6.5-4.95=1.55$ s. $L/T=0.313$ (dentro del rango de ZN).
2. **ZN lazo abierto, PID serie:** $K'_c=1.2\cdot\dfrac{4.95}{5\cdot1.55}=0.766$, $T'_i=2L=3.10$, $T'_d=0.5L=0.775$.
   Pasado a **ideal**: $K_c=0.766(1+0.775/3.1)=0.958$, $T_i=3.875$, $T_d=3.1\cdot0.775/3.875=0.62$.
3. **Cohen-Coon, PID ideal:** $K_c=\dfrac{4.95}{7.75}\left(\dfrac43+\dfrac{0.313}{4}\right)=0.90$, $T_I=1.55\cdot\dfrac{32+6(0.313)}{13+8(0.313)}=3.39$, $T_D=\dfrac{4\cdot1.55}{11+2(0.313)}=0.53$.
4. **ITAE para la perturbación, PID ideal:** $K_c=\dfrac{1.357}{5}(0.313)^{-0.947}=0.81$, $T_I=\dfrac{4.95}{0.842}(0.313)^{0.738}=2.50$, $T_D=0.381\cdot4.95\cdot(0.313)^{0.995}=0.59$.
5. **ITAE para la referencia, PID ideal:** $K_c=\dfrac{0.965}{5}(0.313)^{-0.855}=0.52$, $T_I=\dfrac{4.95}{0.796-0.147(0.313)}=6.60$, $T_D=0.308\cdot4.95\cdot(0.313)^{0.929}=0.52$.

**Observación:** la sintonía para la referencia es más conservadora (menos $K_c$, más $T_I$) que la sintonía para la perturbación.

---

## 13. Trampas típicas y checklist para el certamen

- [ ] ¿La tabla da el **PID serie** (ZN) o el **ideal** (CC, integrales)? Si piden comparar, **convertir** con las fórmulas de la sección 3.2.
- [ ] $T_{28}$ y $T_{63}$ se miden **desde el escalón** (incluyen $L$).
- [ ] $K$ del proceso **con signo**: el controlador debe ser directo o inverso para que la realimentación sea negativa.
- [ ] $K_u$ va en **veces**, no en dB ($K_u=10^{G_m[dB]/20}$). Y $T_u=2\pi/\omega_u$.
- [ ] Verificar el rango de validez: ZN $0.1\le L/T\le0.5$; integrales $0.1\le L/T\le1$.
- [ ] En los criterios integrales, **para la referencia** $T_I=\tau/(a_2+b_2L/\tau)$; **para la perturbación** $T_I=(\tau/a_2)(L/\tau)^{b_2}$.
- [ ] Error ante la **perturbación**: el integrador debe estar en **C**, no en G.
- [ ] En MO/MS, llevar primero la planta a la **forma de constantes de tiempo** $\dfrac{K}{Ts+1}$ antes de identificar $T_u$ (Ejemplo 4).
- [ ] $T_u$ = la **menor** constante de tiempo (no se compensa); las demás las cancela el controlador.
- [ ] Cancelación ⇒ la respuesta a la perturbación es **más lenta** que a la referencia (aparece el polo de la planta).
- [ ] PI con prefiltro: $t_{ss}<8T$ para que $K_p>0$; prefiltro si $t_{ss}<5T$.
- [ ] Cascada: sintetizar **primero el lazo interno** y aproximarlo a 1er orden con $2T_1$.
- [ ] Siempre comentar la **saturación** del actuador y el **anti-windup**.
- [ ] MO/MS/cancelación **no** sirven para plantas con retardo.

**Preguntas conceptuales probables:**
1. ¿Por qué la acción I elimina el error ante escalón, y por qué puede volver oscilatorio al sistema?
2. ¿Por qué la acción D se aplica sobre la medición y con filtro?
3. ¿Qué es el windup y cómo funciona el back-calculation?
4. Comparar ZN, CC y los criterios integrales (objetivo, forma de PID, sensibilidad a $L/T$).
5. ¿Cuándo usar ISE, IAE o ITAE?
6. ¿Por qué el PI por cancelación es lento ante perturbaciones y el PI con 2º orden impuesto no?
7. Diferencias MO vs. MS (tipo del lazo, errores que anulan, $\zeta$ vs. margen de fase).
8. Ventajas del control en cascada.

---

## 14. Formulario resumen

| Concepto | Fórmula |
|---|---|
| Error, referencia | $e_{ss}=\lim_{s\to0}\dfrac{sX}{1+CG}$ |
| Error, perturbación | $e_{ss}=-\lim_{s\to0}\dfrac{sG\,D}{1+CG}$ |
| PID ideal | $K_c(1+\frac{1}{T_is}+T_ds)$ |
| Serie → ideal | $K_c=K'_c(1+\frac{T'_d}{T'_i})$, $T_i=T'_i+T'_d$, $T_d=\frac{T'_iT'_d}{T'_i+T'_d}$ |
| Anti-windup $K_b$ | PI: $1/T_i$ · PID: $1/\sqrt{T_iT_d}$ |
| PORT | $K=\Delta Y/\Delta U$, $T=1.5(T_{63}-T_{28})$, $L=T_{63}-T$ |
| ZN LC (serie) | P: $K_{cu}/2$ · PI: $K_{cu}/2.2,\ T_u/1.2$ · PID: $K_{cu}/1.6,\ T_u/2,\ T_u/8$ |
| ZN LA (serie) | P: $\frac{T}{KL}$ · PI: $0.9\frac{T}{KL},\ 3.33L$ · PID: $1.2\frac{T}{KL},\ 2L,\ 0.5L$ |
| PI cancelación | $T_i=T$, $K_p=T/(T_{LC}K)$, $t_{ss}=4T_{LC}$ |
| PI 2º orden | $\omega_n=\frac{4}{\zeta t_{ss}}$, $K_p=\frac{2\zeta\omega_nT-1}{K}$, $T_i=\frac{K_pK}{T\omega_n^2}$ |
| Módulo óptimo | $G_c=\dfrac{1}{K_r\,2T_us(T_us+1)G_p}$, $\zeta=0.707$ |
| Módulo simétrico | $G_c=\dfrac{4T_us+1}{K_r\,8T_u^2s^2(T_us+1)G_p}$ |
| Cascada | Interno: $T'=2T_1K_{r1}K_1K_2$; externo: $T''=4K_3K_rT_1/K_{r1}$ |
| $\zeta$ ↔ $M_p$ | $0.7 \to 4\%$ · $0.4\to25\%$ · $t_{ss}\approx 4/(\zeta\omega_n)$ |
