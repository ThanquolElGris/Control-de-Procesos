# Control de Procesos — Guía de estudio (Conferencias 4, 5 y 6)

> Ramo: Control de Procesos · Dr. Angel E. Rubio R. · DIEE-UBB
> Cubre las conferencias 4 (Estrategias de control realimentado), 5 (Control por adelanto: feedforward) y 6 (Compensadores de retardo y respuesta inversa, control inferencial).
> Las Conf. 4 y 5 cierran el **Tema 1: Control SISO** (cascada, selectivo, rango dividido, relación, feedforward). La Conf. 6 abre el **Tema 2: Compensadores**, que se evalúa en el **Certamen 2 (C2)**.
> Continúa la guía [`Resumen_Certamen1_Conf1-3.md`](Resumen_Certamen1_Conf1-3.md); las referencias a "sección 2.6", "MO" o "MS" apuntan a esa guía.

---

## Índice

1. [Mapa de las conferencias 4–6](#1-mapa-de-las-conferencias-46)
2. [Conf. 4 — Control en cascada (cont.)](#2-conf-4--control-en-cascada-cont)
3. [Conf. 4 — Control selectivo y por sobrecorrida](#3-conf-4--control-selectivo-y-por-sobrecorrida)
4. [Conf. 4 — Control de rango dividido](#4-conf-4--control-de-rango-dividido)
5. [Conf. 4 — Control de relación](#5-conf-4--control-de-relación)
6. [Conf. 5 — Control feedforward](#6-conf-5--control-feedforward)
7. [Conf. 5 — Realizabilidad del feedforward](#7-conf-5--realizabilidad-del-feedforward)
8. [Conf. 5 — Feedforward vs. feedback y esquema combinado](#8-conf-5--feedforward-vs-feedback-y-esquema-combinado)
9. [Conf. 6 — Predictor de Smith](#9-conf-6--predictor-de-smith)
10. [Conf. 6 — Compensador de respuesta inversa](#10-conf-6--compensador-de-respuesta-inversa)
11. [Conf. 6 — Control inferencial](#11-conf-6--control-inferencial)
12. [Trampas típicas y checklist](#12-trampas-típicas-y-checklist)
13. [Formulario resumen](#13-formulario-resumen)

---

## 1. Mapa de las conferencias 4–6

| Conf. | Estrategia | Qué resuelve | Idea en una línea |
|---|---|---|---|
| 4 | **Cascada** | Perturbaciones que entran en una parte rápida de la planta | Un lazo interno rápido corrige la perturbación antes de que llegue a la salida |
| 4 | **Sobrecorrida** (*override*) | Proteger la planta: respetar restricciones en otras variables | Varios controladores, **un actuador**; un selector elige cuál manda |
| 4 | **Selectivo** | La variable crítica puede estar en distintos puntos | Varias **mediciones**, un controlador; el selector elige qué medición usar |
| 4 | **Rango dividido** | Una variable controlada, varias manipulables con prioridad | Un controlador, **varios actuadores** repartidos por tramos de la señal |
| 4 | **Relación** | Mezclar flujos en proporción fija | $F_B = R\cdot F_A$ |
| 5 | **Feedforward** | Perturbaciones grandes, **medibles y modelables** | Medir la perturbación y actuar **antes** de que afecte la salida |
| 6 | **Predictor de Smith** | Retardos grandes ($L > 0.5T$) | Realimentar una predicción de la salida **sin retardo** |
| 6 | **Compensador de resp. inversa** | Plantas de fase no mínima (cero en el SPD) | Mismo truco que Smith, pero para "borrar" el cero positivo |
| 6 | **Control inferencial** | La variable a controlar no se puede medir | Estimarla a partir de otra que sí se mide |

**Hilo conductor:** la realimentación simple (Conf. 1–3) solo reacciona cuando el error ya apareció en la salida y se degrada con retardos o respuesta inversa. Estas conferencias agregan **mediciones, lazos, selectores y modelos** para hacerlo mejor.

---

## 2. Conf. 4 — Control en cascada (cont.)

Se mide, manipula y controla **más de una señal**, así que hay más de un regulador (repaso de la teoría en la sección 11 de la guía Conf. 1–3).

### 2.1 Ejemplo industrial: reactor con horno precalentador (Corripio, pág. 443)
1. **Feedback clásico:** el TC del reactor manipula directamente el combustible del horno. Cualquier variación en el horno o en el combustible tarda en verse en el reactor.
2. **+ lazo interno de temperatura del horno:** el TC del reactor (maestro) da la consigna al TC de salida del horno (esclavo), que corrige las variaciones del horno.
3. **+ lazo más interno de flujo de combustible:** un FC corrige las variaciones de flujo causadas por la **presión del combustible**. Queda una cascada de **tres** niveles: TC reactor → TC horno → FC combustible.

### 2.2 RFB (*reset feedback*): anti-windup en la cascada
Un PI se puede implementar como una **realimentación positiva a través de un retardo de 1er orden**:

$$\frac{K_c}{1-\dfrac{1}{t_is+1}}=K_c\,\frac{t_is+1}{t_is}=K_c\left(1+\frac{1}{t_is}\right)$$

Es decir: la salida del controlador vuelve a su entrada pasando por $\dfrac{1}{t_is+1}$, y eso reproduce exactamente la acción integral. La idea del **RFB** es que, en vez de realimentar **su propia salida**, el controlador maestro realimenta una **señal externa**: la medición del lazo interno ($c_2$).

- Si el lazo interno se satura o **se abre** (pasa a manual), el maestro **no integra** sin límite (no hay windup): queda listo para retomar el control y puede seguir en automático.
- El controlador **interno debe tener acción integral**, para que su medición $c_2$ llegue a igualar su consigna $r_2$ (salida del maestro) sin error. Si no, el RFB "vería" un error que no existe.
- En el lazo de RFB se puede agregar un **retardo de tiempo** que modele el de la planta (al estilo del predictor de Smith): permite reducir la acción integral y mejora el desempeño.

### 2.3 Diseño por ZN de la cascada (código de la clase)
Planta: $G_v=\dfrac{3}{0.2s+1}$, $G_h=\dfrac{1}{(3s+1)(s+1)}$ (horno), $G_r=\dfrac{0.8}{(4s+1)(s+1)}$ (reactor), perturbación $G_a=\dfrac{1}{2s+1}$, sensores $H=0.5$.

```matlab
% Feedback clásico: un PI por ZN sobre toda la planta
[Gm,Pm,Wgm,Wpm] = margin(Gv*Gh*Gr*H);
Ku = Gm;  Tu = 2*pi/Wgm;
Kc = Ku/2.2;  Ti = Tu/1.2;
Gc = tf(Kc*[Ti 1],[Ti 0]);

% Cascada: 1) lazo interno, un P por ZN
[Gm,Pm,Wgm,Wpm] = margin(Gv*Gh*H);
Ku = Gm;  Tu = 2*pi/Wgm;
Gc2 = Ku/2;
% 2) lazo externo, PI por ZN sobre (lazo interno cerrado)*Gr
Gcl2 = feedback(Gc2*Gv*Gh, H);
[Gm,Pm,Wgm,Wpm] = margin(Gcl2*Gr*H);
Ku = Gm;  Tu = 2*pi/Wgm;
Kc = Ku/2.2;  Ti = Tu/1.2;
Gc1 = tf(Kc*[Ti 1],[Ti 0]);
```

Valores que resultan (calculados con los datos de arriba):

| | $K_u$ | $T_u$ | Controlador |
|---|---|---|---|
| Lazo único | 4.33 | 12.4 | PI: $K_c=1.97$, $T_i=10.3$ |
| Cascada, interno | 17.1 | — | P: $K_c=8.53$ |
| Cascada, externo | 7.17 | 4.07 | PI: $K_c=3.26$, $T_i=3.39$ |

Fíjate que, con el lazo interno cerrado, el externo admite **más ganancia** y un $T_i$ **tres veces menor**: el lazo interno "acelera" la planta que ve el externo.

### 2.4 Conclusiones sobre la cascada
- Se sintoniza **primero el lazo interno**, que tiene que quedar **más rápido** que el externo.
- El lazo interno compensa las perturbaciones **antes de que lleguen** al externo, lo que acelera el rechazo.
- La realimentación **reduce la constante de tiempo** del lazo interno. Por eso aumenta el **margen de ganancia** del externo, se puede subir la ganancia del regulador y mejora la exactitud.

---

## 3. Conf. 4 — Control selectivo y por sobrecorrida

El "control selectivo" (en sentido amplio) tiene dos variantes. Ambas usan **selectores** (bloques de mínimo LS / máximo HS):

| | Sobrecorrida (*override*) | Selectivo (*auctioneering*) |
|---|---|---|
| Entradas al selector | Salidas de **varios controladores** | **Varias mediciones** |
| Salidas | **Un** actuador | **Un** controlador |
| Objetivo | Controlar una variable respetando **restricciones** en otras | Controlar el valor **crítico** (p. ej. el máximo) entre varias mediciones |

### 3.1 Sobrecorrida o sobrepaso
- Controla una variable teniendo en cuenta restricciones en otras: **selecciona entre varias acciones de control** para mover **una sola variable manipulada**.
- Protege la planta, al personal y la productividad, manteniendo las variables dentro de límites seguros.
- Ante una falla **no detiene drásticamente** el proceso: hace una **transición suave** entre controladores para seguir obteniendo el máximo beneficio posible.

**Ejemplo: flujo que sale de un tanque con bomba.**
1. Un **FC** controla el flujo de salida $F$ con la válvula $V$ (si $F$ baja, el FC abre $V$).
2. Problema: si el nivel cae por debajo de $h_2$, la **bomba cavita** y se daña.
3. Se agrega un **LC** de nivel (consigna entre $h_1$ y $h_2$): si el nivel baja, el LC **cierra** $V$.
4. Un **selector de mínimo (LS)** entrega a la válvula **la menor** de las dos señales. En operación normal manda el FC; si el nivel es bajo, la señal del LC es menor y **toma el mando**.
5. Se usa **RFB** en ambos controladores, con la **salida del selector** como señal de reset: el controlador que no está mandando no hace windup y puede retomar el control sin saltos.

Otro ejemplo de la clase: control de **presión de vapor** en una caldera **sin permitir nivel bajo** en el domo.

### 3.2 Selectivo (varias mediciones)
Se usa cuando varias mediciones deben influir sobre un mismo controlador.

**Ejemplo: reactor tubular.** El punto caliente se desplaza a lo largo del reactor. Con **un solo sensor** el punto caliente puede quedar donde no se mide y ocurrir reacciones indeseadas. Con **varios sensores** y un **selector de máximo (HS)**, el TC controla siempre la **temperatura más alta**.

---

## 4. Conf. 4 — Control de rango dividido

Permite actuar sobre **varias variables manipuladas** a partir de **una medición** (un controlador). Se usa cuando:
- hay **una variable a controlar y varias manipulables**,
- todas las manipulables afectan a la controlada,
- existe un **orden de prioridad** en su uso.

La salida del controlador (0–100 %) se **reparte en tramos**: cada válvula trabaja solo en una parte del rango.

### 4.1 Ejemplo: presión en un reactor (válvula $V_1$ a la entrada, $V_2$ a la salida)

```
 Posición
   1 ┤- - - - - - - -●───────────── V2 (salida): cerrada hasta 0.2, abre de 0.2 a 0.6
     │            ╱   ╲
     │          ╱       ╲  V1 (entrada): abierta hasta 0.6, cierra de 0.6 a 1.0
   0 ┼────────●───────────●──► acción de control
     0       0.2   0.6   1.0
```

- **Acción < 0.6** (la presión bajó): $V_1$ queda **totalmente abierta** y $V_2$ se va **cerrando**: sale menos producto y la presión **sube**.
- **Acción > 0.6** (la pregunta "???" de la diapositiva): $V_2$ queda **totalmente abierta** y $V_1$ se va **cerrando**: entra menos reactivo y la presión **baja**.
- Prioridad: primero se usa la válvula de salida; solo si no basta se recorta la alimentación.

### 4.2 Otros ejemplos
- **Presión en el tope de una columna de destilación:** el PC primero abre el **agua de refrigeración** del condensador (0–50 % de la señal). Si el condensador es **insuficiente**, en el tramo 50–100 % abre la válvula a la **antorcha** (venteo).
- **Control de pH:** una válvula de **ácido** y otra de **base**. Con la salida del controlador entre 0 y 60 % se dosifica ácido (máximo en 0 %), y entre 60 y 100 % se dosifica base (máximo en 100 %).

---

## 5. Conf. 4 — Control de relación

Se requiere mezclar dos componentes en proporción fija:

$$F_B=R\cdot F_A$$

- Si se pueden manipular **ambos** flujos, basta con dos FC con consignas ajustadas a la relación.
- Si $F_A$ es **libre** (no se puede actuar sobre A, es una perturbación medida), hay dos formas:

| Forma | Cómo funciona | Sensibilidad | Veredicto |
|---|---|---|---|
| **1. Controlador de relación (RC)** | Se calcula la razón medida $F_B/F_A$ y se compara con la deseada $R$ | $\dfrac{\partial (F_B/F_A)}{\partial F_A}=-\dfrac{F_B}{F_A^2}$ → **no lineal** (la ganancia cambia con el punto de operación) | Evitar |
| **2. Consigna calculada** | Se calcula $F_{B,sp}=R\cdot F_A$ y se usa como consigna de un **FC** de B | $\dfrac{\partial F_B}{\partial F_A}=R$ → **lineal** | **Recomendada** |

- Si se controlan **los dos** flujos (por ejemplo, la producción total la fija un FC de A), se combina: un FC para A y, para B, la consigna $R\cdot F_A$ con $F_A$ **medido** (no la consigna de A), para que B siga lo que realmente pasa por A.
- Nótese que la forma 2 es en realidad un **feedforward**: $F_A$ es una perturbación medida y se actúa sobre B antes de que se altere la composición (conecta con la Conf. 5).

**Conclusión de la Conf. 4:** hay dinámicas y exigencias que superan al lazo simple. Estas estrategias pueden costar más (sensores, controladores), pero cuando está en juego la **operación segura** y la protección de la planta y el personal, no hay alternativa.

---

## 6. Conf. 5 — Control feedforward

**Contexto.** Todas las estrategias anteriores son *feedback*: actúan **solo cuando el error ya apareció** en la salida. El feedback, por eso, **no puede lograr el control perfecto** (mantener la consigna sin importar las perturbaciones).

El **feedforward (CFF)** **mide la perturbación** y actúa para compensar su efecto **antes** de que se vea en la salida. En teoría puede lograr el **control perfecto**.

### 6.1 Idea estática
*Ejemplo de la clase:* un cambio unitario en $d$ sube $c$ en 10 unidades, y un cambio unitario en $m$ sube $c$ en 5. Para que $c$ no cambie:

$$5\,m_{FF}+10\,d=0\ \Rightarrow\ m_{FF}=-2\,d$$

Esto compensa solo el **estado estable**. Si el efecto de la perturbación no es instantáneo, el CFF puede **mover la salida por su cuenta** (antes o después que la perturbación). Hay que **retardar** la acción del CFF o incluir la **dinámica** de la perturbación.

### 6.2 Procedimiento generalizado

```
              D ──┬────────► Gd ──┐
                  │               ▼
                  └► Gm ─► Gff ─►(+)─► M ─► Gp ─►(+)──► Y
```

Con el modelo $Y=G_pM+G_dD$ y $M=G_mG_{ff}D$ (la perturbación se **mide** con el sensor $G_m$):

$$Y=\big(G_mG_{ff}G_p+G_d\big)D=0\quad\Rightarrow\quad\boxed{G_{ff}(s)=-\frac{G_d(s)}{G_m(s)\,G_p(s)}}$$

(Sin sensor, $G_m=1$: $G_{ff}=-G_d/G_p$.) Si hay válvula $G_v$, va dentro de $G_p$.

### 6.3 Ejemplo: intercambiador de calor (CFF estático)
Datos: $F_1=F_2=F$ constante; $T_1$ = perturbación medible; $T_2$ = variable controlada; $Q$ = manipulable.

Balance de energía:

$$\rho FC_pT_1(t)+Q(t)-\rho FC_pT_2(t)=\frac{d\big(\rho AhC_pT_2(t)\big)}{dt}$$

En Laplace (variables de desviación) y despejando:

$$T_2(s)=\frac{F}{Ahs+F}\,T_1(s)+\frac{1/(\rho C_p)}{Ahs+F}\,Q(s)$$

Para que $T_2$ no cambie cuando cambia $T_1$ ($T_2(s)=0$):

$$Q(s)=-F\rho C_p\,T_1(s)\quad\Rightarrow\quad G_{ff}=-F\rho C_p$$

Las dos FT tienen el **mismo denominador** ($Ahs+F$), por eso se cancela y el CFF queda **estático** (una ganancia).

---

## 7. Conf. 5 — Realizabilidad del feedforward

$$G_{ff}(s)=-\frac{G_d(s)}{G_m(s)G_p(s)}$$

En general **no es un PID**: es un filtro (de adelanto-atraso o de orden superior). Hay que revisar si se puede implementar:

1. **Polos que se cancelan.** En procesos, la ecuación característica de $G_p$ suele **coincidir** con la de $G_d$ (misma dinámica del equipo), así que se cancelan. Lo que queda depende de las **ganancias estáticas** y de los **ceros** de $G_p$ y $G_d$. Y $G_m$ en el denominador **aporta ceros** a $G_{ff}$.
2. **Más ceros que polos** ⇒ $G_{ff}$ **impropia**, no implementable tal cual (habría que derivar). En la práctica se aproxima (se agregan polos rápidos o se compensa solo la ganancia).
3. **Retardos.** $G_{ff}$ contiene $e^{-(L_d-L_p-L_m)s}$:
   - $L_d \ge L_p+L_m$: retardo positivo, **realizable**.
   - $L_d < L_p+L_m$ (el proceso tiene **más retardo** que la perturbación): aparece un **adelanto** $e^{+\ldots s}$, **no realizable**. Se omite el término y se compensa lo demás.

**Ejemplo de la clase:** $G_p=\dfrac{10e^{-0.5s}}{1+2s}$, $G_d=\dfrac{2e^{-0.1s}}{1+2s}$, $G_m=1$:

$$G_{ff}=-\frac{G_d}{G_p}=-\frac{1}{5}\,e^{(-0.1+0.5)s}=-0.2\,e^{+0.4s}\quad\text{¡No implementable!}$$

(La perturbación llega a la salida antes de que la acción de control pueda hacerlo; lo único realizable es $G_{ff}=-0.2$.)

---

## 8. Conf. 5 — Feedforward vs. feedback y esquema combinado

### 8.1 Comparación

| | **Feedforward** | **Feedback** |
|---|---|---|
| Ventajas | Actúa **antes** de que la perturbación se sienta · Bueno en procesos **lentos** (multicapacitivos) · **No introduce inestabilidad** (está fuera del lazo) | No necesita identificar ni medir las perturbaciones · Insensible a errores de modelo · Insensible a cambios en los parámetros |
| Desventajas | Hay que **identificar y medir** cada perturbación · **No compensa** perturbaciones no medidas · Trabaja en **lazo abierto**: no detecta cambios en los parámetros · Exige **buen modelo** | La perturbación **se ve en la salida** antes de corregirse · Difícil de ajustar en procesos complejos · **Puede hacerse inestable** |

Además, algunas perturbaciones medibles son tan **poco frecuentes** que no justifican un CFF. Regla: el CFF se usa para las **perturbaciones grandes, medibles y modelables**, y se **combina con un feedback** que corrige el resto (y los errores del modelo).

### 8.2 Esquema FF-FB

```
                    D ──┬──────────────► Gd ──┐
                        └► Gm2 ─► Gff ─┐      │
                                       ▼      ▼
 Ysp ─►(+)─► Gfb ─────────────────────(+)─► Gp ─►(+)──┬──► Y
        ▲-                                            │
        └──────────────── Gm1 ◄───────────────────────┘
```

Las salidas del FB y del FF **se suman** para formar la acción de control. Con la receta de la sección 2.6 (guía Conf. 1–3):

$$Y=\frac{G_{fb}G_p}{1+G_{fb}G_pG_{m1}}\,Y_{sp}+\frac{G_d+G_{m2}G_{ff}G_p}{1+G_{fb}G_pG_{m1}}\,D$$

- El FF aparece **solo en el numerador** de la FT de la perturbación: **no cambia la ecuación característica**, así que **no afecta la estabilidad** ni la respuesta a la referencia.
- Si $G_{ff}$ es la ideal, el numerador se anula y $D$ no afecta a $Y$. Si es aproximada, el FB corrige lo que queda.

### 8.3 Ejercicio de la clase (intercambiador)

$$G_p=G_vG_I=\frac{2}{s+1}\cdot\frac{10\,e^{-0.1s}}{2s+1},\qquad G_d=\frac{2\,e^{-0.5s}}{2s+1},\qquad G_m=\frac{1}{0.5s+1}\ (\text{para }T_1\text{ y }T_2)$$

$$G_{ff}=-\frac{\frac{2}{2s+1}e^{-0.5s}}{\frac{1}{0.5s+1}\cdot\frac{2}{s+1}\cdot\frac{10}{2s+1}e^{-0.1s}}=-0.1\,(0.5s+1)(s+1)\,e^{-0.4s}$$

Se cancela el polo $(2s+1)$ y el retardo **sí es realizable** ($0.5>0.1$). Pero quedan **dos ceros y ningún polo** (impropia). Por eso en Simulink se comparan versiones parciales:

| Versión | $G_{ff}$ |
|---|---|
| Sin CFF | 0 |
| Ganancia estática + retardo | $-0.1\,e^{-0.4s}$ |
| + polo de la válvula | $-0.1\,(s+1)\,e^{-0.4s}$ |
| + polo del sensor | $-0.1\,(0.5s+1)\,e^{-0.4s}$ |
| Ambos polos (ideal) | $-0.1\,(s+1)(0.5s+1)\,e^{-0.4s}$ |

(Para implementar los ceros en Simulink se agrega un polo rápido, p. ej. $\dfrac{s+1}{0.01s+1}$.) Cuanto más dinámica se compensa, menor es la desviación de $T_2$ ante el escalón en $T_1$.

Después se agrega un **feedback PID por ZN** y se evalúa ante escalones en la referencia y en la perturbación:

```matlab
Gp = tf(2,[1 1])*tf(10,[2 1]);  Gp.OutputDelay = 0.1;
Gd = tf(2,[2 1]);               Gd.OutputDelay = 0.5;
Gm = tf(1,[0.5 1]);
[Ku,Pm,Wcg,Wcp] = margin(Gp*Gm);
Tu = 2*pi/Wcg;
kcp = Ku/1.7;  tip = Tu/2;  tdp = Tu/8;            % ZN para PID real (serie)
Gc = kcp*(1+tf(1,[tip 0]))*tf([tdp 1],[0.01 1]);   % PID serie con filtro
step(feedback(Gc*Gp,Gm)), grid
```

Con estos datos: $K_u\approx0.42$, $T_u\approx3.85$ ⇒ $K'_c\approx0.25$, $T'_i\approx1.93$, $T'_d\approx0.48$.
(La clase usa $K_u/1.7$; la tabla ZN de la Conf. 2 dice $K_u/1.6$. Son prácticamente iguales.)

**Conclusiones de la Conf. 5:** el feedforward es, en teoría, la estrategia **más exacta**, pero exige un **modelo preciso** y **medir la perturbación**. No siempre es realizable dinámicamente, pero **al menos en forma estática** ya mejora el sistema.

---

## 9. Conf. 6 — Predictor de Smith

### 9.1 Por qué molesta el retardo
- Mediciones de composición (densidad, índice de refracción, viscosidad…) y fenómenos de transporte o mezclado (reactores, tuberías) introducen **retardos puros**.
- Un retardo grande **degrada mucho** a un controlador lineal como el PID. Por ejemplo, ZN para un PORT solo vale con $L=(10\%\text{–}50\%)\,T$.
- Los métodos de síntesis (MO, MS, cancelación) **no sirven**: necesitarían $e^{+Ls}$ (sección 11 de la guía Conf. 1–3).
- Otto Smith (1957), *"The Process Engineer's Crystal Ball"*: compensar el retardo prediciendo la salida.

### 9.2 Deducción
Se escribe la planta como $G_p(s)\,L(s)$, con $L(s)=e^{-\theta s}$ el retardo y $G_p$ la parte sin retardo.

- La salida real: $Y=E\,G_cG_p\,L$.
- La salida **deseada sin retardo**: $Y^*=E\,G_cG_p=M\,G_p$.
- Lo que falta sumarle a $Y$ para obtener $Y^*$: $Y'=Y^*-Y=(1-L)\,M\,G_p$.

Como $M$ se conoce, $Y'$ **se calcula con el modelo**. Se realimenta $Y^*=Y+Y'$ en vez de $Y$:

```
 Ysp ─►(+)─E─► Gc ──M──┬──► [ Gp·e^{-θs} ] (planta real) ───────┬──► Y
        ▲-             │                                       │
        │              └──► [ Gp·(1 - e^{-θs}) ] (modelo) ─ Y' │
        │                                                  ▼   │
        └──────────────────────── Y* = Y + Y' ◄───────────(+)◄─┘
```
(En la diapositiva el bloque del modelo se dibuja como $G_p$ seguido de una resta entre su salida y la misma salida retardada.)

Resultado (modelo perfecto):

$$\frac{Y}{Y_{sp}}=\frac{G_cG_p}{1+G_cG_p}\,e^{-\theta s}$$

El retardo **sale de la ecuación característica**: se diseña $G_c$ para la planta **sin retardo** (con cualquier método, incluido MO/MS), y la salida simplemente llega $\theta$ más tarde.

### 9.3 Ejemplo de la clase: $G_p=\dfrac{e^{-s}}{0.5s+1}$
- $L=1>T=0.5$: **fuera** del rango de ZN.
- Ganancia crítica $K_u\approx1.5$ ($\omega_u\approx2.29$ rad/s). Un P por ZN: $K_c=1.5/2=0.75$:

  $$e_{ss}=\frac{1}{1+K_cK_p}=\frac{1}{1+0.75}=0.57\quad(\text{¡57 % de error!})$$
- Con $K_c=5$ el lazo simple es **inestable**.
- Sin retardo, con $K_c=5$: $e_{ss}=\dfrac{1}{1+5}=0.16$.
- **Con predictor de Smith y $K_c=5$**: estable, con $e_{ss}=0.16$ (en teoría la ganancia podría ser tan grande como se quiera, porque la planta sin retardo es de 1er orden).

### 9.4 Sensibilidad al modelo
- Se simuló con un error del **20 % en $T$** y del **20 % en $L$** de la planta (el modelo del predictor queda fijo).
- **Los errores en el retardo son más perjudiciales que los errores en las constantes de tiempo.**

### 9.5 Limitaciones (¡OJO!)
- La estructura es **internamente inestable** con plantas **integradoras** o con **polos inestables**: no se puede usar ahí (el modelo en paralelo corre en lazo abierto).
- **No permite actuar sobre la dinámica del rechazo a perturbaciones**: ante $D$ la respuesta sigue dependiendo de los polos de lazo abierto de la planta.
- Mejora: **predictor de Smith filtrado**: se agrega un filtro $F_r(s)$ en la realimentación del error de predicción ($Y-\hat Y$). Detalles de diseño: <http://controlautomaticoeducacion.com/predictor-de-smith-filtrado/>.

---

## 10. Conf. 6 — Compensador de respuesta inversa

### 10.1 El fenómeno
Algunos procesos tienen **dos capacidades (acumuladores de energía) en oposición**. Ante un cambio en la entrada, la salida **primero va en un sentido y después en el otro**.

**Ejemplo clásico: nivel del domo de una caldera.**
- Si sube de golpe el **agua de alimentación** (vapor constante), el nivel **primero baja y después sube** (el agua fría colapsa burbujas).
- Si sube de golpe la **demanda de vapor** (alimentación constante), el nivel **primero sube y después baja** (cae la presión, las burbujas se expanden: *swell*).

En el modelo, esto es un sistema de **fase no mínima**: un número **impar de ceros en el semiplano derecho**.

### 10.2 Dos capacidades de 1er orden en oposición

$$G_p=\frac{K_1}{\tau_1s+1}-\frac{K_2}{\tau_2s+1}=\frac{(K_1\tau_2-K_2\tau_1)\,s+(K_1-K_2)}{(\tau_1s+1)(\tau_2s+1)}$$

Cero en $\;z=-\dfrac{K_1-K_2}{K_1\tau_2-K_2\tau_1}$, que es **positivo** (respuesta inversa) si:

$$K_1>K_2\ \text{y}\ K_1\tau_2<K_2\tau_1\ \Rightarrow\ \frac{\tau_1}{\tau_2}>\frac{K_1}{K_2}>1\qquad\text{ó}\qquad K_1<K_2\ \text{y}\ K_1\tau_2>K_2\tau_1\ \Rightarrow\ 1>\frac{K_1}{K_2}>\frac{\tau_1}{\tau_2}$$

Interpretación: el término con **más ganancia** es el **más lento**. El rápido (de signo contrario) domina al principio y el lento al final.

**Ejemplo:** $K_1=2,\ \tau_1=3,\ K_2=1,\ \tau_2=1$ ($\tau_1/\tau_2=3>K_1/K_2=2>1$ ✔):

$$G_p=\frac{2}{3s+1}-\frac{1}{s+1}=\frac{-s+1}{(3s+1)(s+1)}\qquad(\text{cero en }s=+1)$$

- **PI por ZN** (`margin`, $K_c=K_u/2.2$, $T_i=T_u/1.2$): funciona, $e_{ss}=0$, pero es lento y con la respuesta inversa visible.
- **MO directo**: $G_c=\dfrac{1}{2T_us(T_us+1)G_p}=\dfrac{3s+1}{2s\,(-s+1)}$ → tiene un **polo en $s=+1$** (invierte el cero inestable): **inestable**.

### 10.3 El compensador (misma idea que Smith)
Se suma a la salida una señal calculada con el modelo para "borrar" el cero positivo:

$$Y^*=Y+K\left(\frac{1}{\tau_2s+1}-\frac{1}{\tau_1s+1}\right)M$$

Agrupando:

$$\frac{Y^*}{M}=\frac{\big(K_1\tau_2-K_2\tau_1-K(\tau_2-\tau_1)\big)\,s+(K_1-K_2)}{(\tau_1s+1)(\tau_2s+1)}$$

Para eliminar el cero:

$$\boxed{K=\frac{K_1\tau_2-K_2\tau_1}{\tau_2-\tau_1}}\qquad\Rightarrow\qquad\frac{Y^*}{M}=\frac{K_1-K_2}{(\tau_1s+1)(\tau_2s+1)}$$

El compensador tiene **ganancia estática cero** (los dos términos se restan en $s=0$), así que no altera el estado estable. Con $K_1>K_2$ y $\tau_1>\tau_2$, se sintoniza por **módulo óptimo** ($T_u=\tau_2$, la menor):

$$G_c=\frac{\tau_1s+1}{2\tau_2\,s\,(K_1-K_2)}$$

**Para el ejemplo:** $K=\dfrac{2\cdot1-1\cdot3}{1-3}=0.5$, la planta vista queda $\dfrac{1}{(3s+1)(s+1)}$ y

$$G_{comp}=0.5\left(\frac{1}{3s+1}-\frac{1}{s+1}\right)\ (\text{se resta a }Y),\qquad G_c=\frac{3s+1}{2s}$$

(Restar $0.5\left(\frac{1}{3s+1}-\frac{1}{s+1}\right)$ es lo mismo que sumar $0.5\left(\frac{1}{s+1}-\frac{1}{3s+1}\right)$, como en la fórmula general.) La respuesta es mucho mejor que con el PI por ZN.

**Sensibilidad:** igual que Smith, es **muy sensible** a errores de modelo. Con un 10 % de error en $\tau_1$ ($\tau_1=3.3$) la respuesta se degrada notablemente.

### 10.4 Caso con integrador

$$G_p=\frac{K_1}{s}-\frac{K_2}{\tau_2s+1}=\frac{(K_1\tau_2-K_2)\,s+K_1}{s\,(\tau_2s+1)},\qquad z=-\frac{K_1}{K_1\tau_2-K_2}$$

Positivo si $K_1>0$ y $K_1\tau_2<K_2$, es decir, $\tau_2<K_2/K_1$.
**Ejemplo:** $K_1=1,\ K_2=3,\ \tau_2=2$ ⇒ $G_p=\dfrac{-s+1}{s(2s+1)}$.

- PI por ZN: logra $e_{ss}=0$, pero con un sobrepaso muy grande y una respuesta muy lenta (la gráfica de la clase necesita más de 100 s).
- Mejora propuesta: compensador + **módulo simétrico** (la planta ya trae un integrador):

  $$G_c=\frac{4T_us+1}{K_r\,8T_u^2s^2(T_us+1)\,G_p}$$
- *Desarrollo (la diapositiva lo deja como ejercicio):* con el mismo patrón, $Y^*=Y+K\left(\dfrac{1}{\tau_2s+1}-\dfrac{1}{s}\right)M$. El coeficiente de $s$ del numerador queda $K_1\tau_2-K_2-K(\tau_2-1)$, que se anula con $K=\dfrac{K_1\tau_2-K_2}{\tau_2-1}$. Para el ejemplo $K=-1$ y $\dfrac{Y^*}{M}=\dfrac{K_1-K}{s(\tau_2s+1)}=\dfrac{2}{s(2s+1)}$; el MS con $T_u=2$ da el PI $G_c=\dfrac{8s+1}{64s}$ (lazo nominal estable: $64s^3+32s^2+8s+1$).
- **Problema:** funciona bien ante la **referencia**, pero ante **perturbaciones se vuelve inestable**. El compensador contiene un **integrador en lazo abierto** ($1/s$): para rechazar una perturbación $M$ debe quedar en un valor distinto de cero, y entonces $Y'$ crece como una rampa sin límite.

---

## 11. Conf. 6 — Control inferencial

Se usa cuando la variable a controlar ($Y_1$) es **muy difícil de medir**, pero se puede medir otra ($Y_2$) **estrechamente relacionada**.
*Ejemplo:* la **composición del destilado** en el tope de una columna (difícil de medir) está muy ligada a la **temperatura del tope** (fácil).

Modelo (misma manipulada $M$ y misma perturbación $D$, **no medida**):

$$Y_1=G_{p1}M+G_{d1}D,\qquad Y_2=G_{p2}M+G_{d2}D$$

Despejando $D$ de la segunda:

$$D=\frac{1}{G_{d2}}Y_2-\frac{G_{p2}}{G_{d2}}M$$

y reemplazando en la primera:

$$\boxed{\hat Y_1=\left(G_{p1}-\frac{G_{d1}}{G_{d2}}G_{p2}\right)M+\frac{G_{d1}}{G_{d2}}\,Y_2}$$

- Se **infiere** $Y_1$ a partir de la medición de $Y_2$ y de $M$. Ese $\hat Y_1$ es el que se **realimenta**.
- El controlador se diseña **para $G_{p1}$**.
- Diagrama: referencia → (+) → controlador → proceso (salidas $Y_1$, $Y_2$); $M$ pasa por $G_{p1}-G_{d1}G_{p2}/G_{d2}$, $Y_2$ pasa por $G_{d1}/G_{d2}$, se suman y vuelven al comparador como "$Y_1$ inferida".
- Limitaciones: depende **totalmente del modelo**, y no siempre es realizable porque hay **inversión de FT** ($1/G_{d2}$: puede quedar impropia o con adelantos).

**Conclusiones de la Conf. 6:**
- Con retardo muy alto se recomienda el **predictor de Smith** (hay controladores comerciales que lo traen integrado). Un modelo mal estimado, sobre todo el retardo, deteriora su desempeño.
- Los **compensadores de respuesta inversa** permiten sintetizar mejores controladores para plantas de fase no mínima, pero **no** se pueden usar si las partes que causan la respuesta inversa son **inestables o integradores puros**.
- El **control inferencial** controla una variable no medible estimándola a partir de otra, pero depende mucho del modelo y no siempre es realizable.

---

## 12. Trampas típicas y checklist

- [ ] **Cascada:** primero el lazo **interno**, más rápido que el externo; el interno con **acción integral** si se usa RFB.
- [ ] **RFB:** realimentar la **medición del lazo interno** (o la salida del selector), no la salida propia del controlador.
- [ ] **Sobrecorrida vs. selectivo:** sobrecorrida = varios **controladores** → un actuador; selectivo = varias **mediciones** → un controlador.
- [ ] **Selector LS o HS:** pensar en qué dirección es la "segura" para la válvula (en el ejemplo del tanque, cerrar ⇒ LS).
- [ ] **Rango dividido:** dibujar la posición de cada válvula vs. la acción de control y razonar qué pasa a cada lado del punto de corte.
- [ ] **Relación:** preferir $F_{B,sp}=R\cdot F_A$ (lineal) sobre el controlador de razón $F_B/F_A$ (no lineal).
- [ ] **$G_{ff}=-G_d/(G_mG_p)$:** no olvidar el **signo menos** ni el sensor $G_m$ (y la válvula dentro de $G_p$).
- [ ] **Realizabilidad del FF:** contar polos vs. ceros y revisar el retardo neto $L_d-L_p-L_m$; si es negativo, se omite.
- [ ] El FF **no cambia la ecuación característica**: no desestabiliza, pero tampoco corrige errores de modelo → siempre con FB.
- [ ] **Smith:** el controlador se diseña para la planta **sin retardo**; no usar con plantas integradoras o inestables.
- [ ] Errores en **$L$** son peores que errores en **$T$** para el predictor.
- [ ] **Respuesta inversa:** MO directo invierte el cero del SPD ⇒ controlador **inestable**. Compensar primero.
- [ ] $K=\dfrac{K_1\tau_2-K_2\tau_1}{\tau_2-\tau_1}$ y después MO sobre $\dfrac{K_1-K_2}{(\tau_1s+1)(\tau_2s+1)}$.
- [ ] **Inferencial:** el controlador se diseña para $G_{p1}$; revisar si $1/G_{d2}$ es realizable.

**Preguntas conceptuales probables:**
1. ¿Por qué el lazo interno de una cascada permite subir la ganancia del externo?
2. ¿Qué es el RFB y por qué el controlador interno debe tener acción integral?
3. Diferencias entre sobrecorrida, selectivo y rango dividido (cuántos controladores, mediciones y actuadores).
4. Explicar el rango dividido del reactor: ¿qué pasa si la acción sube de 0.6?
5. ¿Por qué el controlador de razón $F_B/F_A$ es no lineal?
6. Deducir $G_{ff}$ y discutir su realizabilidad (ceros, retardos).
7. Ventajas y desventajas del FF frente al FB; ¿por qué se combinan?
8. Deducir el predictor de Smith; ¿cuándo no se puede usar?
9. ¿Qué es un sistema de fase no mínima? ¿Por qué el MO falla y cómo se compensa?
10. ¿En qué consiste el control inferencial y cuál es su principal limitación?

---

## 13. Formulario resumen

| Concepto | Fórmula |
|---|---|
| PI como realimentación positiva (RFB) | $\dfrac{K_c}{1-\frac{1}{t_is+1}}=K_c\left(1+\frac{1}{t_is}\right)$ |
| Relación (recomendada) | $F_{B,sp}=R\cdot F_A$, $\;\partial F_B/\partial F_A=R$ |
| Relación (controlador de razón) | $\partial(F_B/F_A)/\partial F_A=-F_B/F_A^2$ (no lineal) |
| Feedforward ideal | $G_{ff}=-\dfrac{G_d}{G_mG_p}$ |
| FF estático | $G_{ff}=-\dfrac{K_d}{K_mK_p}$ |
| FF + FB | $Y=\dfrac{G_{fb}G_p}{1+G_{fb}G_pG_{m1}}Y_{sp}+\dfrac{G_d+G_{m2}G_{ff}G_p}{1+G_{fb}G_pG_{m1}}D$ |
| Retardo neto del FF | $e^{-(L_d-L_p-L_m)s}$, realizable si $L_d\ge L_p+L_m$ |
| Predictor de Smith | $Y'=(1-e^{-\theta s})G_pM$, $\;\dfrac{Y}{Y_{sp}}=\dfrac{G_cG_p}{1+G_cG_p}e^{-\theta s}$ |
| Respuesta inversa (2 capacidades) | $G_p=\dfrac{(K_1\tau_2-K_2\tau_1)s+(K_1-K_2)}{(\tau_1s+1)(\tau_2s+1)}$, cero SPD si $\frac{\tau_1}{\tau_2}>\frac{K_1}{K_2}>1$ |
| Compensador de resp. inversa | $Y^*=Y+K\left(\frac{1}{\tau_2s+1}-\frac{1}{\tau_1s+1}\right)M$, $\;K=\dfrac{K_1\tau_2-K_2\tau_1}{\tau_2-\tau_1}$ |
| Controlador tras compensar (MO) | $G_c=\dfrac{\tau_1s+1}{2\tau_2s(K_1-K_2)}$ |
| Resp. inversa con integrador | $G_p=\dfrac{(K_1\tau_2-K_2)s+K_1}{s(\tau_2s+1)}$, cero SPD si $\tau_2<K_2/K_1$ |
| Control inferencial | $\hat Y_1=\left(G_{p1}-\dfrac{G_{d1}}{G_{d2}}G_{p2}\right)M+\dfrac{G_{d1}}{G_{d2}}Y_2$ |

**Bibliografía de las conferencias:**
- Corripio & Smith, *Principles and Practice of Automatic Process Control*, 2ª ed.: cap. 5 págs. 243–244 (RFB); cap. 10 págs. 439–445 (cascada); cap. 11 págs. 470–476 (selectivo); cap. 12 págs. 487–490 (rango dividido) y 494–526 (feedforward).
- Stephanopoulos, *Chemical Process Control*: págs. 383–390 (Smith), 390–392 (resp. inversa), 438–443 (inferencial).
- <https://www.controlglobal.com/articles/2006/095/> (RFB) · <http://controlautomaticoeducacion.com> (Smith filtrado).
