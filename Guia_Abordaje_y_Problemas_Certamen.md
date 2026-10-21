# Certamen 1, Control de Procesos: cómo abordar los ejercicios y problemas tipo

> Guía construida a partir de las Conferencias 1–3 y los prácticos CP-1 a CP-4 (ver las carpetas `CP1/` … `CP4/`).
> Los problemas de la Parte D están pensados para resolverse **a mano** (calculadora científica). Todas las soluciones fueron verificadas numéricamente.

## Índice
- [Parte A: Método general para cualquier ejercicio](#parte-a-método-general-para-cualquier-ejercicio)
- [Parte B: ¿Qué método de diseño uso?](#parte-b-qué-método-de-diseño-uso)
- [Parte C: Recetas por tipo de problema](#parte-c-recetas-por-tipo-de-problema)
- [Parte D: Problemas tipo certamen resueltos](#parte-d-problemas-tipo-certamen-resueltos)
- [Parte E: Preguntas conceptuales probables (con respuesta)](#parte-e-preguntas-conceptuales-probables-con-respuesta)
- [Parte F: Errores que más restan puntos](#parte-f-errores-que-más-restan-puntos)
- [Parte G: Estrategia el día del certamen](#parte-g-estrategia-el-día-del-certamen)

---

## Parte A: Método general para cualquier ejercicio

Antes de calcular, siempre estos 6 pasos. Anotarlos en la hoja también da puntos.

1. **Identificar variables.** Cuál es la salida controlada, cuál la manipulada y **dónde entra la perturbación** (antes o después de cada bloque, dentro o fuera del lazo interno). Dibujar el diagrama de bloques si no viene.
2. **Llevar cada bloque a la forma de constantes de tiempo** $\dfrac{K}{Ts+1}$.
   - Ejemplo: $\dfrac{2}{s+2}=\dfrac{1}{0.5s+1}$ y $\dfrac{100}{s(s+10)}=\dfrac{10}{s(0.1s+1)}$.
   - Este es el error más común: las constantes de tiempo y las ganancias se leen **solo** en esta forma.
3. **Clasificar la planta:**
   - Signo de $K$: acción directa o inversa, de modo que el controlador sea el opuesto para que la realimentación sea negativa.
   - **Tipo** (número de integradores $1/s$).
   - **¿Tiene retardo** $e^{-Ls}$? Con retardo, MO, MS y cancelación **no** se pueden aplicar; se usa ZN, CC o criterios integrales.
   - Constantes de tiempo ordenadas de menor a mayor.
4. **Leer qué se exige:** error cero ante qué entrada (escalón o rampa, referencia o perturbación), sobrepaso máximo, tiempo de establecimiento, límite del mando. Traducirlo a requisitos:
   - error cero ante escalón en la **referencia** → integrador en el lazo (en C o en G);
   - error cero ante escalón en la **perturbación** → integrador **en el controlador**, antes del punto de entrada;
   - error cero ante **rampa** → lazo tipo 2 (MS);
   - $M_p\le4\%$ → $\zeta=0.707$ (MO, o PI de 2º orden con $\zeta=0.7$).
5. **Elegir el método** (Parte B) y **aplicar la receta** (Parte C).
6. **Verificar y comentar:**
   - llevar el controlador a la forma **$P+I/s+Ds$** e **identificarlo** (P, PI, PD, PID);
   - calcular $e_{ss}$ con el teorema del valor final;
   - comprobar el mando máximo si hay saturación;
   - concluir en una o dos frases (rapidez, sobrepaso, error, perturbación).

---

## Parte B: ¿Qué método de diseño uso?

```
¿La planta tiene retardo e^{-Ls}?
├── SÍ ──► Sintonía empírica/óptima sobre un PORT o sobre Ku,Tu:
│          ├── ¿Se puede llevar a oscilación sostenida o hay modelo? → ZN lazo cerrado (Ku, Tu)  [PID SERIE]
│          ├── Solo respuesta al escalón (curva de reacción)         → ZN lazo abierto (K,T,L)   [PID SERIE]
│          ├── Menos sensible a L/T                                   → Cohen-Coon                [PID IDEAL]
│          └── Minimizar un índice (dicen ISE/IAE/ITAE)               → Tablas de criterios       [PID IDEAL]
│                ├── "ante cambios en la referencia"  → Rovira (TI = τ/(a2 + b2·L/τ))
│                └── "ante perturbaciones"            → Smith-Murrill (TI = (τ/a2)(L/τ)^b2)
└── NO ──► Síntesis basada en el modelo:
           ├── Planta de 1er orden K/(Ts+1):
           │     ├── simple / piden cancelar             → PI por cancelación (Ti = T)
           │     └── piden ζ, Mp, tss o buen rechazo      → PI con 2º orden impuesto (+ prefiltro si tss < 5T)
           ├── Piden Mp ≈ 4 %, ζ = 0.707, "óptimo"         → MÓDULO ÓPTIMO
           ├── Piden error 0 ante rampa, o ante perturbación
           │   con planta que ya trae el integrador         → MÓDULO SIMÉTRICO
           └── Hay un sensor intermedio / planta compleja /
               la perturbación entra en el medio            → CASCADA (interno MO, externo MO o MS)
```

**Qué forma de PID entrega cada método (¡se pregunta mucho!):**

| Método | Forma | Qué hay que hacer para pasarlo a $P+I/s+Ds$ |
|---|---|---|
| ZN (lazo cerrado y lazo abierto) | **Serie** $K'_c(1+\frac{1}{T'_is})(T'_ds+1)$ | Convertir a ideal: $K_c=K'_c(1+\frac{T'_d}{T'_i})$, $T_i=T'_i+T'_d$, $T_d=\frac{T'_iT'_d}{T'_i+T'_d}$. Luego $P=K_c$, $I=K_c/T_i$, $D=K_cT_d$ |
| Cohen-Coon, ISE/IAE/ITAE | **Ideal** $K_c(1+\frac{1}{T_is}+T_ds)$ | $P=K_c$, $I=K_c/T_i$, $D=K_cT_d$ |
| MO, MS, cancelación | Sale directo como FT | Dividir término a término por $s$ (o fracciones parciales) |

---

## Parte C: Recetas por tipo de problema

### C0. Escribir las FT de lazo cerrado (se usa en todas las recetas)
1. Dibujar el diagrama y marcar dónde entra cada señal ($R$, $D$).
2. Escribir **una ecuación por bloque**: $E=R-Y$, $U=G_cE$, $Y=G_p(U+D)$ (o $Y=G_pU+G_dD$ si la perturbación tiene su propio camino).
3. Reemplazar hasta que solo quede $Y$ y despejar: $Y(1+G_cG_p)=G_cG_pR+G_pD$.
4. Leer cada FT con la otra entrada en cero:

$$\frac{Y}{R}=\frac{G_cG_p}{1+G_cG_p},\qquad \frac{Y}{D}=\frac{G_p}{1+G_cG_p}\ \Big(\text{o }\frac{G_d}{1+G_cG_p}\Big),\qquad \frac{U}{R}=\frac{G_c}{1+G_cG_p}$$

   Atajo: **camino directo desde la entrada hasta $Y$, dividido por 1 + ganancia del lazo.**
5. Simplificar con $G_c=n_c/d_c$, $G_p=n_p/d_p$: $\;\dfrac{Y}{R}=\dfrac{n_cn_p}{d_cd_p+n_cn_p}$, $\;\dfrac{Y}{D}=\dfrac{n_pd_c}{d_cd_p+n_cn_p}$. El denominador es la ecuación característica, **igual para ambas**.
6. Interpretar:
   - Si el controlador **canceló** un polo de la planta, ese polo no está en $Y/R$ pero **reaparece en $Y/D$** (el numerador $G_p$ no pasa por el controlador) → rechazo más lento.
   - Si $d_c$ tiene $s$ (integrador en $G_c$), $Y/D$ tiene $s$ arriba → $Y/D(0)=0$, sin error ante perturbación escalón.
   - Con prefiltro $P$: solo se multiplica $Y/R$ por $P$; $Y/D$ no cambia.
7. **Cascada** (interno $G_{c1}$ sobre $G_a$, externo $G_{c2}$ sobre $G_b$, $D$ a la entrada de $G_a$): mismas ecuaciones con $U=G_{c1}(G_{c2}(R-Y)-V)$, $V=G_a(U+D)$, $Y=G_bV$:

$$\frac{Y}{D}=\frac{G_aG_b}{1+G_aG_{c1}(1+G_{c2}G_b)},\qquad \frac{Y}{R}=\frac{G_{c2}G_{LC1}G_b}{1+G_{c2}G_{LC1}G_b},\ G_{LC1}=\frac{G_{c1}G_a}{1+G_{c1}G_a}$$

Ejemplos resueltos paso a paso: Resumen secciones 2.6, 8.1, 8.2, 9, 10 y 11; CP2 (diseños A y B); CP3 (ejercicios 2, 3 y 4); CP4 (ejercicios 1 y 2).

### C1. Error en estado estable (Conf. 1)
1. Escribir la FT del error: ante la referencia $E=\dfrac{1}{1+CG}X$, ante la perturbación $Y=\dfrac{G_d}{1+CG}D$.
2. Aplicar el teorema del valor final: $e_{ss}=\lim_{s\to0}sE(s)$, con $X=1/s$ (escalón) o $1/s^2$ (rampa).
3. Atajos por tipo del lazo $CG$:

| Tipo de $CG$ | Escalón | Rampa |
|---|---|---|
| 0 | $\frac{1}{1+K_p}$, con $K_p=CG(0)$ | ∞ |
| 1 | 0 | $\frac{1}{K_v}$, con $K_v=\lim sCG$ |
| 2 | 0 | 0 |

4. Ante la perturbación a la entrada de la planta, con **G tipo 1 y C sin integrador**: $y_{ss}=1/C(0)$. Solo es 0 si **C** tiene integrador.

### C2. Identificar un PORT de la curva de reacción (Conf. 2 y CP-1)
1. $K=\dfrac{\Delta Y}{\Delta U}$, en unidades de salida/entrada y con signo.
2. Leer $t_{28}$ y $t_{63}$ **desde el instante del escalón** (incluyen el retardo).
3. $T=1.5\,(t_{63}-t_{28})$ y $L=t_{63}-T$.
4. Calcular $L/T$ y comentar la validez: ZN 0.1–0.5, criterios integrales 0.1–1.

### C3. Ku y Tu (a mano o con `margin`)
1. Plantear la condición de fase: $\sum -\arctan(T_i\omega_u)-L\omega_u=-180°$ (radianes: $-\pi$).
2. Para $\dfrac{K}{(Ts+1)^n}$ sin retardo hay forma cerrada:
   - $n=3$: $\omega_u=\sqrt3/T$ y $K_u=\dfrac{8}{K}$ (porque $|1+j\sqrt3|^3=2^3$);
   - $n=2$: nunca llega a −180°, así que **no hay Ku** (el lazo con P es siempre estable).
3. $K_u=\dfrac{1}{|G(j\omega_u)|}$ y $T_u=\dfrac{2\pi}{\omega_u}$. En MATLAB, `[Gm,~,Wcg]=margin(G)` da $K_u=G_m$ (**no en dB**).

### C4. Tablas ZN, CC y criterios integrales
Aplicar la tabla, **decir la forma** (serie o ideal) y convertir a paralelo. Si piden comparar métodos:
- ZN con Ku es el más agresivo: mejor ante la perturbación, peor sobrepaso.
- Los criterios "para la referencia" dan un $T_i$ grande: no sobrepasan, pero rechazan lento la perturbación.
- Los criterios "para la perturbación" hacen lo contrario.

### C5. Planta de 1er orden $K/(Ts+1)$ (Conf. 3 y CP-2)
**Cancelación**
- $T_i=T$, $T_{LC}=t_{ss}/4$, $K_p=\dfrac{T}{T_{LC}K}$.
- Lazo cerrado de 1er orden, sin sobrepaso.
- ❌ Ante la perturbación vuelve el polo $T$: la respuesta es lenta.
  (Porque $G_cG_p=\frac{K_pK}{Ts}$ pero $\frac{Y}{D}=\frac{G_p}{1+G_cG_p}=\frac{KTs}{(Ts+1)(Ts+K_pK)}$: el $(Ts+1)$ de $G_p$ no se cancela.)
- Límite del mando: con PI por cancelación el máximo de $u$ ocurre en $t=0^+$ y vale $K_p\,\Delta r$. Además hay que chequear el valor final $\Delta r/K$.

**2º orden impuesto**

$$\omega_n=\frac{4}{\zeta t_{ss}},\quad K_p=\frac{2\zeta\omega_nT-1}{K},\quad T_i=\frac{K_pK}{T\omega_n^2}$$

- Requiere $t_{ss}<8T$ para que $K_p>0$.
- Si $t_{ss}<5T$ → prefiltro $\dfrac{1}{T_is+1}$.
- ✅ Tiene los mismos polos ante la referencia y ante la perturbación.
  (Ecuación característica $T_iTs^2+T_i(1+K_pK)s+K_pK=0$ para ambas; $\frac{Y}{R}=\frac{\omega_n^2(T_is+1)}{s^2+2\zeta\omega_ns+\omega_n^2}$ y $\frac{Y}{D}=\frac{(K/T)s}{s^2+2\zeta\omega_ns+\omega_n^2}$.)

### C6. Módulo óptimo y módulo simétrico
1. Forma de constantes de tiempo. $T_u$ = **la menor**, que queda sin compensar.
2. Sustituir en

$$G_c^{MO}=\frac{1}{K_r\,2T_us(T_us+1)G_p}\qquad G_c^{MS}=\frac{4T_us+1}{K_r\,8T_u^2s^2(T_us+1)G_p}$$

3. Cancelar y expandir. Cada constante lenta aporta un cero.

| Planta (sin contar $T_u$) | MO da | MS da |
|---|---|---|
| 1 constante lenta, sin integrador | PI | PI + doble integrador |
| 1 integrador | P | PI |
| 1 integrador + 1 lenta | PD | PID |
| 2 lentas, sin integrador | PID | PID + doble integrador |

4. Resultado esperado:
   - **MO:** $\omega_n=\dfrac{1}{\sqrt2T_u}$, $\zeta=0.707$, $M_p\approx4.3\%$, $t_s\approx8.4T_u$.
   - **MS:** $M_p\approx43\%$, margen de fase ≈ 37°, $\omega_c=\dfrac{1}{2T_u}$.

### C7. Cascada (mando subordinado)
1. **Primero el lazo interno** (el que mide la variable intermedia), por MO con su $T_u$.
2. Aproximar el lazo interno cerrado: $\dfrac{1/K_{r1}}{2T_u^2s^2+2T_us+1}\approx\dfrac{1/K_{r1}}{2T_us+1}$.
3. Lazo externo: planta = (1er orden con **$2T_u$**) × el resto. Diseñar por MO o MS con **$T_u^{ext}=2T_u^{int}$**.
4. Comentar:
   - dos PI en vez de un PID;
   - la perturbación que entra **dentro** del lazo interno se rechaza mucho mejor;
   - el lazo externo es algo más lento (su $T_u$ es el doble);
   - el mando es más suave (sin golpe derivativo).

### C8. Implementación práctica (saturación, anti-windup, filtro)
- Saturación sin anti-windup → windup: más sobrepaso y respuesta lenta.
- Anti-windup (back-calculation): $K_b=1/T_i$ en un PI y $1/\sqrt{T_iT_d}$ en un PID (Conf. 1). La guía del CP-3 sugiere $K_b=I/P$.
- Filtro derivativo: $\dfrac{Ds}{s/N+1}$, con $1/N\ll T_u$ (en el CP-3 vimos que si $1/N$ es comparable a $T_u$ el diseño se degrada).
- D sobre la medición: elimina el golpe derivativo ante la referencia. Pero en diseños por cancelación (MO) quita el cero que cancelaba el polo lento ante la referencia.

---

## Parte D: Problemas tipo certamen resueltos

### Problema 1: Error en estado estable (tipo Conf. 1)
Planta $G(s)=\dfrac{5}{(s+1)(0.5s+1)}$ con realimentación unitaria. La perturbación $D$ entra a la **entrada** de la planta.

**a)** Con un controlador P, $K_c=2$, calcule el error ante un escalón unitario en la referencia y la desviación de la salida ante un escalón unitario en la perturbación.
**b)** ¿Qué pasa ante una rampa?
**c)** ¿Qué controlador elimina ambos errores ante escalón? Justifique.

**Solución**
- **a)** $CG$ es tipo 0 y $K_p=CG(0)=2\cdot5=10$.
  - Referencia: $e_{ss}=\dfrac{1}{1+10}=\mathbf{0.091}$.
  - Perturbación: $y_{ss}=\lim_{s\to0}\dfrac{G}{1+CG}=\dfrac{5}{11}=\mathbf{0.455}$.
- **b)** Con un lazo tipo 0, el error ante rampa es $e_{ss}=\infty$.
- **c)** Un **PI**. El integrador en el **controlador** hace el lazo tipo 1, con error cero ante escalón en la referencia. Además está *antes* del punto de entrada de la perturbación, así que también la anula. Poner el integrador en la planta solo arreglaría la referencia.

---

### Problema 2: Curva de reacción → ZN, CC y criterios integrales (tipo CP-1)
Se aplica un escalón de 5 % en la válvula y la temperatura sube de 0 a 20 °C en estado estable. La curva alcanza el 28.3 % del cambio en $t=6$ min y el 63.2 % en $t=11$ min (ambos medidos desde el escalón).

**a)** Obtenga el modelo PORT. **b)** Sintonice un PID por ZN y llévelo a la forma paralela. **c)** Sintonice un PID por Cohen-Coon. **d)** Sintonice un PI por ITAE para perturbaciones y otro por IAE para cambios de referencia. Compare.

**Solución**
- **a)** Modelo PORT:
  - $K=20/5=\mathbf{4}$ °C/%.
  - $T=1.5(11-6)=\mathbf{7.5}$ min.
  - $L=11-7.5=\mathbf{3.5}$ min.
  - $L/T=0.467$: dentro de 0.1–0.5, así que ZN es válido.
- **b)** ZN (serie):
  - $K'_c=1.2\dfrac{T}{KL}=1.2\dfrac{7.5}{14}=\mathbf{0.643}$, $T'_i=2L=\mathbf{7}$, $T'_d=0.5L=\mathbf{1.75}$.
  - A ideal: $K_c=0.643(1+1.75/7)=\mathbf{0.804}$, $T_i=\mathbf{8.75}$, $T_d=\dfrac{7\cdot1.75}{8.75}=\mathbf{1.4}$.
  - Paralelo: $P=0.804$, $I=0.804/8.75=\mathbf{0.0919}$, $D=0.804\cdot1.4=\mathbf{1.125}$.
- **c)** CC (ideal), con $r=0.467$:
  - $K_c=\dfrac{7.5}{14}\left(\dfrac43+\dfrac{0.467}{4}\right)=\mathbf{0.777}$.
  - $T_I=3.5\dfrac{32+6(0.467)}{13+8(0.467)}=\mathbf{7.28}$.
  - $T_D=\dfrac{4(3.5)}{11+2(0.467)}=\mathbf{1.17}$.
- **d)** Criterios integrales:
  - ITAE perturbación: $K_c=\dfrac{0.859}{4}(0.467)^{-0.977}=\mathbf{0.452}$ y $T_I=\dfrac{7.5}{0.674}(0.467)^{0.680}=\mathbf{6.63}$.
  - IAE referencia: $K_c=\dfrac{0.758}{4}(0.467)^{-0.861}=\mathbf{0.365}$ y $T_I=\dfrac{7.5}{1.02-0.323(0.467)}=\mathbf{8.63}$.
  - **Comparación:** el ajuste para la referencia es más conservador (menos $K_c$, más $T_I$). Sobrepasa poco, pero rechaza lento la perturbación. El de perturbación hace lo contrario.

---

### Problema 3: Ku y Tu a mano (tipo CP-1, CP-4)
Para $G(s)=\dfrac{2}{(s+1)^3}$, determine $K_u$ y $T_u$ analíticamente, sintonice un PID por ZN y expréselo como $P+I/s+Ds$.

**Solución**
1. Condición de fase: $-3\arctan\omega_u=-180°$, de donde $\arctan\omega_u=60°$ y $\omega_u=\sqrt3=\mathbf{1.732}$ rad/s.
2. Módulo: $|G(j\omega_u)|=\dfrac{2}{(\sqrt{1+3})^3}=\dfrac{2}{8}=0.25$, así que $K_u=\mathbf{4}$ y $T_u=\dfrac{2\pi}{\sqrt3}=\mathbf{3.628}$ s.
3. ZN serie: $K'_c=4/1.6=\mathbf{2.5}$, $T'_i=\mathbf{1.814}$, $T'_d=\mathbf{0.4534}$.
4. A ideal: $K_c=2.5(1+0.25)=\mathbf{3.125}$, $T_i=\mathbf{2.267}$, $T_d=\mathbf{0.3628}$.
5. **$P=3.125$, $I=1.378$, $D=1.134$.**

> Truco: en ZN con $K_u$ y $T_u$ siempre se cumple $T'_d/T'_i=1/4$, así que la conversión a ideal es $K_c=1.25K'_c$, $T_i=1.25T'_i$ y $T_d=0.8T'_d$.

---

### Problema 4: PI para planta de 1er orden con límite de mando (tipo CP-2)
$G_p=\dfrac{4}{8s+1}$ (min). Se quiere $t_{ss}=10$ min (en lazo abierto es $4T=32$ min).

**a)** Diseñe un PI por cancelación. ¿Cuál es el mando inicial ante un escalón de 2 en la referencia? Si el mando admite ±2, ¿satura?
**b)** Diseñe un PI con 2º orden impuesto con $M_p=4\%$. ¿Necesita prefiltro?
**c)** Compare el rechazo a una perturbación escalón a la entrada de la planta.

**Solución**
- **a)** $T_i=T=\mathbf{8}$, $T_{LC}=10/4=2.5$ y $K_p=\dfrac{8}{2.5\cdot4}=\mathbf{0.8}$.
  - Mando: $u(0^+)=K_p\Delta r=0.8\cdot2=1.6$ y $u(\infty)=\Delta r/K=0.5$.
  - Con cancelación el máximo es $u(0^+)=1.6<2$: **no satura**.
- **b)** $\zeta=0.7$ y $\omega_n=\dfrac{4}{0.7\cdot10}=\mathbf{0.571}$.
  - $K_p=\dfrac{2(0.7)(0.571)(8)-1}{4}=\mathbf{1.35}$ y $T_i=\dfrac{1.35\cdot4}{8\cdot0.571^2}=\mathbf{2.07}$ min.
  - Como $t_{ss}=10<5T=40$, **sí conviene prefiltro** $\dfrac{1}{2.07s+1}$. Sin él el sobrepaso sería ≈ 15 %, con él ≈ 4.6 %.
- **c)** Ante perturbación unitaria:

| Diseño | Desviación máxima | Tiempo de recuperación |
|---|---|---|
| Cancelación | ≈ 0.74 | ≈ 38 min |
| 2º orden | ≈ 0.40 | ≈ 12 min |

  Con cancelación, ante la perturbación reaparece el polo de 8 min. El diseño de 2º orden tiene los mismos polos para ambas entradas.

  **FT (receta C0, $Y/D=\frac{G_p}{1+G_cG_p}$):**
  - Cancelación: $G_cG_p=0.8\frac{8s+1}{8s}\cdot\frac{4}{8s+1}=\frac{3.2}{8s}$, así que $\dfrac{Y}{R}=\dfrac{3.2}{8s+3.2}=\dfrac{1}{2.5s+1}$ y

    $$\frac{Y}{D}=\frac{4}{8s+1}\cdot\frac{8s}{8s+3.2}=\frac{10\,s}{(8s+1)(2.5s+1)}$$

    El polo $(8s+1)$ de la planta sigue ahí → recuperación de ≈ $4\cdot8$ min.
  - 2º orden: ecuación característica $T_iTs^2+T_i(1+K_pK)s+K_pK$ → $s^2+0.8s+0.326$, y

    $$\frac{Y}{D}=\frac{(K/T)\,s}{s^2+0.8s+0.326}=\frac{0.5\,s}{s^2+0.8s+0.326}$$

    Mismos polos que $Y/R$ ($\zeta\omega_n=0.4$ → ≈ 10–12 min).

---

### Problema 5: Linealización + diseño (tipo CP-2)
Un estanque de área $A=2$ m² tiene caudal de entrada $F_i$ (manipulable) y salida libre $F_o=k\sqrt h$, con $k=0.5$. En operación, $F_{i0}=1$ m³/min.

**a)** Encuentre $h_0$. **b)** Linealice y obtenga $H(s)/F_i(s)$. **c)** Diseñe un PI por cancelación para $t_{ss}=20$ min.

**Solución**
- **a)** En estado estable $F_{i0}=k\sqrt{h_0}$, de donde $h_0=(1/0.5)^2=\mathbf{4}$ m.
- **b)** $A\dfrac{dh}{dt}=F_i-k\sqrt h$. Por Taylor: $k\sqrt h\approx k\sqrt{h_0}+\dfrac{k}{2\sqrt{h_0}}\Delta h=1+0.125\,\Delta h$.
  - Ecuación de desviación: $2\dfrac{d\Delta h}{dt}+0.125\Delta h=\Delta F_i$.
  - $$\frac{H}{F_i}=\frac{8}{16s+1}\qquad(K=8\ \text{min/m}^2,\ \tau=16\ \text{min})$$
- **c)** $T_i=16$, $T_{LC}=20/4=5$ y $K_p=\dfrac{16}{5\cdot8}=\mathbf{0.4}$. Es decir, $G_c=0.4+\dfrac{0.025}{s}$.

---

### Problema 6: Módulo óptimo (tipo CP-3)
$G_p=\dfrac{2}{(0.2s+1)(3s+1)(0.05s+1)}$ con $K_r=1$. Diseñe por MO, llévelo a paralelo y prediga $M_p$ y $t_s$.

**Solución**
1. $T_u=0.05$ y se compensan 0.2 y 3.
2. $$G_c=\frac{(0.2s+1)(3s+1)}{2(0.05)s\cdot2}=\frac{0.6s^2+3.2s+1}{0.2s}=\mathbf{16+\frac{5}{s}+3s}\ \to\ \textbf{PID}$$
3. Predicción: $M_p\approx4.3\%$ y $t_s\approx8.4T_u=0.42$ s. La simulación da 4.3 % y 0.42 s ✔.
4. Ante la perturbación responde más lento, porque vuelven los polos de 0.2 y 3 s. Con la receta C0: $G_cG_p=\dfrac{1}{0.1s(0.05s+1)}$, entonces

   $$\frac{Y}{R}=\frac{1}{0.005s^2+0.1s+1},\qquad \frac{Y}{D}=\frac{G_p}{1+G_cG_p}=\frac{0.2\,s}{(0.2s+1)(3s+1)(0.005s^2+0.1s+1)}$$

   En $Y/D$ están los polos cancelados $(0.2s+1)(3s+1)$; la $s$ de arriba (integrador del PID) da error cero.

---

### Problema 7: MO vs. MS en planta tipo 1 (tipo CP-3 ej. 2 y CP-4 ej. 2)
$G_p=\dfrac{5}{s(0.1s+1)(s+1)}$. Diseñe por MO y por MS y compare errores ante un escalón en la referencia, una rampa en la referencia y un escalón en la perturbación (a la entrada de la planta).

**Solución** ($T_u=0.1$, $K=5$, se compensa $T=1$):
- **MO:** $G_c=\dfrac{s+1}{2(0.1)(5)}=\mathbf{1+s}$ → **PD**.
- **MS:** $G_c=\dfrac{(0.4s+1)(s+1)}{8(0.01)(5)s}=\dfrac{0.4s^2+1.4s+1}{0.4s}=\mathbf{3.5+\dfrac{2.5}{s}+s}$ → **PID**.

| | Escalón ref. | Rampa ref. | Perturbación escalón | $M_p$ | $t_s$ |
|---|---|---|---|---|---|
| MO | 0 | $2T_u=0.2$ | $y_{ss}=1/P=1$ | 4.3 % | 0.84 |
| MS | 0 | 0 | 0 | 43 % | 1.66 |

**De dónde sale $y_{ss}=1$ del MO** (receta C0): $G_cG_p=\dfrac{1}{0.2s(0.1s+1)}$ (el cero $(s+1)$ canceló el polo de la planta), así que

$$\frac{Y}{D}=\frac{G_p}{1+G_cG_p}=\frac{5}{s(0.1s+1)(s+1)}\cdot\frac{0.2s(0.1s+1)}{0.02s^2+0.2s+1}=\frac{1}{(s+1)(0.02s^2+0.2s+1)}\ \Rightarrow\ Y/D(0)=1$$

La $s$ de la planta se simplifica con la $s$ del lazo y no queda ninguna $s$ arriba: hay error. En el MS, $d_c=0.4s$ pasa al numerador de $Y/D$ y da $Y/D(0)=0$.

**Conclusión:** el MO es mejor para seguir escalones. El MS elimina los errores ante rampa y perturbación a costa del sobrepaso, que se puede reducir con un prefiltro $1/(4T_us+1)$ o derivando la medición.

---

### Problema 8: Cascada vs. lazo único (tipo CP-3 ej. 3, CP-4)
Planta: $u\to\dfrac{1}{0.02s+1}\to\dfrac{4}{0.5s+1}\to m_1\to\dfrac{2}{5s+1}\to y$. Hay sensor en $m_1$ y la perturbación entra en $u$.

**a)** Diseñe un solo controlador por MO. **b)** Diseñe la cascada con ambos lazos por MO. **c)** Compare.

**Solución**
- **a)** $T_u=0.02$ y $K=8$:

  $$G_c=\frac{(0.5s+1)(5s+1)}{2(0.02)(8)s}=\frac{2.5s^2+5.5s+1}{0.32s}=\mathbf{17.19+\frac{3.125}{s}+7.81s}\ (\textbf{PID})$$
- **b)** Cascada:
  - Interno, sobre $\dfrac{4}{(0.02s+1)(0.5s+1)}$ con $T_u=0.02$: $G_{c1}=\dfrac{0.5s+1}{0.16s}=\mathbf{3.125+\dfrac{6.25}{s}}$ (PI).
  - El lazo interno cerrado se aproxima como $\dfrac{1}{0.04s+1}$.
  - Externo, sobre $\dfrac{1}{0.04s+1}\cdot\dfrac{2}{5s+1}$ con $T_u=0.04$: $G_{c2}=\dfrac{5s+1}{0.16s}=\mathbf{31.25+\dfrac{6.25}{s}}$ (PI).
- **c)** Comparación:

| | $M_p$ | $t_s$ | Pico ante perturbación | Recuperación |
|---|---|---|---|---|
| Lazo único (PID) | 4.3 % | 0.17 s | 0.050 | > 20 s (reaparecen 0.5 y 5 s) |
| Cascada (2 PI) | 8.1 % | 0.27 s | **0.009** | **≈ 9 s** |

  La cascada es algo más lenta ante la referencia, pero rechaza la perturbación **5.5 veces mejor** y usa dos PI (sin D).

  **FT ante la perturbación** (receta C0; $G_a=\frac{4}{(0.02s+1)(0.5s+1)}$ hasta $m_1$, $G_b=\frac{2}{5s+1}$):
  - Lazo único: $G_cG_aG_b=\dfrac{1}{0.04s(0.02s+1)}$, así que

    $$\frac{Y}{D}=\frac{G_aG_b}{1+G_cG_aG_b}=\frac{0.32\,s}{(0.5s+1)(5s+1)(0.0008s^2+0.04s+1)}$$

    Reaparecen los polos de 0.5 y 5 s que el PID había cancelado.
  - Cascada: $\dfrac{Y}{D}=\dfrac{G_aG_b}{1+G_aG_{c1}(1+G_{c2}G_b)}$. El término $G_aG_{c1}$ (lazo interno, rápido) no pasa por $G_b$ y ataca la perturbación antes de que llegue a la salida.

---

### Problema 9: ¿Dónde entra la perturbación? (tipo CP-4 ej. 2, pregunta de análisis)
$u\to\dfrac{3}{(\frac13s+1)(2s+1)}\xrightarrow{+d}m_1\to\dfrac4s\to y$.

¿Hay error ante $d$ escalón si se usa **a)** MO en lazo único, **b)** cascada con interno PI (MO) y externo P (MO)?

**Solución**
- **a)** El MO da un PD, $G_c=0.125+0.25s$, sin integrador. El integrador de la planta ($4/s$) está **después** de $d$.
  - $$y_{ss}=\lim_{s\to0}\frac{4/s}{1+G_c(s)\,\frac{3}{(\frac13s+1)(2s+1)}\,\frac4s}=\frac{4}{G_c(0)\cdot3\cdot4}=\frac{4}{0.125\cdot12}=\mathbf{2.67}\ \Rightarrow\ \text{hay error}$$
- **b)** La perturbación entra en $m_1$, que es la variable **medida por el lazo interno**, y el lazo interno tiene un PI. El PI interno la elimina y **no hay error**, aunque el externo sea solo P.
  - Con ecuaciones ($G_i$ = planta interna): $U=G_{c,in}(G_{c,ex}(R-Y)-M_1)$, $M_1=G_iU+d$, $Y=\frac4sM_1$. Con $R=0$:

    $$\frac{Y}{d}=\frac{4/s}{1+G_iG_{c,in}\left(1+G_{c,ex}\frac4s\right)}$$

    Para $s\to0$ el denominador crece como $G_{c,in}G_{c,ex}\frac4s\sim\frac{1}{s^2}$ (integrador del PI interno × integrador de la planta), más rápido que el numerador $\sim\frac1s$. Por eso $Y/d(0)=0$.
- Conclusión: el error ante la perturbación depende de **si hay un integrador en un controlador cuyo lazo contiene el punto de entrada de la perturbación**.

---

### Problema 10: Saturación y anti-windup (análisis, tipo CP-1 y CP-3)
Un PID ajustado por ZN produce 59 % de sobrepaso. Al agregar una saturación del mando de ±3, el sobrepaso sube a 68 %, y al agregar anti-windup baja a 19 %. Explique cada efecto.

**Solución**
- **Saturación sin anti-windup:** mientras el actuador está saturado, el error sigue siendo grande y el integrador sigue acumulando (windup). Cuando la salida cruza la referencia, el integrador tiene un valor excesivo que tarda en "desintegrarse", y eso produce más sobrepaso y más tiempo.
- **Anti-windup por back-calculation:** se realimenta $K_b(u_s-u)$ a la entrada del integrador. Mientras hay saturación, la integral se descarga y queda en un valor coherente con lo que el actuador puede dar. El sobrepaso baja incluso por debajo del caso lineal, porque la acción integral acumulada es menor.

---

## Parte E: Preguntas conceptuales probables (con respuesta)

1. **¿Qué es acción directa o inversa, y por qué importa?**
   El signo de la ganancia del proceso. El controlador se elige con la acción opuesta para que la realimentación sea **negativa**. Si se equivoca, el sistema es inestable.
2. **¿Por qué un P no elimina el error en una planta tipo 0?**
   Porque con $e=0$ la acción P es nula y la planta necesita $u\neq0$ para mantener la salida.
3. **¿Por qué la acción I elimina el error y qué desventaja tiene?**
   La integral mantiene un valor cuando $e=0$ (sube el tipo del lazo). Desventajas: es lenta (un escalón en $e$ da una rampa en $u$), puede volver el sistema oscilatorio y produce windup si hay saturación.
4. **¿Para qué sirve la acción D y cuál es su problema?**
   Amortigua, porque se opone a los cambios. Su problema es que amplifica el ruido; por eso lleva filtro y se aplica sobre la medición para evitar el golpe ante cambios de referencia.
5. **¿Integrador en la planta o en el controlador?**
   Ante la referencia da lo mismo. Ante la perturbación, el integrador debe estar en el controlador, antes del punto de entrada de la perturbación.
6. **¿Qué significa ¼ de razón de decrecimiento?**
   Que cada pico de la oscilación es ¼ del anterior (ZN y CC). Es rápido, pero con sobrepaso alto (≈ 50 %).
7. **ZN vs. CC:**
   - ambos buscan ¼ RD;
   - CC es menos sensible a $L/T$;
   - ZN da un PID serie y CC uno ideal;
   - ZN en lazo cerrado necesita llevar la planta a oscilación sostenida, lo que puede ser peligroso o impracticable.
8. **¿Cuándo ISE, IAE o ITAE?**
   - ISE penaliza los errores grandes (iniciales) y tiende a respuestas oscilatorias;
   - IAE pondera todos los errores por igual;
   - ITAE e ITSE penalizan los errores que persisten.
   
   El ajuste depende de si es para la referencia o para la perturbación.
9. **¿Qué controlador según el proceso?**
   - P: nivel y presión (plantas integradoras, basta con P);
   - PI: flujo (medición ruidosa, sin D);
   - PID: temperatura y composición (dinámica lenta, multicapacitiva o con retardo).
10. **¿Por qué el PI por cancelación es lento ante perturbaciones?**
    Porque la cancelación solo ocurre en el camino de la referencia. Ante la perturbación reaparece el polo de la planta. Además, es sensible a los errores en $T$.
11. **Objetivos del MO y cómo se logran:**
    - compensar las constantes lentas, con ceros en el controlador;
    - error cero ante escalón, con integrador en el lazo;
    - 2º orden con $\zeta=0.707$ ($M_p$ 4 %), óptimo ITAE, con $a=2$.
12. **Objetivos del MS y cómo se logran:**
    - compensar las constantes lentas;
    - −40 dB/dec en bajas frecuencias (dos integradores) para llegar rápido al estado estacionario y tener error cero ante rampa y perturbación;
    - cruce por 0 dB a −20 dB/dec para tener buena estabilidad relativa.
13. **¿Por qué MS casi solo en plantas tipo 1?**
    En una planta tipo 0, el controlador tendría que aportar dos integradores ($1/s^2$). En ese caso conviene más el mando subordinado.
14. **¿Por qué MO, MS y cancelación no sirven con retardo?**
    Porque requieren $G_p^{-1}$, y $e^{+Ls}$ no es realizable (sería predecir el futuro). En ese caso se usa ZN, CC o criterios integrales (o el predictor de Smith, Tema 2).
15. **Ventajas y desventajas de la cascada:**
    - Ventajas: reguladores más simples (PI); las perturbaciones que entran en el lazo interno se corrigen antes de llegar a la salida; se puede limitar la variable intermedia.
    - Desventajas: más sensores y controladores; el lazo externo es algo más lento ($T_u$ doble); el lazo interno debe ser bastante más rápido que el externo.
16. **¿Por qué el lazo interno de la cascada se aproxima como $1/(2T_us+1)$?**
    El MO lo deja en $1/(2T_u^2s^2+2T_us+1)$. Si $T_u$ es pequeño, el término $2T_u^2s^2$ es despreciable.

---

## Parte F: Errores que más restan puntos

1. **No pasar a la forma de constantes de tiempo** antes de leer $K$ y $T$. Por ejemplo, en $\dfrac{100}{s(s+10)}$ la constante es 0.1, no 10.
2. **Elegir mal $T_u$:** es la **menor** constante de tiempo, no la mayor.
3. **Mezclar PID serie e ideal:** ZN es serie. Hay que convertir antes de dar P, I, D.
4. **Medir $t_{28}$ y $t_{63}$ desde donde empieza a moverse la curva**, en vez de desde el escalón.
5. **Usar $K_u$ en dB** (salida de `margin` en dB). Debe ir en veces.
6. **Confundir las fórmulas de $T_I$** de los criterios integrales para la referencia y para la perturbación.
7. **Decir "hay integrador, entonces no hay error"** sin mirar si está en la planta o en el controlador, y dónde entra la perturbación.
8. **No verificar el mando** cuando el enunciado da un límite.
9. **Olvidar identificar el controlador** (P, PI, PD, PID) y **dejar de concluir.** Casi siempre piden "compare y comente".
10. **Aplicar MO o MS a una planta con retardo.**

---

## Parte G: Estrategia el día del certamen

- **Primero** los problemas de fórmula directa: PORT, ZN, MO/MS de plantas conocidas. Son puntos seguros.
- En cada problema, escribir los datos en forma de constantes de tiempo en una línea (**K, T's, tipo, retardo**) antes de calcular.
- **Dejar explícito el razonamiento:** "$T_u=\dots$ por ser la menor", "PID serie → convierto a ideal", "la perturbación entra antes del integrador del controlador → $e_{ss}=0$".
- **Chequeos rápidos de coherencia:**
  - $K_c$ sale positivo si $K>0$;
  - $L/T$ está en rango;
  - en MO, $M_p\approx4\%$;
  - en MS, $M_p\approx43\%$;
  - en cascada, el externo es más lento que el interno.
- Si falta tiempo, plantear el procedimiento con las fórmulas aunque no se termine el número.

---

**Material de apoyo en este repositorio:**
- `Resumen_Certamen1_Conf1-3.md`: la materia y el formulario;
- `CP1/`: ZN, CC y criterios integrales, más el PID práctico;
- `CP2/`: linealización y PI para 1er orden;
- `CP3/`: MO, MS y cascada, con saturación;
- `CP4/`: plantas completas, ZN con retardo y cascada.
