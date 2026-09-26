# Diagramas Simulink de los prácticos CP-1 a CP-4: bloques y parámetros

> Cada modelo trae: **diagrama**, **tabla de bloques** (ruta en el *Library Browser* y parámetros exactos), **configuración de la simulación** y **qué graficar**. Los valores numéricos son los de las soluciones de `CP1/`…`CP4/`.
> En los campos de Simulink se pueden escribir expresiones MATLAB, por ejemplo `conv([8.34 1],[0.502 1])`.
> Los modelos no se probaron en Simulink (no hay MATLAB en este entorno). Los resultados esperados vienen de la simulación equivalente en Python.

## Índice
- [0. Convenciones y bloques comunes](#0-convenciones-y-bloques-comunes)
- [CP-1: PID por ZN, CC y CI + PID práctico](#cp-1-pid-por-zn-cc-y-ci--pid-práctico)
- [CP-2: Mezclador y PI para planta de 1er orden](#cp-2-mezclador-y-pi-para-planta-de-1er-orden)
- [CP-3: Módulo óptimo, simétrico y mando subordinado](#cp-3-módulo-óptimo-simétrico-y-mando-subordinado)
- [CP-4: Control de plantas](#cp-4-control-de-plantas)
- [Script para graficar los resultados](#script-para-graficar-los-resultados)

---

## 0. Convenciones y bloques comunes

**Rutas de los bloques** (Library Browser → *Simulink*):

| Bloque | Ruta |
|---|---|
| Step | Sources |
| Ramp | Sources |
| Constant | Sources |
| Sum | Math Operations |
| Gain | Math Operations |
| Transfer Fcn | Continuous |
| Integrator | Continuous |
| Transport Delay | Continuous |
| PID Controller | Continuous |
| PID Controller (2DOF) | Continuous |
| Saturation | Discontinuities |
| Fcn (MATLAB Fcn / Fcn) | User-Defined Functions |
| Mux | Signal Routing |
| Scope | Sinks |
| To Workspace | Sinks |

**Bloque PID Controller: dónde está cada parámetro**
- *Main*:
  - Controller = PID / PI / PD / P.
  - Form = **Parallel**.
  - Time domain = Continuous.
  - Campos P, I, D y *Filter coefficient (N)*.
- *Output Saturation* (en versiones antiguas, pestaña *PID Advanced*):
  - ☑ *Limit output*, *Upper limit* y *Lower limit*.
  - *Anti-windup method* = back-calculation, con la *Back-calculation coefficient (Kb)*.
- **Derivada sobre la medición:** usar **PID Controller (2DOF)** con *Setpoint weight* **b = 1** y **c = 0**. Ese bloque tiene dos entradas: `Ref` y `y` (la medición).

**Convenciones de todos los modelos**
- **Sum del error:** *List of signs* `|+-` (la referencia entra por `+` y la medición por `−`).
- **Sum de la perturbación:** `|++`.
- **Escalón de referencia** en t = 1. **Escalón de perturbación** aplicado cuando la salida ya se asentó.
- **To Workspace:** *Save format* = **Array**, con los nombres `r`, `y`, `u`, `e`.
- **Model Settings** (Ctrl+E): *Solver* = auto (ode45). Con retardos o dinámicas rápidas, fijar *Max step size*.

**Cómo leer los diagramas**

| Símbolo | Significado |
|---|---|
| `[ Nombre ]` | Un bloque de Simulink |
| `──►` | Línea de señal, en el sentido de la flecha |
| `( Σ )` | Bloque Sum. Los signos junto a sus entradas (`+`, `−`) indican cómo entra cada señal |
| `▲` / `▼` | Una señal que entra al Σ desde abajo o desde arriba (realimentación o perturbación) |
| `┬` / `┌ ┘ └` | Punto de derivación: la misma señal se lleva a otro lugar (clic derecho y arrastrar desde una línea) |
| Texto sobre una línea (`e`, `u`, `C`…) | Nombre de esa señal. Se puede escribir en Simulink con doble clic sobre la línea |

**Salidas mínimas de cada modelo** (lo que piden las guías):
1. Referencia y salida: *Mux* de 2 entradas → *Scope*.
2. Acción de control `u` → *Scope*.
3. Error `e` → *Scope*.

---

# CP-1: PID por ZN, CC y CI + PID práctico

Planta (Smith & Corripio, ej. 6-1.1). El tiempo está en **minutos**.

## Modelo CP1-A: lazo simple para comparar los 7 controladores (pasos 5–11)

```
                                                       [Step F] ──►[ Gf ]────────┐
                                                                                 ▼ +
 [Step R]──►( Σ )── e ──►[ PID Controller ]── m ──►[ Gv ]── w ──►[ Gs ]──────►( Σ )── T ──►[ H ]──┬── C ──►[Mux]──►[Scope]
            + ▲ −                                                                  +                   │
              └────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| R | Referencia (set point) de la temperatura, en %TO. Escalón de 1 en t = 1 |
| F | Perturbación: cambio en el caudal de alimentación del tanque (ft³/min) |
| e | Error = R − C (lo que ve el controlador) |
| m | Salida del controlador (%CO): la señal que va a la válvula |
| Gv | Válvula de vapor: convierte m en caudal de vapor w |
| w | Caudal de vapor (lb/min) |
| Gs | Proceso: efecto del vapor sobre la temperatura del tanque |
| Gf | Proceso: efecto de la alimentación F sobre la temperatura (ganancia negativa: más alimentación fría → baja T) |
| T | Temperatura real del tanque (°F), suma de ambos efectos |
| H | Sensor-transmisor de temperatura |
| C | Temperatura **medida** (%TO) = salida de H. Es la señal que se realimenta al Σ del error |

| Bloque | Tipo | Parámetros |
|---|---|---|
| Step R | Step | Step time = 1, Initial = 0, Final = 1 |
| Step F | Step | Step time = 40, Initial = 0, Final = 1 |
| Σ error | Sum | `|+-` |
| PID | PID Controller | Form = Parallel; P, I, D según la tabla de abajo; N = 100 |
| Gv | Transfer Fcn | Num = `1.652`, Den = `[0.2 1]` |
| Gs | Transfer Fcn | Num = `1.183`, Den = `conv([8.34 1],[0.502 1])` |
| Gf | Transfer Fcn | Num = `-3.34*[0.524 1]`, Den = `conv([8.34 1],[0.502 1])` |
| Σ salida | Sum | `|++` (Gs arriba, Gf abajo) |
| H | Transfer Fcn | Num = `1`, Den = `[0.75 1]` |

**Configuración:** Stop time = 80 (min).

**P, I, D para cada controlador** (todos en forma paralela: $P=K_c$, $I=K_c/T_i$, $D=K_cT_d$, a partir de la forma ideal):

| Controlador | $K_c$ | $T_i$ | $T_d$ | **P** | **I** | **D** |
|---|---|---|---|---|---|---|
| ZN-Ku (serie→ideal) | 8.145 | 2.890 | 0.462 | **8.145** | **2.818** | **3.766** |
| ZN-PORT (serie→ideal) | 4.339 | 3.698 | 0.591 | **4.339** | **1.173** | **2.566** |
| CC (ideal) | 3.984 | 3.392 | 0.521 | **3.984** | **1.175** | **2.076** |
| IAE-ref | 2.504 | 11.649 | 0.597 | **2.504** | **0.215** | **1.495** |
| IAE-pert | 3.620 | 2.601 | 0.562 | **3.620** | **1.392** | **2.034** |
| ITAE-ref | 2.171 | 10.847 | 0.515 | **2.171** | **0.200** | **1.118** |
| ITAE-pert | 3.581 | 2.765 | 0.568 | **3.581** | **1.295** | **2.034** |

> **¿Dónde van $K_c$, $T_i$, $T_d$?** En ninguna parte del bloque si se usa *Form = Parallel*. Son valores intermedios (forma ideal) que sirven para **calcular** P, I y D. En el bloque solo se escriben **P, I, D y N** (las columnas en negrita). Si se prefiere escribirlos directamente, se cambia *Form = Ideal*. Ahí el bloque es $P(1+I\frac1s+D\,s)$, así que va **P = $K_c$**, **I = $1/T_i$** y **D = $T_d$**. Por ejemplo, para ZN-Ku: P = 8.145, I = 1/2.890 = 0.346, D = 0.462.

> Para ver el efecto de usar ZN "en la forma equivocada", también se puede poner el PID en *Form = Ideal*: P = $K'_c$, I = $1/T'_i$, D = $T'_d$, con los valores serie (ZN-Ku: 6.516, 1/2.312, 0.578).

## Modelo CP1-B: PID práctico (paso 14): saturación ±3, D sobre la medición, filtro y anti-windup

### Opción A: con un solo bloque (PID Controller 2DOF)
```
                                                               [Step F]──►[ Gf ]─────┐
                                                                                     ▼+
 [Step R]──►Ref┐                                                                          
               [ PID Controller (2DOF) ]── u ──►[ Gv ]──►[ Gs ]─────────────────────►( Σ )── T ──►[ H ]──┬── C
       C ──►y  ┘   (saturación y antiwindup                                          +                   │
       ▲            internos al bloque)                                                                  │
       └─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| R, F, T, C | Iguales que en CP1-A |
| Ref | Entrada 1 del PID 2DOF: la referencia R |
| y | Entrada 2 del PID 2DOF: la medición C (con c = 0 la acción D se calcula solo con ella) |
| u | Salida del PID ya saturada (±3): es el mando m que va a la válvula |

> **¿Qué es C?** Es la **temperatura medida**: la salida del sensor-transmisor $H(s)=\frac{1}{0.75s+1}$, en %TO. No es un bloque. Es la **línea** que sale de H y vuelve a la entrada `y` del PID 2DOF (el camino de realimentación). La salida del proceso es T, y C es lo que el controlador "ve" de T a través del sensor. Es la misma C de $C(s)=\frac{G_cG_1}{1+G_cG_1}R(s)+\frac{G_2}{1+G_cG_1}F(s)$ de la guía CP-1.

| Bloque | Parámetros |
|---|---|
| PID Controller (2DOF) | Form = Parallel; **P = 8.1449, I = 2.8182, D = 3.7663, N = 20**; **b = 1, c = 0** (D sobre la medición); Output saturation: ☑ Limit output, **Upper = 3, Lower = −3**; Anti-windup = **back-calculation, Kb = 0.865** |
| Resto | Igual que en CP1-A |

### Opción B: con bloques sueltos (para agregar las modificaciones una a una)
```
 R ──►( Σ )── e ──┬──►[Gain P=8.1449]──────────────────────────────────┐
      + ▲ −       │                                                     ▼+
        │         └──►[Gain I=2.8182]──►( Σ )──►[Integrator 1/s]──ui──►( Σ )── u ──►[Saturation ±3]──┬── us ──► a Gv
        │                               ++▲                             +▲ +                          │
        │                                 └──[Gain Kb=0.865]◄──( Σ )◄────┼───────── us ───────────────┘
        │                                                      −▲ +      │
        │                                                       u        │
        │   C ──►[Gain −1]──►[ Transfer Fcn  D·N·s/(s+N) ]── ud ─────────┘
        └──────── C
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| e | Error R − C |
| up = P·e | Acción proporcional |
| ui | Acción integral (salida del Integrator) |
| ud | Acción derivativa filtrada, calculada sobre −C (no sobre e) para evitar el golpe al cambiar R |
| u | Mando calculado = up + ui + ud (antes de saturar) |
| us | Mando real, ya saturado a ±3: es lo que entra a Gv |
| us − u | Diferencia por saturación. Multiplicada por Kb se resta al integrador (anti-windup): si no hay saturación vale 0 |
| Kb | Ganancia de back-calculation (anti-windup) |

| Bloque | Tipo | Parámetros |
|---|---|---|
| Gain P | Gain | 8.1449 |
| Gain I | Gain | 2.8182 |
| Integrator | Integrator | Initial condition = 0 |
| D filtrado | Transfer Fcn | Num = `[3.7663*20 0]`, Den = `[1 20]` (entrada: **−C**, es decir, D sobre la medición) |
| Σ u | Sum | `|+++` (up + ui + ud) |
| Saturation | Saturation | Upper = 3, Lower = −3 |
| Σ antiwindup | Sum | `|+-` (us − u) |
| Gain Kb | Gain | 0.865 |
| Σ integrador | Sum | `|++` (I·e + Kb·(us − u)) |

**Casos a simular** (para evaluar cada modificación):
1. Lineal: sin Saturation (Upper/Lower = ±Inf), D sobre `e`, N = 1000, Kb = 0.
2. \+ saturación ±3.
3. \+ N = 20.
4. \+ D sobre la medición.
5. \+ Kb = 0.865.

**Resultados esperados** (escalón de R): $M_p$ ≈ 59 % → 68 % → 68 % → 68 % → **19 %**.

**Configuración:** Stop time = 80. R en t = 1 y F en t = 40.

---

# CP-2: Mezclador y PI para planta de 1er orden

## Modelo CP2-1: mezclador no lineal (para contrastar la linealización, ej. 1g)

$$10\,\dot x_3 = 0.6F_1 + x_2 - x_3(F_1+1),\qquad F_3=F_1+1$$

```
 [Step F1: 3→3.3]──┬─────────────────────►┌───────────────┐
                   │                       │ Fcn:          │    ┌──────────────┐
 [Step x2: 0.4→0.44]──────────────────────►│ (0.6*u(1)+u(2)│───►│ Integrator   │──┬── x3 ──►[Scope]
                   │                ┌─────►│ -u(3)*(u(1)+1)│    │ x3(0) = 0.55 │  │
                   │                │      │ )/10          │    └──────────────┘  │
                   │                │      └───────────────┘                      │
                   │                └─────────────────────────────────────────────┘
                   └──►[Bias +1]──► F3 ──►[Scope]
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| F1 | Caudal de entrada 1 (cm³/min), el manipulable (tiene válvula). Operación: 3 |
| x2 | Composición de la corriente 2, perturbación (la fija el proceso anterior). Operación: 0.4 |
| x3 | Composición de salida del mezclador (salida a controlar). Operación: 0.55 |
| F3 | Caudal de salida = F1 + F2 = F1 + 1. Operación: 4 |
| Fcn | Calcula dx3/dt a partir del balance de masa parcial |
| Integrator | Integra dx3/dt para obtener x3. La condición inicial 0.55 es el punto de operación |

| Bloque | Tipo | Parámetros |
|---|---|---|
| Step F1 | Step | Time = 1, Initial = 3, Final = 3.3 (para el escalón de x2: Final = 3) |
| Step x2 | Step | Time = 1, Initial = 0.4, Final = 0.4 (para su escalón: Final = 0.44) |
| Mux | Mux | 3 entradas: [F1, x2, x3] |
| Fcn | Fcn | `(0.6*u(1) + u(2) - u(3)*(u(1)+1))/10` |
| Integrator | Integrator | **Initial condition = 0.55** (punto de operación) |
| F3 | Bias | Bias = 1 (o Sum con un Constant 1) |

**Configuración:** Stop time = 25 (min).

**Resultados esperados:** con F1 = 3.3, x3 → 0.5535 (el lineal da 0.55375). Con x2 = 0.44, x3 → 0.56.

## Modelo CP2-2: control de x3 con PI (ej. 2b), diseños A y B

```
                                                             [Step d = 0.01]──►[ 1/(10s+4) ]──┐
                                                                                              ▼+
 [Step r=0.02]──►[ Prefiltro ]──►( Σ )── e ──►[ PID (PI) ]── u ──►[Saturation ±2]──►[ 0.05/(10s+4) ]──►( Σ )──┬── x3
                  (solo en B)    + ▲ −                       │                                         +       │
                                   └─────────────────────────┼─────────────────────────────────────────────────┘
                                                             └──►[Scope u]  (verificar |u| ≤ 2)
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de **variación** de x3: escalón de 0.02 (se trabaja en desviaciones respecto del punto de operación) |
| Prefiltro | 1/(Ti·s+1): suaviza la referencia para cancelar el cero del PI (solo en el diseño B) |
| e | Error = referencia (filtrada) − x3 |
| u | Variación del caudal F1 que ordena el PI (ΔF1). Debe quedar dentro de ±2 |
| Saturation | Límite físico del mando (±2). Se pone para verificar que el diseño no lo alcanza |
| 0.05/(10s+4) | Planta: efecto de ΔF1 sobre Δx3 |
| d | Perturbación: variación de x2, escalón de 0.01 |
| 1/(10s+4) | Efecto de Δx2 sobre Δx3 |
| x3 | Variación de la composición de salida (Δx3), la variable controlada |

| Bloque | Tipo | Diseño A (cancelación) | Diseño B (2º orden + prefiltro) |
|---|---|---|---|
| Step r | Step | Time = 1, Final = 0.02 | igual |
| Prefiltro | Transfer Fcn | *(no va, o Num = 1, Den = 1)* | Num = `1`, Den = `[1.222 1]` |
| PID | PID Controller, Controller = **PI** | **P = 100, I = 40** | **P = 88.42, I = 72.36** |
| Saturation | Saturation | ±2 (solo para verificar; no debería actuar) | ±2 |
| Planta | Transfer Fcn | Num = `0.05`, Den = `[10 4]` | igual |
| Gd | Transfer Fcn | Num = `1`, Den = `[10 4]` | igual |
| Step d | Step | Time = 30, Final = 0.01 | igual |

**Configuración:** Stop time = 60 (min).

**Resultados esperados:**
- **A:** $t_s$ ≈ 7.8 min, sin sobrepaso, $u$ parte en 2.0 y baja a 1.6. Ante la perturbación: pico ≈ 0.00082 y $t_s$ ≈ 15 min.
- **B:** $t_s$ ≈ 9.9 min, $M_p$ ≈ 4.6 %, $u_{max}$ ≈ 1.99. Ante la perturbación: pico ≈ 0.00076 y $t_s$ ≈ 12 min.

---

# CP-3: Módulo óptimo, simétrico y mando subordinado

## Modelo CP3-2: ej. 2, MO vs. MS (escalón y rampa en la referencia, escalón en la perturbación)

```
 [Step r]──┐                                                      [Step d]──┐
           ├─►[Manual Switch]──►( Σ )── e ──►[ PID ]── u ──►( Σ )──►[ Planta Gp ]──┬── y
 [Ramp r]──┘                    + ▲ −                       + ▲+                   │
                                  └────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia: escalón (Step) o rampa (Ramp), según el Manual Switch |
| e | Error r − y |
| u | Salida del controlador MO o MS |
| d | Perturbación que se suma a la **entrada de la planta** (en el mando) |
| Gp | Planta Gp1 = 100/(s(s+10)) o Gp2 = 2/(s(0.8s+1)(s+0.5)) |
| y | Salida de la planta (realimentación unitaria, Kr = 1) |

| Bloque | Parámetros |
|---|---|
| Step r | Time = 1, Final = 1 |
| Ramp r | Slope = 1, Start time = 1 |
| Manual Switch | Elige el escalón o la rampa (doble clic para cambiar) |
| Σ perturbación | `|++`, entre el PID y la planta (d a la entrada de la planta, como `feedback(Gp,Gc)`) |
| Step d | Time = 10 (Gp1) / 60 (Gp2), Final = 1 |

**Planta y controladores:**

| Caso | Planta (Transfer Fcn) | MO: PID Controller | MS: PID Controller |
|---|---|---|---|
| **Gp1** | Num `100`, Den `[1 10 0]` | Controller = **P**, **P = 0.5** | Controller = **PI**, **P = 0.5, I = 1.25** |
| **Gp2** | Num `2`, Den `conv([1 0],conv([0.8 1],[1 0.5]))` | Controller = **PD**, **P = 0.15625, D = 0.3125, N = 100** | Controller = **PID**, **P = 0.2539, I = 0.04883, D = 0.3125, N = 100** |

**Configuración:**
- Gp1: Stop time = 20.
- Gp2: Stop time = 120.

**Resultados esperados:**
- MO: $M_p$ ≈ 4.3 %, error ante rampa = 0.2 (Gp1) / 1.6 (Gp2), $y_{ss}$ ante perturbación = 2 (Gp1) / 6.4 (Gp2).
- MS: $M_p$ ≈ 43 %, y todos los errores en 0.

## Modelo CP3-3a: ej. 3, un solo PID por MO (con saturación ±10 y anti-windup)

```
                                                              [Step d = +1, t = 1]──┐
                                                                                    ▼+
 [Step r]──►( Σ )── e ──►[ PID Controller ]── u ──►( Σ )──►[1/(0.01s+1)]──►[1/(0.1s+1)]── V ──►[1/(0.1s+1)]──┬── Y
            + ▲ −          (sat ±10, Kb)            +                                                        │
              └──────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de Y (escalón 1) |
| e | Error r − Y |
| u | Acción de control, saturada a ±10 dentro del PID |
| d | Perturbación de +1 sumada a la acción de control (entrada de 1/(0.01s+1)) |
| 1/(0.01s+1) | Primer bloque de la planta (el más rápido: Tu = 0.01) |
| V | Variable intermedia, salida del 2º bloque. Aquí no se mide (sí en la cascada) |
| Y | Salida final a controlar |

| Bloque | Parámetros |
|---|---|
| Step r | Time = 0, Final = 1 |
| PID Controller | PID, Parallel: **P = 10, I = 50, D = 0.5, N = 1000**; Limit output **±10**; anti-windup **back-calculation, Kb = I/P = 5** |
| Σ d | `|++`, **después** del PID (perturbación en la acción de control) |
| Step d | Time = 1, Final = 1 |
| Transfer Fcn ×3 | `1/[0.01 1]`, `1/[0.1 1]`, `1/[0.1 1]` |

**Configuración:** Stop time = 2 s, *Max step size* = 1e-4.

**Casos a simular:**
1. Sin límite.
2. ±10 con Kb = 0 (sin anti-windup).
3. ±10 con Kb = 5.

> **N = 1000, no 200.** Con la fórmula de la guía (N = P/(0.1·D) = 200) el filtro queda lento respecto de $T_u$ = 0.01 y el sobrepaso sube de 4 % a ≈ 15 %.

## Modelo CP3-3b: ej. 3, mando subordinado (cascada)

```
                                                                                [Step d]──┐
                                                                                          ▼+
 [Step r]──►( Σ )──►[ PID externo ]── Vref ──►( Σ )──►[ PID interno ]── u ──►( Σ )──►[1/(0.01s+1)]──►[1/(0.1s+1)]──┬── V ──►[1/(0.1s+1)]──┬── Y
            + ▲ −                             + ▲ −    (sat ±10, Kb)          +                                    │                      │
              │                                 └──────────────────────────────────────────────────────────────────┘ (sensor en V)        │
              └───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de Y |
| PID externo | Controla Y. Su salida **Vref** es la referencia del lazo interno |
| Vref | Valor que se le pide a la variable intermedia V |
| PID interno | Controla V (sensor en V). Su salida u es el mando real (±10) |
| u, d, V, Y | Iguales que en CP3-3a |

| Bloque | Parámetros |
|---|---|
| PID externo | Controller = **PI**: **P = 2.5, I = 25** (sin límite) |
| PID interno | Controller = **PI**: **P = 5, I = 50**; Limit output **±10**; back-calculation **Kb = 10** |
| Resto | Igual que CP3-3a |

**Resultados esperados:**

| Caso | $M_p$ | $t_s$ | $\lvert u\rvert_{max}$ | Perturbación: desviación | Perturbación: $t_s$ |
|---|---|---|---|---|---|
| Cascada, sin límite | 8 % | 0.13 s | 13 | 0.043 | 0.11 s |
| Cascada, ±10 con anti-windup | 10 % | 0.15 s | 10 | 0.043 | 0.11 s |
| PID único, ±10 con anti-windup | 2.4 % | 0.40 s | 10 | 0.074 | 0.38 s |

## Modelo CP3-4: ej. 4, cascada con planta integradora (Gc1 MO, Gc2 MO o MS)

```
                                                                               [Step d=1000, t=5]──┐
                                                                                                   ▼+
 [Step r]──►( Σ )──►[ Gc2 ]── Vref ──►( Σ )──►[ Gc1 ]── u ──►( Σ )──►[1/(0.1s+1)]──►[1/(0.01s+1)]──┬── V ──►[1/(2s+1)]──►[1/s]──┬── C
            + ▲ −                     + ▲ −   (±5000, Kb)     +                                    │                            │
              │                         └──────────────────────────────────────────────────────────┘                            │
              └─────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de la salida C |
| Gc2 | Controlador externo (MO → PD o MS → PID). Su salida Vref es la referencia del interno |
| Gc1 | Controlador interno (PI por MO). Su salida u es el mando real, limitado a ±5000 |
| d | Perturbación de 1000 en la acción de control (t = 5) |
| V | Variable intermedia medida (salida de 1/(0.01s+1)), realimentada al lazo interno |
| 1/(2s+1) · 1/s | Parte lenta de la planta, con el integrador |
| C | Salida final, realimentada al lazo externo |

| Bloque | Parámetros |
|---|---|
| **Gc1** (interno, MO) | PI: **P = 5, I = 50**; Limit output **±5000**; back-calculation **Kb = 10** |
| **Gc2 por MO** | Controller = **PD**: **P = 25, D = 50, N = 1000** |
| **Gc2 por MS** | Controller = **PID**: **P = 650, I = 312.5, D = 50, N = 1000** |
| 1/s | Integrator (o Transfer Fcn Num `1`, Den `[1 0]`) |
| Step d | Time = 5, Final = 1000 (perturbación en la acción de control) |

**Configuración:** Stop time = 10 s, *Max step size* = 1e-4.

**Resultados esperados:**
- **Gc2 MO:** referencia con $M_p$ ≈ 10 %, $t_s$ ≈ 0.14 s. Ante la perturbación la desviación es ≈ 0.34 y la recuperación lenta (≈ 6 s).
- **Gc2 MS:** $M_p$ ≈ 57 % (sin límite) → ≈ 10 % con ±5000 y anti-windup. Ante la perturbación la desviación es ≈ 0.19 y la recuperación rápida (≈ 0.3 s).

## Subsistema "PID de la guía CP-3" (reutilizable)

```
 SP ──►( Σ )── e ─┬──►[Gain P]──────────────────────────────────┐
       + ▲ −      │                                              ▼+
 med ────┤        └──►[Gain I]──►( Σ )──►[1/s]──── ui ─────────►( Σ )── u ──►[Saturation]──┬──► U (salida)
         │                       + ▲+                            +▲ +                      │
         │                         └──[Gain Kb]◄──( Σ )◄─────────┼──── us ─────────────────┘
         │                                        −▲ +           │
         │                                         u             │
         └──►[Gain −D]──►( Σ )──►[Gain N]──┬── ud ───────────────┘
                         + ▲ −             │
                           └────[1/s]◄─────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| SP | Set point (referencia) que recibe el subsistema |
| med | Medición de la variable controlada |
| e | Error SP − med (solo alimenta las acciones P e I) |
| ui | Salida del integrador (acción integral) |
| ud | Acción derivativa filtrada, calculada sobre −med |
| u | Suma P + ui + ud, antes de saturar |
| us / U | Mando saturado = salida del subsistema |
| us − u | Diferencia que, multiplicada por Kb, descarga el integrador cuando hay saturación (anti-windup) |
| N | Coeficiente del filtro de la derivada. La constante del filtro 1/N debe ser menor que Tu |

- Se crea como *Subsystem* con **In1 = SP**, **In2 = med** y **Out1 = U**.
- Los parámetros P, I, D, N, Kb y los límites se pasan como variables del workspace, o con una *Mask* (clic derecho → Mask → Create Mask).
- Sirve para todos los PID de CP-3 si se quiere implementar "a mano".

---

# CP-4: Control de plantas

## Modelo CP4-1ID: identificación (a1 y b3)

```
 [Step man: 0→1, t=1]──►[ Gv ]──►( Σ )──►[ G1 ]──┬── med1          [Constant 0]──► dist
                                  ▲+              └──►[ G2 ]──►[Transport Delay 1]── med2 ──►[To Workspace "y"]
                        dist ─────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| man | Entrada manipulable de la planta (mando). Aquí se aplica un escalón de 1 en lazo abierto |
| dist | Entrada de perturbación. En la identificación se deja en 0 (Constant 0) |
| Gv | Actuador (válvula), 2/(s+2) |
| G1, G2 | Etapas del proceso |
| med1 | Medición intermedia (salida de G1) |
| med2 | Medición de la salida final (después de G2 y del retardo de 1 s) |
| y | Nombre de la variable To Workspace donde se guarda med2 para calcular K, T y L |

Luego:
```matlab
[K,T,L] = port_datos(out.tout, out.y, 1, 1)
```
Para el lazo externo (b3), identificar con el lazo interno cerrado (ver CP4-1b), con un escalón en la referencia del interno.

## Modelo CP4-1a: PID por ZN, lazo único sobre med2

```
                                              [Step dist=1, t=60]──┐
                                                                   ▼+
 [Step r]──►( Σ )── e ──►[ PID Controller ]── man ──►[ Gv ]──►( Σ )──►[ G1 ]── med1 ──►[ G2 ]──►[Transport Delay]──┬── med2
            + ▲ −                                                  +                                                │
              └─────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de med2 |
| e | Error r − med2 |
| man | Salida del PID = mando que entra a Gv |
| dist | Perturbación: se suma entre Gv y G1 (escalón de 1 en t = 60) |
| med1 | Salida de G1 (en el lazo único no se usa) |
| Transport Delay | Retardo puro de 1 s (el e^−s del enunciado) |
| med2 | Salida medida a controlar, realimentada al Σ del error |

| Bloque | Parámetros |
|---|---|
| Gv | Transfer Fcn: Num `2`, Den `[1 2]` |
| G1 | Transfer Fcn: Num `5`, Den `[2 1]` |
| G2 | Transfer Fcn: Num `3`, Den `[5 1]` |
| Transport Delay | Time delay = **1** |
| PID Controller | PID, Parallel: **P = 0.3025, I = 0.0498, D = 0.2941, N = 100** |
| Step r | Time = 1, Final = 1 |
| Step dist | Time = 60, Final = 1 |

**Configuración:** Stop time = 120 s, *Max step size* = 0.01.

**Resultados esperados:** $M_p$ ≈ 52 %, $t_s$ ≈ 25 s. Ante la perturbación: pico ≈ 3.7.

## Modelo CP4-1b: cascada, PI interno por MO (med1) + PID externo por ZN (med2)

```
                                                                            [Step dist]──┐
                                                                                         ▼+
 [Step r]──►( Σ )──►[ PID externo ]── r1 ──►( Σ )──►[ PI interno ]── man ──►[ Gv ]──►( Σ )──►[ G1 ]──┬── med1 ──►[ G2 ]──►[Delay 1]──┬── med2
            + ▲ −                           + ▲ −                                        +           │                                │
              │                               └──────────────────────────────────────────────────────┘                                │
              └───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de med2 |
| PID externo | Controla med2. Su salida r1 es la referencia del lazo interno |
| r1 | Valor que se le pide a med1 |
| PI interno | Controla med1. Su salida man es el mando real |
| med1 | Medición intermedia, realimentada al lazo interno. La perturbación entra **dentro** de este lazo |
| med2, dist | Iguales que en CP4-1a |

| Bloque | Parámetros |
|---|---|
| PI interno | Controller = **PI**: **P = 0.4, I = 0.2** |
| PID externo | PID, Parallel: **P = 1.2264, I = 0.2687, D = 0.8956, N = 100** |
| Resto | Igual que CP4-1a |

**Resultados esperados:** $M_p$ ≈ 53 %, $t_s$ ≈ 21 s. Ante la perturbación: pico ≈ 1.33 (≈ 2.8 veces menor que el lazo único).

## Modelo CP4-2a: MS en lazo único sobre med2

```
                                                                    [Step dist=1, t=15]──┐
                                                                                         ▼+
 [Step r]──►( Σ )── e ──►[ PID Controller ]── man ──►[ Gv ]──►[ G1 ]──────────────────►( Σ )──┬── med1 ──►[ G2 = 4/s ]──┬── med2
            + ▲ −                                                                        +                               │
              └──────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de med2 |
| e | Error r − med2 |
| man | Mando (salida del PID), entra a Gv |
| dist | Perturbación: se suma **después de G1** (en med1), antes del integrador 4/s |
| med1 | Salida de G1 + perturbación (no se usa en el lazo único) |
| G2 = 4/s | Integrador del proceso: la planta es tipo 1 |
| med2 | Salida a controlar |

| Bloque | Parámetros |
|---|---|
| Gv | Transfer Fcn: Num `3`, Den `[1 3]` |
| G1 | Transfer Fcn: Num `3`, Den `[2 1]` |
| G2 | Transfer Fcn: Num `4`, Den `[1 0]` (o Gain 4 → Integrator) |
| PID (MS) | PID, Parallel: **P = 0.3125, I = 0.09375, D = 0.25, N = 100** |
| PID (MO, comparación) | Controller = **PD**: **P = 0.125, D = 0.25, N = 100** |
| Step dist | Time = 15, Final = 1 (se suma **después de G1**, en med1) |

**Configuración:** Stop time = 30 s.

**Resultados esperados:**
- MS: $M_p$ ≈ 44 %, $t_s$ ≈ 5.4 s, error ante la perturbación = 0.
- MO: $M_p$ ≈ 5 %, $t_s$ ≈ 2.8 s, pero error ante la perturbación = −2.67.

## Modelo CP4-2b: cascada, interno MO (med1) + externo MS (med2)

```
                                                                                   [Step dist]──┐
                                                                                                ▼+
 [Step r]──►( Σ )──►[ PI externo ]── r1 ──►( Σ )──►[ PI interno ]── man ──►[ Gv ]──►[ G1 ]──►( Σ )──┬── med1 ──►[ 4/s ]──┬── med2
            + ▲ −                          + ▲ −                                               +     │                    │
              │                              └───────────────────────────────────────────────────────┘                    │
              └───────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Qué es cada señal / bloque del diagrama:**

| Símbolo | Qué es |
|---|---|
| r | Referencia de med2 |
| PI externo | Controla med2 (MS: P = 0.1875, I = 0.07031; o MO: solo P). Su salida r1 es la referencia del interno |
| r1 | Valor que se le pide a med1 |
| PI interno | Controla med1 (MO: P = 1, I = 0.5). Su salida man es el mando real |
| med1 | Medición intermedia (incluye la perturbación), realimentada al lazo interno |
| med2 | Salida final, realimentada al lazo externo |

| Bloque | Parámetros |
|---|---|
| PI interno (MO) | **P = 1, I = 0.5** |
| PI externo (MS) | **P = 0.1875, I = 0.07031** |
| Externo por MO (variante de las notas) | Controller = **P**: **P = 0.1875** |
| Resto | Igual que CP4-2a |

**Resultados esperados:**
- Externo MS: $M_p$ ≈ 54 %, $t_s$ ≈ 9.2 s, $\lvert u\rvert_{max}$ ≈ 1.24.
- Externo MO: $M_p$ ≈ 8 %, $t_s$ ≈ 4.4 s.
- Ambos: error ante la perturbación = 0.

---

## Script para graficar los resultados

Con *To Workspace* `r`, `y`, `u`, `e` (formato Array) en cualquier modelo:

```matlab
out = sim('nombre_modelo');
t = out.tout;
figure
subplot(3,1,1), plot(t, out.r, 'k--', t, out.y), grid on, legend('referencia','salida'), title('Referencia y salida')
subplot(3,1,2), plot(t, out.u), grid on, title('Acción de control')
subplot(3,1,3), plot(t, out.e), grid on, title('Error'), xlabel('t')
```

Para **comparar dos variantes** (por ejemplo, con y sin anti-windup):
```matlab
Kb = 0;    out1 = sim('modelo');     % en el bloque PID escribir Kb como variable "Kb"
Kb = 0.865; out2 = sim('modelo');
plot(out1.tout, out1.y, out2.tout, out2.y), legend('sin antiwindup','con antiwindup'), grid on
```

> Tip: en los bloques, escribir **nombres de variables** (P, I, D, N, Kb, umax) en vez de números. Así se cambia todo desde un script sin tocar el diagrama.
