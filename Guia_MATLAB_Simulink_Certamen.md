# Certamen en MATLAB/Simulink: cómo abordarlo, con plantillas de código y esquemas de diagramas

> Complementa `Guia_Abordaje_y_Problemas_Certamen.md` (teoría y problemas a mano). Aquí está **cómo hacer lo mismo en MATLAB/Simulink**, como en las guías CP-1 a CP-4.
> Las funciones auxiliares están en la carpeta **`Plantillas_MATLAB/`**.
> ⚠️ El código se escribió sin acceso a MATLAB (no se ejecutó). Sigue la sintaxis estándar del Control System Toolbox y de Simulink, pero **pruébalo antes del certamen** con los ejercicios de las guías, cuyos resultados conocidos están en `CP1/`…`CP4/`.

## Índice
1. [Preparación antes del certamen](#1-preparación-antes-del-certamen)
2. [Estructura del script en el certamen](#2-estructura-del-script-en-el-certamen)
3. [Comandos esenciales (chuleta)](#3-comandos-esenciales-chuleta)
4. [Recetas en MATLAB por tipo de ejercicio](#4-recetas-en-matlab-por-tipo-de-ejercicio)
5. [Esquemas de Simulink](#5-esquemas-de-simulink)
6. [Leer resultados de Simulink y graficar](#6-leer-resultados-de-simulink-y-graficar)
7. [Errores típicos en MATLAB/Simulink](#7-errores-típicos-en-matlabsimulink)
8. [Qué mostrar en el documento de respuesta](#8-qué-mostrar-en-el-documento-de-respuesta)

---

## 1. Preparación antes del certamen

1. Copiar la carpeta `Plantillas_MATLAB/` al computador del certamen (si se permite) y ejecutar `addpath('ruta/Plantillas_MATLAB')`.

   | Función | Para qué |
   |---|---|
   | `port_datos(t,y,dU,t0)` | Modelo PORT por el método 28 %–63 % (desde `step` o desde datos de Simulink) |
   | `ku_tu(G)` | $K_u$, $T_u$ con `margin` (acepta retardo) |
   | `sintonia(metodo,tipo,...)` | ZN-LC, ZN-LA, CC, ISE/IAE/ITAE (ref/pert). Entrega ideal y **paralela**; convierte solo el PID serie de ZN |
   | `sintesis('MO'\|'MS',Gp,Tu,Kr)` | Controlador por módulo óptimo o simétrico |
   | `tf2pid(Gc)` | Lleva una FT a $P+I/s+Ds$ y dice si es P/PI/PD/PID |
   | `evaluar_lazo(Gc,Gp,Gd,H,tfin,td)` | Lazo simple: los 3 gráficos (ref/salida, mando, error) + $M_p$, $t_s$, $e_{ss}$ |
   | `plantilla_cascada_connect.m` | Cascada o cualquier diagrama con `connect` |
   | `crear_lazo_simulink(...)` | (Opcional) Crea el modelo Simulink de lazo simple por código |
2. **Probar cada función** con los CP ya resueltos, por ejemplo:

   ```matlab
   G = tf(1.652,[0.2 1])*tf(1.183,conv([8.34 1],[0.502 1]))*tf(1,[0.75 1]);
   ku_tu(G)
   ```

   Debe dar $K_u$ ≈ 10.43 y $T_u$ ≈ 4.62 (CP-1).
3. Si no se permite llevar archivos, memorizar las **recetas cortas** de la sección 4. Cada una son 5–10 líneas.

---

## 2. Estructura del script en el certamen

Un solo script con **secciones** (`%%`), que se ejecutan una a una con *Ctrl+Enter*:

```matlab
%% Certamen - Pregunta 1
clear; clc; close all;
s = tf('s');

%% 1) Modelo de la planta (forma de constantes de tiempo)
Gv = 2/(s+2);             % = 1/(0.5s+1)
G1 = 5/(2*s+1);
G2 = 3/(5*s+1)*exp(-1*s); % retardo de 1 s
Gp = Gv*G1*G2;
zpk(Gp)                   % ver polos/ceros; damp(Gp) muestra las constantes de tiempo

%% 2) Identificación / datos de diseño
% ... (PORT, Ku/Tu, o Tu para MO/MS)

%% 3) Diseño del controlador
% ... (sintonia / sintesis) -> P, I, D

%% 4) Validación (MATLAB o Simulink) y métricas
% ... (evaluar_lazo / sim('modelo'))

%% 5) Comentarios
% Escribir aquí las conclusiones (o en el doc de respuesta)
```

**Regla de oro:** que cada resultado pedido quede **impreso** (`fprintf` o sin `;`) y **graficado con título**, porque es lo que se copia al documento.

---

## 3. Comandos esenciales (chuleta)

| Tarea | Comando |
|---|---|
| Variable de Laplace | `s = tf('s');` |
| FT | `G = tf(num, den)` o `G = 5/(2*s+1)` |
| Retardo | `G = tf(3,[5 1],'InputDelay',1)` o `G*exp(-1*s)` |
| Serie / paralelo / lazo cerrado | `G1*G2`, `G1+G2`, `feedback(G, H)` (realimentación negativa por defecto) |
| Lazo cerrado ref → salida | `T = feedback(Gc*Gp, H)` |
| Perturbación a la entrada de la planta → salida | `Td = feedback(Gp, Gc*H)` |
| Perturbación con camino propio $G_d$ → salida | `Td = Gd*feedback(1, Gc*Gp*H)` |
| Acción de control ante ref. | `Ur = feedback(Gc, Gp*H)` (Gc debe ser **propio**) |
| Simplificar | `minreal(G)` |
| Respuesta a escalón / rampa / señal | `step(G)`, `[y,t]=step(G,t)`, `lsim(G,t,t)` (rampa), `lsim(G,u,t)` |
| Métricas | `S = stepinfo(T)` → `S.Overshoot`, `S.SettlingTime` (2 %) |
| Ganancia estática ($e_{ss}$) | `dcgain(G)` → error ante escalón = `1 - dcgain(T)` |
| Error ante rampa | `dcgain(minreal(tf(1,[1 0])/(1+Gc*Gp)))` o `lsim` y leer el final |
| Márgenes, Ku, Tu | `[Gm,Pm,Wcg,Wcp] = margin(G)`; Ku = Gm (veces); `20*log10(Gm)` si lo piden en dB |
| Polos, ζ, ωn | `pole(T)`, `damp(T)` |
| PID paralelo con filtro | `C = pid(P, I, D, Tf)` con `Tf = 1/N` |
| PID ideal (estándar) | `C = pidstd(Kc, Ti, Td, N)` → a paralelo con `pid(C)` |
| Coeficientes de una FT | `[num, den] = tfdata(G, 'v')` |
| Sistema con nombres (cascada) | `G.InputName='u'; G.OutputName='y'; sumblk('e = r - y'); connect(...)` |
| Linealizar un modelo Simulink | `[A,B,C,D] = linmod('modelo'); G = tf(ss(A,B,C,D))` (los retardos no los toma bien: para eso, experimento de escalón) |
| Correr Simulink | `out = sim('modelo');` y luego `out.y`, `out.tout` (ver sección 6) |

---

## 4. Recetas en MATLAB por tipo de ejercicio

### 4.1 Modelo de la planta y forma de constantes de tiempo
```matlab
s = tf('s');
Gp = 100/(s*(s+10));
zpk(Gp)          % polos: 0 y -10  ->  Gp = 10/(s(0.1s+1))  ->  Tu = 0.1, K = 10
damp(Gp)         % columna "Time Constant"
dcgain(Gp*s)     % ganancia sin el integrador (K = 10)
```

### 4.2 Identificación PORT (curva de reacción), tipo CP-1 y CP-4
```matlab
% a) Desde el modelo (MATLAB)
t = 0:0.01:100;
y = step(Gp, t);
[K, T, L] = port_datos(t, y, 1, 0);
Gport = tf(K, [T 1], 'InputDelay', L);
figure, step(Gp, Gport, t), legend('planta','PORT'), grid on
% Marcar los puntos en el gráfico:
hold on, plot([L+T/3, L+T], K*[0.283 0.632], 'ro')

% b) Desde Simulink (experimento en lazo abierto, esquema 5.1): escalón dU en t0
out = sim('identificacion');
[K, T, L] = port_datos(out.tout, out.y, dU, t0);
```

### 4.3 Ku y Tu, tipo CP-1 y CP-4
```matlab
Glazo = Gv*Gp*H;                 % TODO el lazo sin el controlador (válvula, planta, sensor)
[Ku, Tu] = ku_tu(Glazo);         % internamente: [Gm,~,Wcg] = margin(G); Ku = Gm; Tu = 2*pi/Wcg
margin(Glazo)                    % gráfico de Bode con los márgenes marcados (para el doc)
```
Si se pide **experimentalmente** (esquema 5.2): en Simulink, lazo cerrado con un bloque *Gain* $K_c$. Se sube $K_c$ hasta que la salida oscile con amplitud constante. Ese valor es $K_u$, y el período entre picos del *Scope* es $T_u$.

### 4.4 Sintonía ZN, CC y criterios integrales, y comparación, tipo CP-1
```matlab
c1 = sintonia('ZN-LC',   'PID', Ku, Tu);    % convierte serie -> ideal -> paralela
c2 = sintonia('ZN-LA',   'PID', K, T, L);
c3 = sintonia('CC',      'PID', K, T, L);
c4 = sintonia('IAE-ref', 'PID', K, T, L);
c5 = sintonia('ITAE-pert','PID', K, T, L);
N  = 100;
metodos = {c1, c2, c3, c4, c5};
figure; hold on; grid on
for k = 1:numel(metodos)
    c = metodos{k};
    C = pid(c.P, c.I, c.D, 1/N);
    step(feedback(C*Gp, H), 40);                        % referencia
end
legend(cellfun(@(c) c.metodo, metodos, 'UniformOutput', false))
% Idem para la perturbación: step(Gd*feedback(1, C*Gp*H), 40)
```
Sin las funciones, **a mano** (ZN con Ku):
```matlab
Kcs = Ku/1.6;  Tis = Tu/2;  Tds = Tu/8;                        % serie
Kc = Kcs*(1+Tds/Tis);  Ti = Tis+Tds;  Td = Tis*Tds/(Tis+Tds);   % ideal
P = Kc;  I = Kc/Ti;  D = Kc*Td;                                % paralela
Creal  = Kcs*(1 + 1/(Tis*s))*(Tds*s + 1);                      % "PID real" (serie), como en la conferencia
Cideal = pid(P, I, D, 1/100);
```

### 4.5 Planta de 1er orden con PI, tipo CP-2
```matlab
K = 0.0125; T = 2.5; Gp = tf(K, [T 1]);
% --- Cancelación ---
tss = 8;  Tlc = tss/4;
Kp = T/(Tlc*K);  Ti = T;
C  = Kp*(Ti*s + 1)/(Ti*s);
% --- 2º orden impuesto (+ prefiltro si tss < 5T) ---
z = 0.7; wn = 4/(z*tss);
Kp = (2*z*wn*T - 1)/K;   Ti = Kp*K/(T*wn^2);
C  = Kp*(Ti*s + 1)/(Ti*s);   Pf = tf(1, [Ti 1]);
% --- Chequear el mando ante un escalón de amplitud A (límite umax) ---
A = 0.02;
u = step(A*Pf*feedback(C, Gp), 0:0.01:40);
fprintf('u max = %.3f\n', max(abs(u)))
% --- Referencia y perturbación (Gd = FT perturbación -> salida en lazo abierto) ---
Gd = tf(1, [10 4]);            % ejemplo CP-2
step(A*Pf*feedback(C*Gp,1), Gd*feedback(1, C*Gp), 40)
```
Si el mando satura: bajar $K_p$ (cancelación: $K_p\le u_{max}/A$) o aumentar $t_{ss}$.

### 4.6 Módulo óptimo y módulo simétrico, tipo CP-3
```matlab
Gp = 100/(s*(s+10));  Tu = 0.1;          % ¡Tu = menor constante de tiempo!
Gco = sintesis('MO', Gp, Tu);   tf2pid(Gco);   % -> P = 0.5
Gcs = sintesis('MS', Gp, Tu);   tf2pid(Gcs);   % -> PI: 0.5 + 1.25/s
% Sin funciones: Gco = minreal(1/(2*Tu*s*(Tu*s+1)*Gp));
%                Gcs = minreal((4*Tu*s+1)/(8*Tu^2*s^2*(Tu*s+1)*Gp));
t = 0:0.001:4;
figure
subplot(3,1,1), step(feedback(Gco*Gp,1), feedback(Gcs*Gp,1), t), title('Escalón en ref'), legend('MO','MS')
subplot(3,1,2), lsim(feedback(Gco*Gp,1), feedback(Gcs*Gp,1), t, t), title('Rampa en ref')
subplot(3,1,3), step(feedback(Gp,Gco), feedback(Gp,Gcs), t), title('Escalón en perturbación (entrada planta)')
% Errores en estado estable
for G = {Gco, Gcs}
    Lz = G{1}*Gp;
    fprintf('e_escalón = %.3g  e_rampa = %.3g  y_pert = %.3g\n', ...
        dcgain(1/(1+Lz)), dcgain(minreal(tf(1,[1 0])/(1+Lz))), dcgain(feedback(Gp, G{1})));
end
```
> Si `tf2pid` dice "no es PID", es el MS sobre una planta tipo 0 (doble integrador). Se comenta y se propone cascada.

### 4.7 Cascada, tipo CP-3 y CP-4
**Receta rápida** (sin `connect`), ambos lazos por MO:
```matlab
Gi  = Ga;                                   % planta del lazo interno (hasta el sensor intermedio)
Gc1 = sintesis('MO', Gi, Tu_in);            % interno
Gcl1 = feedback(Gc1*Gi, 1);                 % ≈ 1/(2*Tu_in*s + 1)
Go  = tf(1,[2*Tu_in 1])*Gb;                 % planta aproximada del externo
Gc2 = sintesis('MO', Go, 2*Tu_in);          % externo: Tu_ext = 2*Tu_in (o 'MS')
Tr  = feedback(Gc2*Gcl1*Gb, 1);             % referencia -> salida (con el interno EXACTO)
% Perturbación d a la entrada de Ga:  Y/D = Gb*Ga/(1 + Ga*Gc1*(1 + Gc2*Gb))
Td  = Gb*feedback(Ga, Gc1*(1 + Gc2*Gb));
step(Tr), figure, step(Td)
```
**General** (perturbación en cualquier punto, varias salidas): usar `plantilla_cascada_connect.m`, que arma el sistema con `connect` a partir de nombres de señales y grafica salida, mando y error.

### 4.8 Saturación, anti-windup, filtro y derivada de la medición
Estos efectos **no lineales** se evalúan en **Simulink** (esquemas 5.3 y 5.4). En MATLAB solo se calculan los parámetros:
```matlab
P = Kc; I = Kc/Ti; D = Kc*Td;
Kb = 1/sqrt(Ti*Td);        % PID (Conf. 1).  PI: Kb = 1/Ti.  (La guía CP-3 usa Kb = I/P)
N  = 10/Tu;                % filtro: 1/N bastante menor que Tu (evitar N = P/(alfa*D) si da 1/N ~ Tu)
```

---

## 5. Esquemas de Simulink

**Bloques más usados** (Library Browser → Simulink):

| Bloque | Ruta | Parámetros clave |
|---|---|---|
| Step | Sources | Step time, Initial value, Final value |
| Ramp | Sources | Slope, Start time |
| Sum | Math Operations | List of signs: `+-` (error) / `++` (perturbación) |
| Gain | Math Operations | Gain |
| Transfer Fcn | Continuous | Numerator `[5]`, Denominator `[2 1]` |
| Transport Delay | Continuous | Time delay (retardo $L$) |
| Integrator | Continuous | (para armar un PID a mano) |
| PID Controller | Continuous | Form (Parallel/Ideal), P, I, D, N, Output saturation, Anti-windup |
| PID Controller (2DOF) | Continuous | Igual + pesos b, c (**c = 0 → D sobre la medición**) |
| Saturation | Discontinuities | Upper/Lower limit |
| Scope | Sinks | Number of input ports (varias curvas: usar *Mux*) |
| Mux | Signal Routing | Number of inputs |
| To Workspace | Sinks | Variable name, Save format: *Array* |

**Configuración del modelo** (*Model Settings* / Ctrl+E):
- Stop time según la dinámica: unas 5–10 veces la constante más lenta.
- Con **Transport Delay** o dinámicas rápidas: *Max step size* = 0.01 (o menor), para que las curvas no salgan quebradas.
- Los escalones se aplican en **t = 1**, no en 0, para que se vea el valor inicial. La perturbación se aplica **después** de que la salida se asentó, para ver ambas respuestas en una sola simulación.

### 5.1 Experimento en lazo abierto (identificación PORT)
```
 [Step]──────────► u ──►[ Planta (Transfer Fcn(s) / Subsystem) ]──►[Transport Delay]──► y ──┬──►[Scope]
 Step time = t0                                                                              └──►[To Workspace "y"]
 Initial = u0, Final = u0 + dU
```
Luego `[K,T,L] = port_datos(out.tout, out.y, dU, t0)`. Si la planta viene como bloque cerrado (*Subsystem* con entradas `man` y `dist`), se conecta el Step a `man`, se deja `dist` en 0 (*Constant* 0) y se mide la salida pedida.

### 5.2 Experimento de Ziegler-Nichols en lazo cerrado (Ku, Tu)
```
 [Step]──►(+)──►[ Gain Kc ]──►[ Planta ]──┬──► y ──►[Scope]
            ▲−                             │
            └──────────[ Sensor H ]◄───────┘
```
Se sube **Kc** (se puede usar un *Slider Gain*) hasta ver oscilaciones sostenidas: esa es $K_u$. $T_u$ es el tiempo entre dos picos consecutivos (usar los *Cursor Measurements* del Scope). Contrastar con `margin`.

### 5.3 Lazo simple con PID, perturbación y los 3 gráficos pedidos (CP-4 a3/b5)
```
                                                     [Step d] (t = td)
                                                         │
 [Step r]──┬──►(+)── e ──►[ PID Controller ]── u ──┬──►(+)──►[ Planta ]──►[Transport Delay]──┬──► y
  t = 1    │    ▲−    │                             │    ++                                    │
           │    │     └──►[To Workspace "e"]        └──►[To Workspace "u"]                     │
           │    └───────────────────────────────[ Sensor H ]◄─────────────────────────────────┤
           └──►[Mux]◄──────────────────────────────────────────────────────────────────────────┘
                 └──►[Scope "ref y salida"]      (+ Scopes para u y e)
```
- **PID Controller:** *Form = Parallel*; P, I, D del diseño; *Filter coefficient N* = 100 (o $10/T_u$).
- Si el método da forma ideal ($K_c$, $T_i$, $T_d$): *Form = Ideal*, con P = $K_c$, I = $1/T_i$, D = $T_d$. Ojo: en el bloque **Ideal**, "I" es $1/T_i$ y "D" es $T_d$. O convertir a paralelo y usar *Parallel*, que es lo más seguro.
- Saturación: pestaña *Output Saturation* → *Limit output* (±umax). Anti-windup: *back-calculation* con $K_b$.
- **D sobre la medición:** usar *PID Controller (2DOF)* con b = 1 y **c = 0**.
- Dónde sumar la perturbación: **exactamente donde indica el enunciado** (en la entrada de la planta, entre bloques, en la salida…).

### 5.4 PID "a mano" (diagrama de la guía CP-3): P, I con anti-windup y D de la medición con filtro
```
 SP ──►(+)── e ─┬──►[Gain P]──────────────────────────────┐
       ▲−       │                                          ▼+
 med ──┤        └──►[Gain I]──►(+)──►[1/s]──────────────►(+)── u ──►[Saturation ±umax]──┬──► us (a la planta)
       │                       ▲+                          ▲+                  │          │
       │                       └──[Gain Kb]◄──(+)◄─────────┼───────── us ──────┼──────────┘
       │                                      ▲−  (us − u) │          u ───────┘
       │                                                   │
       └──►[Gain −D]──►(+)──►[Gain N]──┬── ud ─────────────┘
                        ▲−             │
                        └────[1/s]◄────┘        ud = N(−D·med − ∫ud)  →  −D·N·s/(s+N) · med
```
Úsalo cuando se pida "evaluar el efecto" de cada modificación: se agregan de a una (saturación → filtro → D sobre la medición → anti-windup), como en el CP-1 y el CP-3. Conviene armarlo como **Subsystem** (entradas SP y med, salida u) para reutilizarlo en la cascada.

### 5.5 Cascada (mando subordinado), CP-3 y CP-4
```
                                                             [Step d]
                                                                │
 [Step r]──►(+)──►[ PID externo Gc2 ]── r1 ──►(+)──►[ PI interno Gc1 ]── u ──►[Ga]──►(+)──►[Gb1]──┬── med1 ──►[Gb2]──►[Retardo]──┬──► med2
            ▲−                                 ▲−                                                  │                              │
            │                                  └──────────────────── med1 ◄────────────────────────┘                              │
            └─────────────────────────────────────────────────── med2 ◄───────────────────────────────────────────────────────────┘
```
- El **interno** genera el mando, así que la **saturación y el anti-windup van en el interno**.
- Salida del externo = referencia del interno (sin límite, salvo que se pida limitar la variable intermedia).
- Para comparar con el lazo único: duplicar el modelo, o usar un *Manual Switch*.

### 5.6 Planta de 1er orden: PI + prefiltro + saturación (CP-2)
```
 [Step r (A)]──►[ Prefiltro 1/(Ti s+1) ]──►(+)──►[ PI: Kp(Ti s+1)/(Ti s) ]──►[Saturation ±umax]──►(+)──►[ K/(Ts+1) ]──┬──► y
                                            ▲−                                                    ▲+ [Step d]→[Gd]   │
                                            └─────────────────────────────────────────────────────────────────────────┘
```
Poner un Scope **en la salida del PI antes de la saturación**, para demostrar que $|u|\le u_{max}$ (sin saturar).

### 5.7 (Opcional) Crear el lazo simple por código
```matlab
% Planta 5/((2s+1)(0.5s+1)), PID P=1 I=0.5 D=0.2 N=100, saturación ±3, Kb=0.5, sin retardo
crear_lazo_simulink('lazo1', 5, conv([2 1],[0.5 1]), 1, 0.5, 0.2, 100, 3, 0.5, 0)
out = sim('lazo1');
```
(Función de `Plantillas_MATLAB/`, no probada. Si falla, se arma a mano con el esquema 5.3.)

---

## 6. Leer resultados de Simulink y graficar

Con bloques **To Workspace** (formato *Array*) llamados `y`, `u`, `e`:
```matlab
out = sim('lazo1');                 % R2019b+: todo queda dentro de "out"
t = out.tout;  y = out.y;  u = out.u;  e = out.e;
r = double(t >= 1);                 % la referencia que se aplicó
figure
subplot(3,1,1), plot(t, r, 'k--', t, y), grid on, legend('referencia','salida'), title('Referencia y salida')
subplot(3,1,2), plot(t, u), grid on, title('Acción de control')
subplot(3,1,3), plot(t, e), grid on, title('Error'), xlabel('t')
% Métricas desde los datos (ventana de la respuesta a la referencia, antes de td):
m = t < td;
info = stepinfo(y(m), t(m), 1)      % sobrepaso y t_s respecto del valor final 1
% OJO: SettlingTime se mide desde t = 0; como el escalón fue en t = 1, restar 1.
ess = r(end) - y(end)
```
Para **comparar varias estrategias** en un gráfico: guardar cada `out` en una variable distinta (`out1 = sim(...)`, cambiar los parámetros con `set_param` o en el workspace, luego `out2 = sim(...)`) y superponer con `plot(out1.tout,out1.y, out2.tout,out2.y)`.

> Tip: usar **variables del workspace** en los bloques (P, I, D, N, umax, Kb) en vez de números. Así se cambia el diseño desde el script y se re-simula sin tocar el diagrama.

---

## 7. Errores típicos en MATLAB/Simulink

1. **`margin` en dB:** el `Gm` que devuelve `[Gm,Pm,Wcg]=margin(G)` está **en veces**. El gráfico de Bode muestra dB. $K_u$ = Gm (veces).
2. **Olvidar sensor o válvula** al calcular $K_u$ o el PORT: se usa el lazo completo sin el controlador.
3. **`pid` vs. `pidstd`:** `pid(P,I,D)` es paralelo; `pidstd(Kc,Ti,Td)` es ideal. Los parámetros de ZN (serie) hay que **convertirlos** antes.
4. **Bloque PID en forma Ideal:** el campo "I" es $1/T_i$, no $T_i$.
5. **FT impropia:** un PID sin filtro hace que `feedback(Gc, Gp)` (la acción de control) sea impropia y `step`/`lsim` fallan. Usar `pid(P,I,D,1/N)`.
6. **Signos del Sum:** el error es `+-` (r − y). La perturbación se suma con `++` (o `+-` si el enunciado la resta).
7. **Signo de la realimentación en `feedback`:** es negativa por defecto. `feedback(G,H,+1)` solo para realimentación positiva.
8. **Retardo:** `exp(-L*s)` funciona con `s = tf('s')`. En Simulink usar *Transport Delay* y un *Max step size* pequeño. MO y MS **no** se aplican a plantas con retardo.
9. **Escalón en t = 0 en Simulink:** el sobrepaso y $t_s$ calculados desde los datos quedan corridos. Aplicar en t = 1 y restar.
10. **`minreal`** después de cancelar polos y ceros (MO/MS). Sin él quedan polos y ceros "fantasma" y `tf2pid` no identifica el controlador.
11. **Tu mal elegido:** es la **menor** constante. `damp(Gp)` lista las constantes de tiempo.
12. **Filtro derivativo demasiado lento:** si $1/N$ es comparable a $T_u$, el MO/MS se degrada (CP-3). Usar $N\gtrsim10/T_u$.

---

## 8. Qué mostrar en el documento de respuesta

Por cada inciso, como piden las guías ("plasme en el doc…"):
1. **Segmento de código** + **salida de MATLAB** (valores impresos: K, T, L, Ku, Tu, P, I, D).
2. **Gráficos con puntos significativos marcados:**
   - identificación: $t_{28}$ y $t_{63}$, o el cruce por −180° con `margin`;
   - respuesta: sobrepaso y $t_s$, con *data tips* (clic derecho en la curva) o con `stepinfo`.
3. **Controlador en forma $P+I/s+Ds$** y su tipo (P/PI/PD/PID).
4. Para la validación: **los 3 gráficos** (referencia y salida, acción de control, error) con **escalón en la referencia y luego en la perturbación**.
5. **Captura del diagrama de Simulink** (*File → Export → To image*, o una captura de pantalla).
6. **Tabla comparativa** ($e_{ss}$, $M_p$, $t_s$, pico ante perturbación, mando máximo) y **2–4 frases de conclusión**. Las conclusiones "tipo" de cada CP están en `CP1/`…`CP4/README.md`.
