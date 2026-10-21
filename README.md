# Control-de-Procesos

Material de estudio del ramo **Control de Procesos** (Dr. Angel E. Rubio R., DIEE-UBB): resúmenes de las conferencias, prácticos (CP) resueltos paso a paso, certámenes resueltos y guías para resolver en MATLAB/Simulink.

## Por dónde empezar

1. **Teoría:** leer el resumen de las conferencias correspondientes.
2. **Práctica:** abrir la carpeta del CP. Su `README.md` tiene el desarrollo completo (fórmulas, números, gráficos y conclusiones), y los scripts `.m` reproducen todos los resultados.
3. **Simulink:** las guías `Diagramas_Simulink_*.md` explican cada modelo bloque por bloque.
4. **Certamen:** las guías de abordaje y los certámenes anteriores resueltos.

## Contenido

### Resúmenes de conferencias

| Archivo | Conferencias | Temas |
|---|---|---|
| [`Resumen_Certamen1_Conf1-3.md`](Resumen_Certamen1_Conf1-3.md) | 1, 2 y 3 | PID, errores en estado estable, modelo PORT, Ziegler-Nichols, Cohen-Coon, criterios integrales, módulo óptimo y simétrico, cascada |
| [`Resumen_Conf4-6.md`](Resumen_Conf4-6.md) | 4, 5 y 6 | Cascada (RFB), selectivo y sobrecorrida, rango dividido, relación, feedforward, predictor de Smith, compensador de respuesta inversa, control inferencial |

### Prácticos resueltos

| Carpeta | Tema | Qué incluye |
|---|---|---|
| [`CP1/`](CP1/README.md) | Ajuste de controladores (ZN, CC, criterios integrales) | Desarrollo, script MATLAB, scripts Python y gráficos |
| [`CP2/`](CP2/README.md) | Control de plantas de primer orden | Ídem |
| [`CP3/`](CP3/README.md) | Módulo óptimo, simétrico y mando subordinado | Ídem |
| [`CP4/`](CP4/README.md) | Control de plantas (lazo simple y cascada) | Ídem |
| [`CP5/`](CP5/README.md) | Feedforward y combinación FF-FB | Desarrollo, `cp5.m`, **modelos Simulink `.slx`** verificados y gráficos |
| [`CP6/`](CP6/README.md) | Predictor de Smith, respuesta inversa, control inferencial | Desarrollo, `cp6.m`, **modelos Simulink `.slx`** verificados y gráficos |

### Simulink

| Archivo | Qué es |
|---|---|
| [`Diagramas_Simulink_CP1-CP4.md`](Diagramas_Simulink_CP1-CP4.md) | Cómo armar a mano los modelos de CP1–CP4: diagramas, bloques y parámetros |
| [`Diagramas_Simulink_CP5-CP6.md`](Diagramas_Simulink_CP5-CP6.md) | Imágenes y tablas de bloques de los modelos `.slx` de CP5–CP6 (que ya vienen hechos y probados) |

### Certámenes

| Archivo / carpeta | Qué es |
|---|---|
| [`Guia_Abordaje_y_Problemas_Certamen.md`](Guia_Abordaje_y_Problemas_Certamen.md) | Cómo abordar los ejercicios del Certamen 1 y problemas tipo para resolver a mano |
| [`Guia_MATLAB_Simulink_Certamen.md`](Guia_MATLAB_Simulink_Certamen.md) | Cómo resolver lo mismo en MATLAB/Simulink, con plantillas |
| [`Plantillas_MATLAB/`](Plantillas_MATLAB) | Funciones auxiliares: Ku/Tu, PORT, reglas de sintonía, síntesis (MO/MS), armado y evaluación de lazos |
| [`Certamen1_2025/`](Certamen1_2025/README.md) | Certamen 1 2025 resuelto: lazo único vs. cascada, saturación y anti-windup |
| [`Certamen1_2026/`](Certamen1_2026/README.md) | Certamen 1 2026 resuelto: perturbación en el lazo interno, PID 2DOF y clamping |

## Cómo correr los scripts de CP5 y CP6

Requieren MATLAB con **Control System Toolbox** (y **Simulink** para los `*_simulink.m`). Se probaron en R2026a.

```matlab
cd CP5
cp5              % cálculos + gráficos ej*.png + métricas en la consola
cp5_simulink     % (re)genera los .slx, los simula, verifica contra cp5.m y exporta diag_*.png
```

Lo mismo con `cp6` y `cp6_simulink` en `CP6/`. Ojo: `*_simulink.m` crea los modelos desde cero, así que pisa los cambios que se hayan hecho a mano en los `.slx`.

## Organización del ramo

| Tema | Contenido | Evaluación |
|---|---|---|
| T1: Control SISO | Sintonía (ZN, CC, criterios integrales), control basado en modelo, cascada, feedforward, selectivo, rango dividido, relación | Certamen 1 |
| T2: Compensadores | Predictor de Smith, compensador de respuesta inversa, control inferencial | Certamen 2 |
| T3: Control MIMO | Desacopladores, arreglo de Bristol (RGA), control de 1, 2 y 3 impulsos | Certamen 3 |
