# 🤖 Manipulación Móvil Asistencial — Hello Robot Stretch

> Arquitectura de manipulación móvil autónoma sobre **Hello Robot Stretch** en el simulador **MuJoCo** para transporte de insumos médicos.  
> Navegación geométrica por odometría, control por fases cerradas y agarre con brazo telescópico.

---

## 📋 Descripción

Este proyecto implementa un ciclo completo de **pick-and-place autónomo** sobre el gemelo digital del robot Stretch. El sistema detecta un insumo médico en la escena, navega hacia él usando control geométrico retroalimentado, ejecuta la maniobra de flanco, agarra el objeto y lo transporta a una zona de entrega.

La arquitectura está organizada como una **máquina de fases cerradas por control**: cada transición ocurre cuando se satisfacen condiciones medibles (tolerancias de distancia, error angular, posturas articulares), no por tiempos fijos.

**Caso de uso:** traslado de medicamentos (Paracetamol / Tylenol) desde una `zona_recoleccion` hacia una `zona_deje` en un entorno médico simulado.

---

## 🏗️ Arquitectura

```
Preparación → Percepción → Cálculo geométrico →
Navegación al flanco → Agarre → Transporte → Depósito
```

| Fase | Descripción |
|------|-------------|
| **Preparación** | Postura segura inicial (stow) |
| **Percepción** | `controller.get_object_pose()` → posición 3D exacta del insumo |
| **Cálculo geométrico** | Distancia euclidiana y ángulo relativo robot↔objeto |
| **Orientación** | Gira hasta tener el objeto al frente |
| **Aproximación** | Avanza/retrocede hasta distancia de trabajo (~0.43 m) |
| **Flanco** | Coloca el objeto en el costado derecho, alineado con el brazo |
| **Ajuste de altura** | Lift se posiciona a la altura del insumo |
| **Alcance** | Brazo telescópico se extiende al rango de contacto |
| **Cierre** | Pinza sujeta el insumo |
| **Levantamiento** | Lift sube para separar el objeto de la mesa |
| **Retracción** | Brazo retrae para transporte seguro |
| **Transporte** | Giro 180° hacia `zona_deje` |
| **Depósito** | Extiende, suelta y retrae |

---

## 🛠️ Tecnologías

- [Hello Robot Stretch](https://hello-robot.com/) — robot móvil con brazo telescópico
- [MuJoCo](https://mujoco.org/) — simulador de física
- [`stretch_mujoco_digital_twin`](https://github.com/hello-robot/stretch_mujoco) — gemelo digital
- `stretch_toolkit` — API de control (velocidades, StateController, odometría)
- Python 3.10 · OpenCV · NumPy

---

## 📁 Estructura del repositorio

```
.
├── medical_precise.py       # Script principal — pick-and-place autónomo
├── medical_camera.py        # Variante con servo visual (cámara de muñeca)
├── medical_scripted.py      # Variante con ruta hardcodeada + cámara en vivo
├── teach_mode.py            # Modo enseñanza interactivo — graba ruta desde terminal
├── ver_camara.py            # Visor de cámaras en vivo (Head / Wrist)
└── README.md
```

### Script principal: `medical_precise.py`

Usa las **posiciones reales del simulador** (`controller.get_object_pose`) y la **odometría** del robot para navegar sin depender de la cámara. La cámara se usa únicamente para visualización:

- Etiquetas **Paracetamol** (azul) y **Tylenol** (rojo) sobre la imagen RGB
- Mapa tipo **LiDAR** con `zona_recoleccion` y `zona_deje`

---

## ⚙️ Instalación

```bash
# Clonar el repositorio
git clone https://github.com/<usuario>/manipulacion-movil-asistencial-stretch
cd manipulacion-movil-asistencial-stretch

# Instalar dependencias (requiere uv)
uv sync
```

**Dependencias principales:**
```
mujoco
stretch-mujoco
opencv-python
numpy
pynput
inputs
```

---

## 🚀 Uso

```bash
# Script principal (recomendado)
uv run medical_precise.py

# Con servo visual por cámara de muñeca
uv run medical_camera.py

# Ruta hardcodeada con visualización
uv run medical_scripted.py

# Modo enseñanza — escribe comandos en terminal y graba la ruta
uv run teach_mode.py

# Ver cámaras en vivo
uv run ver_camara.py
```

**Controles durante la ejecución:**
| Tecla | Acción |
|-------|--------|
| `ESPACIO` | Pausar / reanudar |
| `q` | Parar inmediatamente |

**Cambiar objeto objetivo** en `medical_precise.py`:
```python
TARGET_NAME = "object2"   # object1 = Paracetamol (azul)
                          # object2 = Tylenol (rojo)
```

---

## 🔧 Parámetros configurables

```python
FLANK_DIST       = 0.43   # distancia de trabajo robot↔objeto (m)
FLANK_OFFSET_DEG = 4.9    # corrección del ángulo de flanco (°)
GRAB_LIFT_OFFSET = 0.12   # cuánto baja el lift respecto a obj_z
SHOW_CAMERA      = True   # mostrar cámara RGB
SHOW_LABELS      = True   # mostrar etiquetas de medicamento
SHOW_LIDAR_VIEW  = True   # mostrar mapa tipo LiDAR
```

---

## 👥 Equipo

| Nombre | Rol |
|--------|-----|
| Diego Hilario López | Navegación, control y arquitectura |
| Patricio Garza Chapa | Percepción y visión computacional |
| Marcelo de la Torre Esquinca | Integración y simulación |
| Rodrigo José Monterroso Bandy | Manipulación y secuencia de agarre |

---

## 🏫 Contexto académico

Proyecto desarrollado para el curso **TE3003B — Implementación de Robótica Inteligente**  
Tecnológico de Monterrey · Semestre Enero–Junio 2026

---

## 📄 Licencia

MIT
