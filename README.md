# 🤖 Manipulación Móvil Asistencial — Hello Robot Stretch

> Arquitectura de manipulación móvil autónoma sobre **Hello Robot Stretch** en el simulador **MuJoCo** para transporte de insumos médicos.  
> Navegación geométrica por odometría, control por fases cerradas y agarre con brazo telescópico.

---

## 📋 Descripción

Este repositorio contiene el script `medical_color_delivery.py`, diseñado para correr dentro del proyecto [`stretch_mujoco_digital_twin`](https://github.com/hello-robot/stretch_mujoco_digital_twin).

El sistema detecta un insumo médico en la escena, navega hacia él usando control geométrico retroalimentado, ejecuta la maniobra de flanco, agarra el objeto y lo transporta a una zona de entrega.

**Caso de uso:** traslado de medicamentos (Paracetamol / Tylenol) desde una `zona_recoleccion` hacia una `zona_deje` en un entorno médico simulado.

---

## ⚙️ Requisitos

Tener el proyecto base ya instalado y funcionando:

```bash
git clone https://github.com/hello-robot/stretch_mujoco_digital_twin
cd stretch_mujoco_digital_twin
uv sync
```

---

## 🚀 Instalación

Copia `medical_color_delivery.py` dentro del directorio del proyecto:

```bash
# Desde la raíz de stretch_mujoco_digital_twin
cp medical_color_delivery.py ./
```

O clona este repositorio y copia el archivo:

```bash
git clone https://github.com/DiegoLopezitesm/manipulacion-movil-asistencial-stretch
cp manipulacion-movil-asistencial-stretch/medical_color_delivery.py stretch_mujoco_digital_twin/
```

---

## ▶️ Uso

```bash
cd stretch_mujoco_digital_twin
uv run medical_color_delivery.py
```

Con aceleración GPU (recomendado en Linux con NVIDIA):

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia uv run medical_color_delivery.py
```

**Controles durante la ejecución:**

| Tecla | Acción |
|-------|--------|
| `ESPACIO` | Pausar / reanudar |
| `q` | Parar inmediatamente |

---

## 🔧 Configuración

Al inicio del archivo se encuentran los parámetros principales:

```python
TARGET_NAME       = "object2"   # object1 = Paracetamol (azul)
                                 # object2 = Tylenol (rojo)

FLANK_DIST        = 0.43        # distancia de trabajo robot↔objeto (m)
FLANK_OFFSET_DEG  = 4.9         # corrección del ángulo de flanco (°)
GRAB_LIFT_OFFSET  = 0.12        # offset del lift respecto a la altura del objeto

SHOW_CAMERA       = True        # mostrar cámara RGB
SHOW_LABELS       = True        # mostrar etiquetas Paracetamol / Tylenol
SHOW_LIDAR_VIEW   = True        # mostrar mapa tipo LiDAR con zonas
```

---

## 🏗️ Arquitectura de fases

```
Preparación → Percepción → Cálculo geométrico →
Orientación → Aproximación → Flanco →
Agarre → Transporte → Depósito
```

| Fase | Descripción |
|------|-------------|
| **Preparación** | Postura segura inicial |
| **Percepción** | `controller.get_object_pose()` — posición 3D del insumo |
| **Cálculo geométrico** | Distancia y ángulo relativo robot↔objeto |
| **Orientación** | Gira hasta tener el objeto al frente |
| **Aproximación** | Avanza hasta la distancia de trabajo |
| **Flanco** | Coloca el objeto alineado con el brazo lateral |
| **Ajuste de altura** | Lift se posiciona a la altura del insumo |
| **Alcance** | Brazo telescópico se extiende al rango de contacto |
| **Cierre** | Pinza sujeta el insumo |
| **Levantamiento** | Lift sube para separar el objeto de la mesa |
| **Retracción** | Brazo retrae para transporte seguro |
| **Transporte** | Giro 180° hacia `zona_deje` |
| **Depósito** | Extiende, suelta y retrae |

---

## 👥 Equipo

| Nombre |
|--------|
| Diego Hilario López |
| Patricio Garza Chapa |
| Marcelo de la Torre Esquinca |
| Rodrigo José Monterroso Bandy |

**Curso:** TE3003B — Implementación de Robótica Inteligente  
**Institución:** Tecnológico de Monterrey · Enero–Junio 2026
