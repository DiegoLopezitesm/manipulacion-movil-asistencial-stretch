#!/usr/bin/env python3
"""
medical_precise.py
==================
Manipulación Móvil Asistencial — Hello Robot Stretch
Arquitectura de fases cerradas por control.

"""

from stretch_toolkit import controller, BACKEND_NAME, StateController, HEAD_RGB_CAMERA
import math, time, cv2, numpy as np

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN
# ══════════════════════════════════════════════════════════════════════════════
TARGET_NAME       = "object2"   # object1=Paracetamol(azul)  object2=Tylenol(rojo)

FLANK_DIST        = 0.43        # distancia de trabajo robot↔objeto (m)
FLANK_TOL         = 0.04        # tolerancia de distancia (m)
ANGLE_TOL         = 0.08        # tolerancia angular (rad) ≈ 4.5°
FLANK_OFFSET_DEG  = 4.9         # corrección del ángulo final de flanco
GRAB_LIFT_OFFSET  = 0.12        # cuánto baja el lift respecto a obj_z para agarrar

SHOW_CAMERA       = True        # mostrar cámara RGB durante la misión
SHOW_LABELS       = True        # mostrar etiquetas Paracetamol / Tylenol
SHOW_LIDAR_VIEW   = True        # mostrar mapa tipo LiDAR con zonas

Kp_angle   = 1.6
Kp_forward = 1.2

# Zonas del mapa (solo visual, no controlan la navegación)
ZONA_RECOLECCION = {"name": "zona_recoleccion", "center": (0.0, -1.0), "size": (1.2, 1.0)}
ZONA_DEJE        = {"name": "zona_deje",         "center": (0.0,  1.0), "size": (1.2, 1.0)}

# ══════════════════════════════════════════════════════════════════════════════
#  VISIÓN — solo etiquetas, no controla nada del robot
# ══════════════════════════════════════════════════════════════════════════════

def draw_medicine_labels(frame):
    """Dibuja etiquetas Paracetamol(azul) y Tylenol(rojo). Solo visual."""
    if frame is None:
        return frame
    vis = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    k   = np.ones((5, 5), np.uint8)

    blue = cv2.morphologyEx(
        cv2.inRange(hsv, np.array([105,120,70]), np.array([130,255,255])),
        cv2.MORPH_OPEN, k)
    r1 = cv2.inRange(hsv, np.array([0,  100, 80]), np.array([10, 255,255]))
    r2 = cv2.inRange(hsv, np.array([165,100, 80]), np.array([180,255,255]))
    red = cv2.morphologyEx(cv2.bitwise_or(r1, r2), cv2.MORPH_OPEN, k)

    for name, mask, color in [("Paracetamol", blue, (255,0,0)),
                               ("Tylenol",     red,  (0,0,255))]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 150: continue
            x, y, w, h = cv2.boundingRect(c)
            if w < 8 or h < 8 or w > 180 or h > 180: continue
            asp = w / float(h)
            if asp < 0.35 or asp > 2.8: continue
            cv2.rectangle(vis, (x,y), (x+w,y+h), color, 2)
            cv2.putText(vis, name, (x, max(20, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return vis

# ══════════════════════════════════════════════════════════════════════════════
#  MAPA TIPO LiDAR — solo visual
# ══════════════════════════════════════════════════════════════════════════════

def _w2px(x, y, W=600, H=600, sc=180):
    return int(W/2 + x*sc), int(H/2 - y*sc)

def draw_lidar_view(get_robot_fn):
    """Mapa 2D con zona_recoleccion, zona_deje y posición del robot. Solo visual."""
    if not SHOW_LIDAR_VIEW:
        return
    img = np.full((600,600,3), 25, dtype=np.uint8)
    for i in range(0, 600, 60):
        cv2.line(img,(i,0),(i,600),(45,45,45),1)
        cv2.line(img,(0,i),(600,i),(45,45,45),1)
    cx,cy = _w2px(0,0)
    cv2.line(img,(cx,0),(cx,600),(80,80,80),1)
    cv2.line(img,(0,cy),(600,cy),(80,80,80),1)

    for zone, color in [(ZONA_RECOLECCION,(0,180,255)), (ZONA_DEJE,(0,255,120))]:
        zx,zy = zone["center"]; sx,sy = zone["size"]
        x1,y1 = _w2px(zx-sx/2, zy+sy/2)
        x2,y2 = _w2px(zx+sx/2, zy-sy/2)
        cv2.rectangle(img,(x1,y1),(x2,y2),color,2)
        tx,ty = _w2px(zx, zy)
        cv2.putText(img, zone["name"], (tx-85,ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    try:
        rx,ry,rt = get_robot_fn()
        rpx,rpy  = _w2px(rx, ry)
        cv2.circle(img,(rpx,rpy),8,(255,255,255),-1)
        dx,dy = int(25*math.cos(rt)), int(-25*math.sin(rt))
        cv2.arrowedLine(img,(rpx,rpy),(rpx+dx,rpy+dy),(255,255,255),2,tipLength=0.3)
        cv2.putText(img,"robot",(rpx+10,rpy-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1)
    except Exception:
        pass

    cv2.putText(img,"Mapa — LIDAR",(15,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(220,220,220),2)
    cv2.imshow("LiDAR - zonas", img)

# ══════════════════════════════════════════════════════════════════════════════
#  GEOMETRÍA / ODOMETRÍA
# ══════════════════════════════════════════════════════════════════════════════

def get_robot():
    s = controller.get_state()
    return s['base_x'], s['base_y'], s['base_theta']

def rel_angle(ox, oy, rx, ry, rt):
    dx, dy = ox-rx, oy-ry
    a = math.atan2(dy, dx) - rt
    return (a + math.pi) % (2*math.pi) - math.pi

def distance(ox, oy, rx, ry):
    return math.sqrt((ox-rx)**2 + (oy-ry)**2)

# ══════════════════════════════════════════════════════════════════════════════
#  BUCLE DE CONTROL (tick + run_timed)
# ══════════════════════════════════════════════════════════════════════════════

pausado = [False]

def tick(cmd):
    controller.set_velocities(cmd)
    if SHOW_CAMERA:
        f = HEAD_RGB_CAMERA.get_frame()
        if f is not None:
            if SHOW_LABELS: f = draw_medicine_labels(f)
            cv2.imshow("Head RGB", f)
        draw_lidar_view(get_robot)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): raise KeyboardInterrupt
        if k == ord(' '):
            pausado[0] = not pausado[0]
            print("\nPAUSADO" if pausado[0] else "\nContinuando...")
    while pausado[0]:
        controller.set_velocities({})
        if SHOW_CAMERA:
            f = HEAD_RGB_CAMERA.get_frame()
            if f is not None:
                if SHOW_LABELS: f = draw_medicine_labels(f)
                cv2.putText(f,"PAUSADO",(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                cv2.imshow("Head RGB", f)
            draw_lidar_view(get_robot)
            if cv2.waitKey(50) & 0xFF == ord(' '): pausado[0] = False
        time.sleep(0.05)
    time.sleep(1/20)

def run_timed(dur, cmd, label=""):
    t0 = time.time()
    while time.time()-t0 < dur:
        print(f"\r    {label}  {time.time()-t0:.1f}/{dur:.1f}s", end="", flush=True)
        tick(cmd)
    controller.set_velocities({}); print()

# ══════════════════════════════════════════════════════════════════════════════
#  FASE: NAVEGACIÓN AL FLANCO  (Sección V del reporte)
# ══════════════════════════════════════════════════════════════════════════════

def fase_orientacion(ox, oy):
    """Orientación inicial: gira hasta que el objeto quede al frente."""
    print("  [Navegación] Orientación inicial...")
    while True:
        rx, ry, rt = get_robot()
        ae = rel_angle(ox, oy, rx, ry, rt)
        if abs(ae) < 0.12: break
        spd  = max(0.20, min(0.60, abs(ae) * Kp_angle))
        sign = 1 if ae > 0 else -1
        print(f"\r    angle_rel={math.degrees(ae):+.1f}°", end="", flush=True)
        tick({"base_counterclockwise": -sign * spd})
    controller.set_velocities({}); print()

def fase_aproximacion(ox, oy):
    """Aproximación: avanza o retrocede hasta llegar a FLANK_DIST."""
    print("  [Navegación] Aproximación...")
    while True:
        rx, ry, rt = get_robot()
        d   = distance(ox, oy, rx, ry)
        err = d - FLANK_DIST
        if abs(err) < FLANK_TOL: break
        ae  = rel_angle(ox, oy, rx, ry, rt)
        spd = max(0.10, min(0.40, abs(err) * Kp_forward))
        fwd = spd if err > 0 else -spd
        print(f"\r    dist={d:.3f}m  err={err:+.3f}m", end="", flush=True)
        tick({"base_forward": fwd, "base_counterclockwise": -Kp_angle * ae * 0.5})
    controller.set_velocities({}); print()

def fase_flanco(ox, oy):
    """Flanco: coloca el objeto en el costado derecho del robot."""
    print("  [Navegación] Maniobra de flanco...")
    TARGET = -math.pi/2 + math.radians(FLANK_OFFSET_DEG)
    while True:
        rx, ry, rt = get_robot()
        ae  = rel_angle(ox, oy, rx, ry, rt)
        err = ae - TARGET
        if abs(err) < ANGLE_TOL: break
        spd  = max(0.12, min(0.45, abs(err) * Kp_angle))
        sign = 1 if err > 0 else -1
        print(f"\r    flank_err={math.degrees(err):+.1f}°  target={math.degrees(TARGET):+.1f}°",
              end="", flush=True)
        tick({"base_counterclockwise": -sign * spd})
    controller.set_velocities({}); print()

# ══════════════════════════════════════════════════════════════════════════════
#  FASE: AGARRE  (Sección VI del reporte — Tabla III)
# ══════════════════════════════════════════════════════════════════════════════

def fase_agarre(obj_z, flank_d):
    """
    Secuencia de manipulación según el reporte:
      1. Preparación     — postura segura (stow)
      2. Ajuste de altura — lift a altura del objeto
      3. Pre-agarre       — apertura moderada de pinza
      4. Alcance          — brazo se extiende al rango de contacto
      5. Cierre           — pinza sujeta el insumo
      6. Levantamiento    — lift sube para separar el objeto de la mesa
      7. Retracción       — brazo retrae para transporte seguro
    """
    grab_z      = max(0.02, obj_z - GRAB_LIFT_OFFSET)
    arm_alcance = min(0.50, max(0.30, flank_d - 0.02))

    print(f"\n  [Agarre] lift→{grab_z:.2f}m  brazo→{arm_alcance:.2f}m")

    # 1. Preparación
    print("    1. Preparación — postura segura")
    stow_grab = StateController(controller, {"arm_out": 0.04, "gripper_open": 0.28})
    t0 = time.time()
    while not stow_grab.is_at_goal() and time.time()-t0 < 3.0:
        tick(stow_grab.get_command())
    controller.set_velocities({})

    # 2. Ajuste de altura
    print("    2. Ajuste de altura")
    lift_pose = StateController(controller, {"lift_up": grab_z, "gripper_open": 0.28})
    t0 = time.time()
    while not lift_pose.is_at_goal() and time.time()-t0 < 4.0:
        tick(lift_pose.get_command())
    controller.set_velocities({})

    # 3. Pre-agarre (apertura moderada — no exagerada)
    print("    3. Pre-agarre")
    pregrip = StateController(controller, {"gripper_open": 0.28})
    t0 = time.time()
    while not pregrip.is_at_goal() and time.time()-t0 < 1.0:
        tick(pregrip.get_command())
    controller.set_velocities({})

    # 4. Alcance — brazo se extiende al rango de contacto con el insumo
    print(f"    4. Alcance brazo → {arm_alcance:.2f}m")
    reach_pose = StateController(controller, {"arm_out": arm_alcance, "gripper_open": 0.28})
    t0 = time.time()
    while not reach_pose.is_at_goal() and time.time()-t0 < 5.0:
        tick(reach_pose.get_command())
    controller.set_velocities({})

    # 5. Cierre
    run_timed(1.8, {"gripper_open": -1.0},                      "    5. Cierre de pinza")

    # 6. Levantamiento
    run_timed(2.0, {"lift_up": 0.55, "gripper_open": -1.0},     "    6. Levantamiento")

    # 7. Retracción
    run_timed(1.2, {"arm_out": -0.8, "gripper_open": -1.0},     "    7. Retracción")

# ══════════════════════════════════════════════════════════════════════════════
#  FASE: TRANSPORTE Y ENTREGA  (Sección VII del reporte — Tabla IV)
# ══════════════════════════════════════════════════════════════════════════════

def fase_transporte_entrega():
    """
    Secuencia de entrega según el reporte:
      1. Transporte seguro   — gira 180° hacia zona_deje
      2. Ajuste de depósito  — lift baja a altura de entrega
      3. Alcance de depósito — brazo extiende hacia superficie
      4. Liberación          — pinza abre y suelta el insumo
      5. Retorno seguro      — brazo retrae, robot queda estable
    """
    print("\n  [Transporte y Entrega]")
    run_timed(4.50, {"base_counterclockwise": 0.95, "gripper_open": -1.0},
              "    1. Transporte seguro → zona_deje")
    run_timed(0.55, {"lift_up": -0.25, "gripper_open": -1.0},
              "    2. Ajuste de depósito")
    run_timed(2.00, {"arm_out": 0.35,  "gripper_open": -1.0},
              "    3. Alcance de depósito")
    run_timed(1.00, {"gripper_open": 1.0},
              "    4. Liberación del insumo")
    run_timed(1.00, {"arm_out": -0.6},
              "    5. Retorno seguro")

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # ── Preparación ───────────────────────────────────────────────────────────
    print("  [Preparación] Postura inicial segura...")
    stow = StateController(controller, {
        "wrist_roll_counterclockwise": 0.0,
        "wrist_yaw_counterclockwise":  0.0,
        "wrist_pitch_up": 0.0,
        "gripper_open":   0.5,
        "arm_out":        0.0,
    })
    t0 = time.time()
    while not stow.is_at_goal() and time.time()-t0 < 4.0:
        tick(stow.get_command())
    controller.set_velocities({})

    # Inclinar cámara levemente hacia la mesa
    run_timed(1.0, {"head_tilt_up": -0.25}, "  [Percepción] Inclinando cámara")

    # ── Percepción / Localización ──────────────────────────────────────────────
    print("\n  [Percepción] Localizando insumos en escena...")
    objects = controller.list_scene_objects()
    if not objects:
        print("No hay objetos en la escena."); return

    print("  Posiciones:")
    for obj in objects:
        p = controller.get_object_pose(obj)
        if p:
            print(f"    {obj}: x={p['x']:.3f}  y={p['y']:.3f}  z={p['z']:.3f}")

    grabables = [o for o in objects
                 if (lambda p: p is not None and p['z'] > 0.05)
                    (controller.get_object_pose(o))]

    if TARGET_NAME and TARGET_NAME in objects:
        target = TARGET_NAME
    elif len(grabables) == 1:
        target = grabables[0]
    else:
        print("\n  Objetos agarrables:", grabables)
        for i, obj in enumerate(grabables):
            p = controller.get_object_pose(obj)
            print(f"    {i}: {obj}  ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})")
        entrada = input("  Nombre o número: ").strip()
        target  = grabables[int(entrada)] if entrada.isdigit() else entrada

    pose       = controller.get_object_pose(target)
    ox, oy, oz = pose['x'], pose['y'], pose['z']
    print(f"\n  Objetivo: {target}  →  ({ox:.3f}, {oy:.3f}, {oz:.3f})")

    # ── Cálculo geométrico ────────────────────────────────────────────────────
    rx, ry, _ = get_robot()
    d0 = distance(ox, oy, rx, ry)
    a0 = rel_angle(ox, oy, rx, ry, _)
    print(f"  [Cálculo geométrico] dist={d0:.3f}m  angle_rel={math.degrees(a0):.1f}°")

    print("\n  Arrancando en 2s...  ESPACIO=pausar  q=parar\n")
    time.sleep(2)

    try:
        # ── Navegación al flanco ──────────────────────────────────────────────
        fase_orientacion(ox, oy)
        fase_aproximacion(ox, oy)
        fase_flanco(ox, oy)

        # ── Agarre ────────────────────────────────────────────────────────────
        rx, ry, _ = get_robot()
        fase_agarre(oz, distance(ox, oy, rx, ry))

        # ── Transporte y entrega ──────────────────────────────────────────────
        fase_transporte_entrega()

        print("\n  ✓ Misión completada.")

    except KeyboardInterrupt:
        print("\n  Misión interrumpida.")
    finally:
        controller.set_velocities({})
        try: controller.stop()
        except: pass
        cv2.destroyAllWindows()
        print("  Robot detenido.")


if __name__ == "__main__":
    main()
