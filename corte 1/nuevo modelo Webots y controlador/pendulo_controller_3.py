"""
Controlador DEFINITIVO para péndulo invertido lineal
"""

from controller import Robot, Keyboard
import math

# Crear instancia del robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Obtener dispositivos - VERIFICAR NOMBRES EXACTOS
try:
    carro_motor = robot.getDevice("carro_motor")
    carro_sensor = robot.getDevice("carro_sensor") 
    sensor_pendulo = robot.getDevice("sensor_pendulo")
    print("✅ Dispositivos encontrados correctamente")
except:
    print("❌ Error: Revisa los nombres de los dispositivos en el mundo VRML")
    exit()

# Habilitar sensores
carro_sensor.enable(timestep)
sensor_pendulo.enable(timestep)

# Configurar motor en modo VELOCIDAD (más confiable)
carro_motor.setPosition(float('inf'))
carro_motor.setVelocity(0.0)

# Teclado
keyboard = Keyboard()
keyboard.enable(timestep)

print("=== CONTROL PÉNDULO LINEAL ===")
print("Teclas: A/Izq ← | D/Der → | S/Abajo ↓ | ESPACIO: Auto/Manual")

# Parámetros PID
kp = 12.0
kd = 6.0
ki = 0.5
kp_pos = 3.0

prev_angle = 0
integral = 0
auto_control = False
target_position = 0

def get_carro_position():
    return carro_sensor.getValue()

def control_manual():
    global auto_control, target_position
    
    key = keyboard.getKey()
    velocidad = 0.0
    
    if key == ord('A') or key == 314:  # Izquierda
        velocidad = -2.0
        print("← Moviendo IZQUIERDA")
    elif key == ord('D') or key == 316:  # Derecha
        velocidad = 2.0
        print("→ Moviendo DERECHA")
    elif key == ord('S') or key == 317:  # Detener
        velocidad = 0.0
        print("⏹ Detenido")
    elif key == ord(' '):  # ESPACIO
        auto_control = not auto_control
        if auto_control:
            target_position = get_carro_position()
        print(f"🔧 Control {'AUTOMÁTICO' if auto_control else 'MANUAL'}")
    
    return velocidad

def control_pid():
    global prev_angle, integral
    
    # Leer sensores
    angle = sensor_pendulo.getValue()
    pos = get_carro_position()
    
    # Normalizar ángulo
    angle_error = angle
    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi
    
    # Error de posición
    pos_error = pos - target_position
    
    # Derivada
    dt = timestep / 1000.0
    angle_deriv = (angle_error - prev_angle) / dt
    prev_angle = angle_error
    
    # Integral
    integral += angle_error * dt
    integral = max(min(integral, 10.0), -10.0)
    
    # Control PID
    control_signal = (kp * angle_error + kd * angle_deriv + ki * integral + 
                     kp_pos * pos_error)
    
    # Limitar velocidad
    return max(min(control_signal, 3.0), -3.0)

# Bucle principal
print("\n🎮 Presiona A/D para probar movimiento...")
counter = 0

while robot.step(timestep) != -1:
    counter += 1
    
    # Mostrar info cada 2 segundos
    if counter % 125 == 0:
        angle_deg = math.degrees(sensor_pendulo.getValue())
        pos = get_carro_position()
        mode = "AUTO" if auto_control else "MANUAL"
        print(f"[{mode}] Pos: {pos:.3f}m | Ángulo: {angle_deg:.1f}°")
    
    # Aplicar control
    if auto_control:
        velocidad = control_pid()
        carro_motor.setVelocity(velocidad)
    else:
        velocidad = control_manual()
        carro_motor.setVelocity(velocidad)

print("Simulación terminada")
carro_motor.setVelocity(0.0)