"""
Controlador para el sistema de péndulo invertido sobre carro móvil
"""

from controller import Robot, Keyboard
import math

# Crear instancia del robot
robot = Robot()

# Obtener el paso de tiempo de la simulación
timestep = int(robot.getBasicTimeStep())

# Obtener dispositivos
carro_motor = robot.getDevice("carro_motor")
carro_sensor = robot.getDevice("carro_sensor")
pendulo_sensor = robot.getDevice("pendulo_sensor")
pendulo_motor = robot.getDevice("pendulo_motor")

# Habilitar sensores
carro_sensor.enable(timestep)
pendulo_sensor.enable(timestep)

# Configurar motor del carro
carro_motor.setPosition(float('inf'))  # Modo velocidad
carro_motor.setVelocity(0)  # Velocidad inicial

# Configurar teclado para control manual
keyboard = Keyboard()
keyboard.enable(timestep)

print("=== CONTROL DEL PÉNDULO INVERTIDO ===")
print("Teclas de control:")
print("  A / ← : Mover carro hacia la izquierda")
print("  D / → : Mover carro hacia la derecha") 
print("  S / ↓ : Parar carro")
print("  SPACE : Activar control automático (PID)")
print("  Q     : Salir")

# Variables para el controlador PID
kp = 100.0  # Aumentado de 50.0 
kd = 50.0   # Aumentado de 25.0
ki = 2.0    # Aumentado de 1.0

prev_angle = 0
integral = 0
auto_control = False
target_position = 0  # Posición objetivo del carro

def control_manual():
    """Control manual del carro usando el teclado"""
    key = keyboard.getKey()
    velocidad = 0
    
    if key == ord('A') or key == 314:  # A o flecha izquierda
        velocidad = -5.0  # Aumentado de -2.0
        print("Moviendo izquierda")
    elif key == ord('D') or key == 316:  # D o flecha derecha
        velocidad = 5.0  # Aumentado de 2.0
        print("Moviendo derecha")
    elif key == ord('S') or key == 317:  # S o flecha abajo
        velocidad = 0
        print("Deteniendo carro")
    elif key == ord(' '):  # SPACE
        global auto_control
        auto_control = not auto_control
        velocidad = 0
        print(f"Control automático: {'ON' if auto_control else 'OFF'}")
    elif key == ord('Q'):
        print("Saliendo...")
        return False, 0
    
    return True, velocidad

def control_pid():
    """Control PID para estabilizar el péndulo"""
    global prev_angle, integral, target_position
    
    # Leer sensores
    pendulo_angle = pendulo_sensor.getValue()
    carro_pos = carro_sensor.getValue()
    
    # Normalizar ángulo del péndulo (0 = vertical hacia arriba)
    angle_error = pendulo_angle
    if angle_error > math.pi:
        angle_error -= 2 * math.pi
    elif angle_error < -math.pi:
        angle_error += 2 * math.pi
    
    # Error de posición del carro
    position_error = carro_pos - target_position
    
    # Calcular derivada del ángulo
    angle_derivative = angle_error - prev_angle
    prev_angle = angle_error
    
    # Actualizar integral
    integral += angle_error
    
    # Limitar integral para evitar windup
    if integral > 100:
        integral = 100
    elif integral < -100:
        integral = -100
    
    # Control PID combinado (ángulo + posición)
    control_signal = (kp * angle_error + 
                     kd * angle_derivative + 
                     ki * integral +
                     5.0 * position_error)  # Factor adicional para posición
    
    # Limitar la señal de control
    max_force = 200  # Aumentado de 50
    if control_signal > max_force:
        control_signal = max_force
    elif control_signal < -max_force:
        control_signal = -max_force
    
    # Convertir fuerza a velocidad (aumentado el factor)
    velocidad = -control_signal * 0.05  # Cambiado de 0.1 a 0.05
    
    return velocidad

# Bucle principal de control
while robot.step(timestep) != -1:
    # Leer estado actual
    pendulo_angle = pendulo_sensor.getValue()
    carro_pos = carro_sensor.getValue()
    
    # Mostrar información cada 50 pasos (aproximadamente cada segundo)
    if robot.getTime() % 1.0 < 0.016:  # timestep = 16ms
        angle_degrees = math.degrees(pendulo_angle)
        print(f"Posición carro: {carro_pos:.3f}m | Ángulo péndulo: {angle_degrees:.1f}°")
    
    if auto_control:
        # Control automático PID
        velocidad = control_pid()
        carro_motor.setVelocity(velocidad)
    else:
        # Control manual
        continuar, velocidad = control_manual()
        if not continuar:
            break
        carro_motor.setVelocity(velocidad)

print("Simulación terminada")