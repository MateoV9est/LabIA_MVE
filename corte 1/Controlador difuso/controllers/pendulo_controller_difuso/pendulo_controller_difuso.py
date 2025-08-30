"""
Controlador DIFUSO para péndulo invertido lineal
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

# Variables para control difuso
prev_angle = 0
auto_control = False
target_position = 0

def get_carro_position():
    return carro_sensor.getValue()

# ===== CONTROLADOR DIFUSO =====

# Funciones de pertenencia triangular y trapezoidal
def trimf(x, params):
    """Función de pertenencia triangular [a, b, c]"""
    a, b, c = params
    if x <= a or x >= c:
        return 0.0
    elif x <= b:
        return (x - a) / (b - a)
    else:
        return (c - x) / (c - b)

def trapmf(x, params):
    """Función de pertenencia trapezoidal [a, b, c, d]"""
    a, b, c, d = params
    if x <= a or x >= d:
        return 0.0
    elif x <= b:
        return (x - a) / (b - a)
    elif x <= c:
        return 1.0
    else:
        return (d - x) / (d - a)

# Definir conjuntos difusos MÁS SUAVES para evitar oscilaciones
def fuzzify_angle(angle):
    """Fuzzifica el ángulo - BALANCEADO"""
    # Convertir a grados para usar los parámetros
    angle_deg = math.degrees(abs(angle))
    
    # Conjuntos más suaves para evitar cambios bruscos
    
    if angle < 0:  # Ángulo negativo
        # AN - Ángulo Negativo pequeño 
        AN = trapmf(angle_deg, [0, 1, 5, 10])  # Más suave
        # AMN - Ángulo Mayor Negativo
        AMN = trapmf(angle_deg, [8, 12, 25, 45])  # Con superposición
        return {'AN': AN, 'AMN': AMN, 'AP': 0.0, 'AMP': 0.0}
    else:  # Ángulo positivo
        # AP - Ángulo Positivo pequeño
        AP = trapmf(angle_deg, [0, 1, 5, 10])  # Más suave
        # AMP - Ángulo Mayor Positivo  
        AMP = trapmf(angle_deg, [8, 12, 25, 45])  # Con superposición
        return {'AN': 0.0, 'AMN': 0.0, 'AP': AP, 'AMP': AMP}

def fuzzify_angular_velocity(vel):
    """Fuzzifica la velocidad angular - BALANCEADO"""
    vel_abs = abs(vel)
    # Parámetros balanceados para estabilidad
    # VAN: Velocidad Angular Negativa
    VAN = trapmf(vel_abs, [0.05, 0.15, 0.8, 2.0]) if vel < 0 else 0.0
    # VAP: Velocidad Angular Positiva  
    VAP = trapmf(vel_abs, [0.05, 0.15, 0.8, 2.0]) if vel > 0 else 0.0
    # Velocidad Cero - Rango razonable
    CERO = trimf(vel_abs, [0, 0, 0.3])  # Más amplio para estabilidad
    
    return {'VAN': VAN, 'VAP': VAP, 'CERO': CERO}

def fuzzify_position(pos):
    """Fuzzifica la posición del carrito"""
    # Basado en encuesta XMP/XP promedio ~2.02
    pos_abs = abs(pos)
    
    if pos < 0:  # Posición negativa (izquierda)
        XN = trapmf(pos_abs, [0, 0.5, 1.5, 2.5])  # Cerca del centro
        XMN = trapmf(pos_abs, [1.5, 2.0, 3.0, 5.0])  # Lejos del centro
        return {'XN': XN, 'XMN': XMN, 'XP': 0.0, 'XMP': 0.0}
    else:  # Posición positiva (derecha)
        XP = trapmf(pos_abs, [0, 0.5, 1.5, 2.5])
        XMP = trapmf(pos_abs, [1.5, 2.0, 3.0, 5.0])
        return {'XN': 0.0, 'XMN': 0.0, 'XP': XP, 'XMP': XMP}

def apply_fuzzy_rules(angle_fuzzy, vel_fuzzy, pos_fuzzy):
    """Aplica las reglas difusas - BALANCEADO Y COHERENTE"""
    rules_output = []
    
    # LÓGICA PRINCIPAL: Control clásico de péndulo invertido
    # Si el péndulo se inclina hacia un lado, mover el carrito al mismo lado
    
    # Regla básica 1: Ángulo negativo (izquierda) → Mover carrito a la izquierda
    if angle_fuzzy['AN'] > 0.1:
        # Pero considerar la velocidad: si ya se está enderezando, suavizar
        if vel_fuzzy['VAP'] > 0.3:  # Se está enderezando (velocidad hacia arriba)
            strength = angle_fuzzy['AN'] * (1.0 - vel_fuzzy['VAP'] * 0.5)
            rules_output.append((strength, -2.0))  # Mover izquierda suave
        else:
            rules_output.append((angle_fuzzy['AN'], -3.0))  # Mover izquierda normal
    
    # Regla básica 2: Ángulo positivo (derecha) → Mover carrito a la derecha  
    if angle_fuzzy['AP'] > 0.1:
        # Considerar la velocidad: si ya se está enderezando, suavizar
        if vel_fuzzy['VAN'] > 0.3:  # Se está enderezando (velocidad hacia arriba)
            strength = angle_fuzzy['AP'] * (1.0 - vel_fuzzy['VAN'] * 0.5)
            rules_output.append((strength, 2.0))  # Mover derecha suave
        else:
            rules_output.append((angle_fuzzy['AP'], 3.0))  # Mover derecha normal
    
    # Reglas para ángulos grandes - MÁS AGRESIVAS pero coherentes
    if angle_fuzzy['AMN'] > 0.1:  # Ángulo grande negativo
        rules_output.append((angle_fuzzy['AMN'], -4.0))  # Mover izquierda fuerte
        
    if angle_fuzzy['AMP'] > 0.1:  # Ángulo grande positivo
        rules_output.append((angle_fuzzy['AMP'], 4.0))  # Mover derecha fuerte
    
    # Control de posición - MUY SUAVE para no interferir
    if pos_fuzzy['XMN'] > 0.3:  # Solo si está MUY lejos
        rules_output.append((pos_fuzzy['XMN'] * 0.2, 0.5))  # Muy suave hacia el centro
        
    if pos_fuzzy['XMP'] > 0.3:  # Solo si está MUY lejos
        rules_output.append((pos_fuzzy['XMP'] * 0.2, -0.5))  # Muy suave hacia el centro
    
    # REGLA DE SEGURIDAD mejorada
    if len(rules_output) == 0:
        angle_rad = angle_fuzzy.get('raw_angle', 0)
        if abs(angle_rad) > 0.01:  # 0.57 grados
            # Control proporcional simple
            control = -angle_rad * 2.0  # Proporcional inverso
            rules_output.append((0.8, max(min(control, 4.0), -4.0)))
    
    return rules_output

def defuzzify(rules_output):
    """Defuzzificación usando método del centroide - RESPETA MAX VELOCITY"""
    if not rules_output:
        return 0.0
    
    numerator = sum(strength * output for strength, output in rules_output)
    denominator = sum(strength for strength, output in rules_output)
    
    if denominator == 0:
        return 0.0
    
    result = numerator / denominator
    # Respetar el límite máximo del motor (maxVelocity = 5 en Webots)
    return max(min(result, 4.8), -4.8)  # 4.8 para margen de seguridad

def normalize_angle(angle):
    """Normaliza el ángulo para que esté entre -π y π radianes (CORREGIDO)"""
    # Normalizar usando módulo para manejar múltiples vueltas
    normalized = ((angle + math.pi) % (2 * math.pi)) - math.pi
    return normalized

def control_fuzzy():
    """Controlador principal difuso - CON DEBUG"""
    global prev_angle
    
    # Leer sensores
    raw_angle = sensor_pendulo.getValue()
    pos = get_carro_position()
    
    # NORMALIZACIÓN CORRECTA para manejar múltiples vueltas
    angle = normalize_angle(raw_angle)
    
    # Calcular velocidad angular con mejor filtrado
    dt = timestep / 1000.0
    if dt > 0:
        angular_velocity = (angle - prev_angle) / dt
        
        # Manejo inteligente de saltos de -π a π
        if angular_velocity > math.pi / dt:  # Salto de -π a π
            angular_velocity -= 2 * math.pi / dt
        elif angular_velocity < -math.pi / dt:  # Salto de π a -π
            angular_velocity += 2 * math.pi / dt
    else:
        angular_velocity = 0.0
    
    prev_angle = angle
    
    # Error de posición respecto al objetivo
    pos_error = pos - target_position
    
    # Fuzzificación
    angle_fuzzy = fuzzify_angle(angle)
    vel_fuzzy = fuzzify_angular_velocity(angular_velocity)
    pos_fuzzy = fuzzify_position(pos_error)
    
    # AÑADIR el ángulo crudo para la regla de seguridad
    angle_fuzzy['raw_angle'] = angle
    
    # Aplicar reglas difusas
    rules_output = apply_fuzzy_rules(angle_fuzzy, vel_fuzzy, pos_fuzzy)
    
    # DEBUG - Imprimir si no hay reglas activas
    if len(rules_output) == 0:
        print(f"🚨 SIN REGLAS! Ángulo: {math.degrees(angle):.1f}°, Vel: {angular_velocity:.3f}")
    
    # Defuzzificación
    control_signal = defuzzify(rules_output)
    
    # DEBUG - Si la señal es cero
    if abs(control_signal) < 0.1:
        print(f"⚠️  SEÑAL DÉBIL: {control_signal:.3f} | Reglas: {len(rules_output)}")
    
    return control_signal

# ===== FIN CONTROLADOR DIFUSO =====

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
        print(f"🔧 Control {'AUTOMÁTICO (DIFUSO)' if auto_control else 'MANUAL'}")
    
    return velocidad

# Bucle principal
print("\n🎮 Presiona A/D para probar movimiento...")
counter = 0

while robot.step(timestep) != -1:
    counter += 1
    
    # Mostrar info cada 2 segundos
    if counter % 125 == 0:
        raw_angle = sensor_pendulo.getValue()
        normalized_angle = normalize_angle(raw_angle)
        angle_deg = math.degrees(normalized_angle)
        pos = get_carro_position()
        mode = "AUTO-DIFUSO" if auto_control else "MANUAL"
        print(f"[{mode}] Pos: {pos:.3f}m | Ángulo: {angle_deg:.1f}°")
    
    # Aplicar control
    if auto_control:
        velocidad = control_fuzzy()  # ← Cambiado de control_pid() a control_fuzzy()
        carro_motor.setVelocity(velocidad)
    else:
        velocidad = control_manual()
        carro_motor.setVelocity(velocidad)

print("Simulación terminada")
carro_motor.setVelocity(0.0)