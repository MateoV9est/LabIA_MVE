import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from scipy.integrate import solve_ivp
import matplotlib.patches as patches

import os
os.system('cls')  # Para Windows

class SquarePendulum:
    def __init__(self):
        #######################################################
        # PARÁMETROS FÍSICOS EDITABLES (UNIDADES SI)
        #######################################################
        
        # Gravedad (m/s²)
        self.g = 9.81
        
        # Masa de la base cuadrada (kg) - base móvil
        self.M = 2.0
        
        # Masa del extremo del brazo (kg) - masa concentrada en el extremo
        self.m = 0.3
        
        # Longitud del brazo rígido (m)
        self.L = 0.8
        
        # Coeficiente de fricción de la base (N·s/m)
        self.b = 0.05
        
        # NUEVOS COEFICIENTES DE AMORTIGUAMIENTO REALISTAS
        # Fricción en el pivote del brazo (N·m·s/rad) - resistencia rotacional
        self.b_pivot = 0.008  # Fricción en el eje del péndulo
        
        # Resistencia del aire sobre la masa (kg/s) - drag lineal
        self.b_air_linear = 0.002  # Resistencia proporcional a la velocidad
        
        # Resistencia del aire cuadrática (kg/m) - drag cuadrático
        self.b_air_quad = 0.001  # Resistencia proporcional al cuadrado de la velocidad
        
        # Momento de inercia del brazo respecto al pivote (kg·m²)
        self.I = self.m * self.L**2  # Masa puntual en el extremo
        
        #######################################################
        # PARÁMETROS DE SIMULACIÓN Y CONTROL
        #######################################################
        
        # Paso de tiempo (s)
        self.dt = 0.01
        
        # Fuerza aplicada por las flechas del teclado (N)
        self.control_force = 0.0
        
        # Magnitud de fuerza por tecla presionada (N)
        self.force_magnitude = 15.0
        
        # Límites de movimiento de la base (m)
        self.max_base_pos = 3.0
        
        #######################################################
        # ESTADO INICIAL DEL SISTEMA
        #######################################################
        
        # Estado: [x_base, v_base, theta, omega]
        # x_base: posición de la base (m)
        # v_base: velocidad de la base (m/s) 
        # theta: ángulo del brazo respecto a la vertical (rad) - 0 = hacia arriba
        # omega: velocidad angular del brazo (rad/s)
        self.state = np.array([0.0, 0.0, np.pi + 0.05, 0.0])  # Pequeña perturbación desde arriba
        
        #######################################################
        # CONFIGURACIÓN VISUAL
        #######################################################
        
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 10), facecolor='#1a1a1a')
        self.ax = plt.axes([0.08, 0.15, 0.84, 0.75])
        
        # Configuración de ejes
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-0.3, 1.8)
        self.ax.set_facecolor('#0d1117')
        self.ax.grid(True, linestyle='--', alpha=0.3, color='gray')
        self.ax.set_title('🎯 Péndulo Invertido con Base Cuadrada Móvil', 
                         fontsize=16, pad=20, color='white', weight='bold')
        self.ax.set_xlabel('Posición (m)', fontsize=12, color='white')
        self.ax.set_ylabel('Altura (m)', fontsize=12, color='white')
        
        # Hacer que las escalas sean proporcionales (grid cuadrado perfecto)
        self.ax.set_aspect('equal', adjustable='box')
        
        #######################################################
        # ELEMENTOS GRÁFICOS
        #######################################################
        
        # Superficie de deslizamiento (piso)
        self.ground = patches.Rectangle((-4, -0.15), 8, 0.1, 
                                      linewidth=2, edgecolor='#444444', 
                                      facecolor='#333333', zorder=1)
        self.ax.add_patch(self.ground)
        
        # Base cuadrada móvil (cuadrado verde)
        self.base_size = 0.2
        self.base = patches.Rectangle((-self.base_size/2, -0.05), 
                                    self.base_size, self.base_size,
                                    linewidth=3, edgecolor='#00ff41', 
                                    facecolor='#004d0d', zorder=5)
        self.ax.add_patch(self.base)
        
        # Brazo rígido (línea gruesa azul)
        self.arm, = self.ax.plot([], [], color='#00bfff', linewidth=6, 
                               solid_capstyle='round', zorder=3)
        
        # Masa en el extremo (círculo rojo grande)
        self.mass = patches.Circle((0, 0), 0.08, 
                                 facecolor='#ff4444', edgecolor='#ffffff', 
                                 linewidth=2, zorder=10)
        self.ax.add_patch(self.mass)
        
        # Punto de pivote (pequeño círculo blanco en el centro de la base)
        self.pivot = patches.Circle((0, self.base_size/2 - 0.05), 0.02,
                                  facecolor='white', edgecolor='black', zorder=8)
        self.ax.add_patch(self.pivot)
        
        #######################################################
        # PANEL DE INFORMACIÓN
        #######################################################
        
        self.info_text = self.ax.text(
            0.02, 0.98,
            '',
            transform=self.ax.transAxes,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#1a1a1a', 
                    edgecolor='#00ff41', alpha=0.9),
            fontfamily='monospace',
            fontsize=11,
            color='#00ff41',
            verticalalignment='top'
        )
        
        #######################################################
        # INSTRUCCIONES DE CONTROL
        #######################################################
        
        self.instructions = self.ax.text(
            0.98, 0.98,
            'CONTROLES:\n'
            '← → Flechas para mover base\n'
            'Objetivo: Mantener brazo vertical ↑',
            transform=self.ax.transAxes,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#1a1a1a', 
                    edgecolor='#00bfff', alpha=0.9),
            fontsize=10,
            color='#00bfff',
            verticalalignment='top',
            horizontalalignment='right'
        )
        
        #######################################################
        # BOTONES DE CONTROL
        #######################################################
        
        # Botón de reinicio
        self.reset_ax = plt.axes([0.25, 0.02, 0.12, 0.08])
        self.btn_reset = Button(self.reset_ax, '🔄 REINICIAR', 
                              color='#ff4444', hovercolor='#ff6666')
        self.btn_reset.label.set_fontweight('bold')
        self.btn_reset.label.set_color('white')
        self.btn_reset.on_clicked(self.reset_simulation)
        
        # Botón para cambiar posición inicial
        self.change_ax = plt.axes([0.39, 0.02, 0.12, 0.08])
        self.btn_change = Button(self.change_ax, '🎲 ALEATORIO', 
                               color='#4444ff', hovercolor='#6666ff')
        self.btn_change.label.set_fontweight('bold')
        self.btn_change.label.set_color('white')
        self.btn_change.on_clicked(self.random_start)
        
        # Botón para mostrar ecuaciones matemáticas
        self.equations_ax = plt.axes([0.53, 0.02, 0.15, 0.08])
        self.btn_equations = Button(self.equations_ax, '📚 ECUACIONES', 
                                  color='#9b59b6', hovercolor='#8e44ad')
        self.btn_equations.label.set_fontweight('bold')
        self.btn_equations.label.set_color('white')
        self.btn_equations.on_clicked(lambda event: self.show_mathematical_model())
        
        #######################################################
        # EVENTOS DE TECLADO
        #######################################################
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        
        # Variables de control de teclas
        self.keys_pressed = set()
    
    def system_dynamics(self, t, y, F):
        """
        Modelo dinámico del péndulo invertido con base cuadrada móvil.
        INCLUYE EFECTOS DISIPATIVOS REALISTAS.
        
        Variables de estado:
        y = [x, x_dot, theta, theta_dot]
        donde:
        - x: posición de la base
        - x_dot: velocidad de la base
        - theta: ángulo del brazo (0 = vertical hacia arriba)
        - theta_dot: velocidad angular del brazo
        """
        x, x_dot, theta, theta_dot = y
        
        # Términos trigonométricos
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # CÁLCULO DE FUERZAS DISIPATIVAS REALISTAS
        
        # 1. Fricción en el pivote (proporcional a velocidad angular)
        torque_pivot_friction = -self.b_pivot * theta_dot
        
        # 2. Velocidad de la masa en el extremo (para resistencia del aire)
        # Velocidad horizontal de la masa
        mass_vel_x = x_dot + self.L * theta_dot * cos_theta
        # Velocidad vertical de la masa  
        mass_vel_y = -self.L * theta_dot * sin_theta
        # Velocidad total de la masa
        mass_speed = np.sqrt(mass_vel_x**2 + mass_vel_y**2)
        
        # 3. Resistencia del aire sobre la masa (lineal + cuadrática)
        # Componente horizontal de la resistencia
        if mass_speed > 1e-6:  # Evitar división por cero
            air_drag_x = -(self.b_air_linear * mass_vel_x + 
                          self.b_air_quad * mass_vel_x * mass_speed)
            air_drag_y = -(self.b_air_linear * mass_vel_y + 
                          self.b_air_quad * mass_vel_y * mass_speed)
        else:
            air_drag_x = 0
            air_drag_y = 0
        
        # 4. Torque por resistencia del aire (momento respecto al pivote)
        # Brazo de momento horizontal: L * cos(theta)
        # Brazo de momento vertical: L * sin(theta)
        torque_air = (air_drag_x * self.L * cos_theta + 
                     air_drag_y * self.L * sin_theta)
        
        # ECUACIONES DE MOVIMIENTO MODIFICADAS
        
        # Términos dinámicos con amortiguamiento
        # Denominador común
        denom = self.M + self.m - self.m * cos_theta**2
        
        # Fuerza horizontal efectiva sobre la masa (incluye resistencia del aire)
        F_mass_horizontal = air_drag_x
        
        # Aceleración de la base (x_ddot) - CON AMORTIGUAMIENTO
        numerator_x = (F - self.b * x_dot + 
                      self.m * self.L * theta_dot**2 * sin_theta - 
                      self.m * self.g * sin_theta * cos_theta +
                      F_mass_horizontal * cos_theta)  # Efecto de resistencia del aire
        
        x_ddot = numerator_x / denom
        
        # Aceleración angular del brazo (theta_ddot) - CON AMORTIGUAMIENTO COMPLETO
        # Torque gravitacional
        torque_gravity = self.m * self.g * self.L * sin_theta
        
        # Torque por aceleración de la base
        torque_base_accel = -self.m * self.L * x_ddot * cos_theta
        
        # Torque total con amortiguamiento
        total_torque = (torque_gravity + torque_base_accel + 
                       torque_pivot_friction + torque_air)
        
        # Momento de inercia efectivo
        I_effective = self.m * self.L**2
        
        theta_ddot = total_torque / I_effective
        
        return [x_dot, x_ddot, theta_dot, theta_ddot]
    
    def update_animation(self, frame):
        """Actualiza la animación en cada frame."""
        
        # Calcular fuerza total basada en teclas presionadas
        total_force = 0.0
        if 'left' in self.keys_pressed:
            total_force -= self.force_magnitude
        if 'right' in self.keys_pressed:
            total_force += self.force_magnitude
        
        self.control_force = total_force
        
        # Resolver ecuaciones diferenciales
        sol = solve_ivp(self.system_dynamics, [0, self.dt], self.state,
                       args=(self.control_force,), t_eval=[self.dt], 
                       method='RK45', rtol=1e-8)
        
        if sol.success:
            new_state = sol.y[:, -1]
            
            # Aplicar límites a la posición de la base
            if abs(new_state[0]) > self.max_base_pos:
                new_state[0] = np.sign(new_state[0]) * self.max_base_pos
                new_state[1] = 0  # Detener la base en el límite
            
            self.state = new_state
        
        # Extraer variables de estado
        x_base, v_base, theta, omega = self.state
        
        # Normalizar ángulo para visualización
        theta_normalized = ((theta + np.pi) % (2 * np.pi)) - np.pi
        
        # Calcular posiciones para la visualización
        pivot_x = x_base
        pivot_y = self.base_size/2 - 0.05  # Centro de la base
        
        mass_x = pivot_x + self.L * np.sin(theta)
        mass_y = pivot_y + self.L * np.cos(theta)
        
        # Actualizar elementos gráficos
        # Base cuadrada
        self.base.set_xy([x_base - self.base_size/2, -0.05])
        
        # Punto de pivote
        self.pivot.center = (pivot_x, pivot_y)
        
        # Brazo rígido
        self.arm.set_data([pivot_x, mass_x], [pivot_y, mass_y])
        
        # Masa en el extremo
        self.mass.center = (mass_x, mass_y)
        
        # Actualizar información
        angle_deg = np.degrees(theta_normalized)
        stability_status = "🟢 ESTABLE" if abs(angle_deg) < 5 else "🔴 INESTABLE"
        
        # Calcular energía total del sistema para mostrar la disipación
        kinetic_energy = 0.5 * self.M * v_base**2 + 0.5 * self.m * self.L**2 * omega**2
        potential_energy = self.m * self.g * self.L * (1 - np.cos(theta))
        total_energy = kinetic_energy + potential_energy
        
        info_text = (
            f'Posición base: {x_base:6.2f} m\n'
            f'Velocidad base: {v_base:6.2f} m/s\n'
            f'Ángulo brazo: {angle_deg:6.1f}°\n'
            f'Vel. angular: {np.degrees(omega):6.1f}°/s\n'
            f'Fuerza aplicada: {self.control_force:6.1f} N\n'
            f'Energía total: {total_energy:6.2f} J\n'
            f'Estado: {stability_status}'
        )
        
        self.info_text.set_text(info_text)
        
        return (self.base, self.pivot, self.arm, self.mass, self.info_text)
    
    def on_key_press(self, event):
        """Maneja las teclas presionadas."""
        if event.key == 'left':
            self.keys_pressed.add('left')
        elif event.key == 'right':
            self.keys_pressed.add('right')
    
    def on_key_release(self, event):
        """Maneja las teclas liberadas."""
        if event.key == 'left':
            self.keys_pressed.discard('left')
        elif event.key == 'right':
            self.keys_pressed.discard('right')
    
    def reset_simulation(self, event):
        """Reinicia la simulación."""
        self.state = np.array([0.0, 0.0, np.pi + 0.05, 0.0])
        self.control_force = 0.0
        self.keys_pressed.clear()
    
    def random_start(self, event):
        """Inicia con una configuración aleatoria."""
        random_x = np.random.uniform(-1.0, 1.0)
        random_theta = np.pi + np.random.uniform(-0.3, 0.3)
        random_omega = np.random.uniform(-1.0, 1.0)
        
        self.state = np.array([random_x, 0.0, random_theta, random_omega])
        self.control_force = 0.0
        self.keys_pressed.clear()
    
    def show_mathematical_model(self):
        """Muestra las ecuaciones matemáticas del péndulo en una ventana separada."""
        
        # Crear nueva figura para las ecuaciones
        fig_math = plt.figure(figsize=(12, 10), facecolor='#f8f8f8')
        fig_math.suptitle('📚 Modelo Matemático del Péndulo Invertido', 
                         fontsize=18, fontweight='bold', color='#2c3e50')
        
        # Crear subplot para las ecuaciones
        ax_math = fig_math.add_subplot(111)
        ax_math.set_xlim(0, 10)
        ax_math.set_ylim(0, 10)
        ax_math.axis('off')
        ax_math.set_facecolor('#f8f8f8')
        
        # Definición de variables
        variables_text = (
            "Variables del Sistema:\n"
            "• x = Posición horizontal de la base cuadrada (m)\n"
            "• θ = Ángulo del brazo respecto a la vertical (rad)\n"
            "• ẋ = Velocidad de la base (m/s)\n"
            "• θ̇ = Velocidad angular del brazo (rad/s)\n"
            "• F = Fuerza de control aplicada (N)\n\n"
            "Parámetros Físicos:\n"
            "• M = Masa de la base cuadrada = 2.0 kg\n"
            "• m = Masa en el extremo del brazo = 0.3 kg\n"
            "• L = Longitud del brazo rígido = 0.8 m\n"
            "• g = Aceleración gravitacional = 9.81 m/s²\n"
            "• b = Coeficiente de fricción = 0.05 N·s/m\n"
            "• Δ = Denominador común = M + m - m·cos²(θ)"
        )
        
        ax_math.text(0.1, 9.0, variables_text, 
                    transform=ax_math.transData,
                    fontsize=11, color='#34495e',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='#ecf0f1', alpha=0.8),
                    verticalalignment='top')
        
        # Título de ecuaciones no lineales
        ax_math.text(10.5, 7.5, 'ECUACIONES NO LINEALES DEL SISTEMA:', 
                    fontsize=14, fontweight='bold', color='#e74c3c',
                    horizontalalignment='center')
        
        # Ecuación 1 - Aceleración de la base
        eq1_text = (
            "Ecuación 1 - Movimiento horizontal de la base:\n\n"
            "                F - b·ẋ + m·L·θ̇²·sin(θ) - m·g·sin(θ)·cos(θ)\n"
            "        ẍ = ────────────────────────────────────────────────\n"
            "                        M + m - m·cos²(θ)\n\n"
            "donde: Δ = M + m - m·cos²(θ)"
        )
        
        ax_math.text(8.5, 6.0, eq1_text,
                    fontsize=10, fontfamily='monospace', color='#2c3e50',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='#fff5f5', alpha=0.9),
                    verticalalignment='top')
        
        # Ecuación 2 - Aceleración angular
        eq2_text = (
            "Ecuación 2 - Movimiento rotacional del brazo:\n\n"
            "               g·sin(θ) - cos(θ)·(F - b·ẋ + m·L·θ̇²·sin(θ))\n"
            "                                  ──────────────────────────\n"
            "        θ̈ = ──────────────────────────────Δ──────────────\n"
            "                                    L\n\n"
            "Simplificando:\n"
            "                    g·sin(θ) - cos(θ)·ẍ·Δ\n"
            "            θ̈ = ─────────────────────────\n"
            "                           L"
        )
        
        ax_math.text(8.5,3.0, eq2_text,
                    fontsize=10, fontfamily='monospace', color='#2c3e50',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='#f5f5ff', alpha=0.9),
                    verticalalignment='top')
        
        # Título de función de transferencia
        ax_math.text(0.5, 3.5, 'FUNCIÓN DE TRANSFERENCIA (Sistema Linealizado):', 
                    fontsize=14, fontweight='bold', color='#27ae60',
                    horizontalalignment='center')
        
        # Función de transferencia principal
        transfer_text = (
            "Función de Transferencia Principal: G(s) = Θ(s)/F(s)\n\n"
            "                            -1\n"
            "                           ───\n"
            "                            L\n"
            "        G(s) = ─────────────────────────────────────\n"
            "                    ⎛      b           mg(M+m)⎞\n"
            "               s² · ⎜s² + ───·s  -  ─────────⎟\n"
            "                    ⎝      Δ            L·Δ  ⎠\n\n"
            "Con valores numéricos (Δ = 2.3 kg):\n\n"
            "                        -1.25\n"
            "        G(s) = ─────────────────────────────\n"
            "               s² · (s² + 0.0217·s - 12.757)\n\n"
            "Polos del sistema:\n"
            "• s₁,₂ = 0 (doble) → Integrador doble\n"
            "• s₃ ≈ -3.58 → Polo estable\n"
            "• s₄ ≈ +3.57 → Polo inestable ⚠️"
        )
        
        ax_math.text(0.5, 2.8, transfer_text,
                    fontsize=10, fontfamily='monospace', color='#2c3e50',
                    bbox=dict(boxstyle='round,pad=0.7', facecolor='#f0fff0', alpha=0.9),
                    verticalalignment='top', horizontalalignment='center')
        
        
        plt.tight_layout()
        plt.show()
    
    def run_simulation(self):
        """Ejecuta la simulación."""
        print("🎯 Simulador de Péndulo Invertido Iniciado")
        print("📋 Controles:")
        print("   ← → : Mover base hacia izquierda/derecha")
        print("   🔄 : Botón para reiniciar")
        print("   🎲 : Botón para posición aleatoria")
        print("   📚 : Botón para ver ecuaciones matemáticas")
        print("🎯 Objetivo: Mantener el brazo vertical hacia arriba")
        
        # Crear animación
        self.animation = animation.FuncAnimation(
            self.fig, self.update_animation, 
            frames=range(10000),
            interval=int(self.dt * 1000),
            blit=True, 
            repeat=True
        )
        
        plt.tight_layout()
        plt.show()

# Ejecutar el simulador
if __name__ == "__main__":
    pendulum = SquarePendulum()
    pendulum.run_simulation()