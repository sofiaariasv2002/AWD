# Análisis cinemático: Tabla de Denavit-Hartenberg

En la siguiente sección se realizará el análisis de la pierna izquierda del robot AWD bípode.

## Pasos:

1. Obtuvimos de RViz los ejes de nuestro robot para visualizar cómo están orientados.

   ![alt text](imagenes/ReferenciaRviz.png)

   **Foto 1. Ejes de nuestro robot bípode**

   En Gazebo el eje rojo es X, el eje verde es Y y el eje azul es Z.

2. Basado en el URDF, pudimos obtener que la cadena cinemática del robot:

| Sistema (Si)    | Eje Z (rotación) | Eje X  | Descripción del movimiento            |
| --------------- | ---------------- | ------ | ------------------------------------- |
| S0 (Pelvis)     | Z₀ (↓)           | X₀ (→) | Base fija del torso                   |
| S1 (hip\_yaw)   | Z₁ (↓)           | X₁ (→) | Rotación en eje vertical (θ1)         |
| S2 (hip\_roll)  | Z₂ (→)           | X₂ (↓) | Rotación lateral de la cadera (θ2)    |
| S3 (hip\_pitch) | Z₃ (←)           | X₃ (→) | Flexión/extensión hacia adelante (θ3) |
| S4 (knee)       | Z₄ (←)           | X₄ (↓) | Flexión de la rodilla (θ4)            |
| S5 (ankle)      | -                | -      | Punto final de la pierna              |

3. Después de analizar los marcos, obtuvimos la siguiente tabla de Denavit-Hartenberg:

   ![alt text](imagenes/DH-pato.png)

   El procedimiento que se realizó para llegar a esto estuvo basado en el URDF:

   Para determinar las Z, se colocaron en la dirección de rotación de cada articulación.
   Con los parámetros a y d se tomaron los valores de `origin` del URDF según el eje que correspondiera.

### Detalles por articulación:

* **left\_hip\_yaw**: Z1 apunta hacia abajo. d es la distancia vertical desde la pelvis hasta la articulación. a es el desplazamiento lateral en Y. Como el eje Z1 es perpendicular al Z0 de la pelvis, el ángulo twist α es -90° (o -π/2 rad). La variable θ1 representa el ángulo de rotación.

* **left\_hip\_roll**: Z2 sigue la dirección del movimiento de "roll". d = 0.076 m es la distancia en Z desde la articulación anterior. El parámetro a es pequeño. α = +90° porque Z2 es perpendicular a Z1. θ2 se ajusta con -π/2 para alinear los marcos.

* **left\_hip\_pitch**: Esta articulación gira en "pitch". d es casi cero. a es la distancia hacia adelante desde la articulación de roll. α = -90° porque Z3 es perpendicular a Z2. θ3 controla el movimiento de la cadera.

* **left\_knee**: Z4 es paralelo a Z2, por lo tanto α = 0°. d es la distancia vertical a la rodilla. a = 0.079306 m. θ4 controla la flexión de la rodilla.

* **left\_ankle**: Z5 apunta opuesto a Z1. d = -0.13 m, distancia desde la rodilla al tobillo. a = 0.0152 m. α = 0°. θ5 controla la inclinación del pie.

# Visualización de la posición del efector final en RViz

Se calculó la posición de la pelvis respecto al pie izquierdo usando la cadena DH.

Los parámetros fueron extraídos del URDF en orden invertido, de `left_ankle` a `pelvis`. Se utilizó un nodo en ROS 2 que escucha `/joint_state`, recorre la tabla DH y multiplica las matrices homogéneas:

$$
T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Se publica un `Marker` en el frame `pelvis` con la posición final.

![alt text](imagenes/DH-pelvis.png)

Valor obtenido:

```
x=0.165, y=-0.122, z=0.207
```

Se verificó usando:

```
ros2 run tf2_ros tf2_echo pelvis left_foot_link
```

Transformación inversa:

$T_{\text{foot} \to \text{pelvis}} = T^{-1}_{\text{pelvis} \to \text{foot}}$

Múltiples transformaciones:

$T_{\text{foot} \to \text{pelvis}} = T_1 \cdot T_2 \cdot T_3 \cdot T_4 \cdot T_5$

# Simulación en Gazebo con controladores

Se utilizó `gazebo.launch.py` para lanzar Gazebo. Se agregó el plugin `gazebo_ros2_control` y el `controller_manager`. Se definieron `transmissions` solo para la parte inferior del robot. Se utilizaron `SimpleTransmission` con control por posición.

Ejemplo:

```xml
<transmission name="left_hip_yaw_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_yaw">
    <hardwareInterface>position</hardwareInterface>
  </joint>
  <actuator name="left_hip_yaw_motor">
    <hardwareInterface>position</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

Definición del archivo `controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_hip_yaw
      - left_hip_roll
      - left_hip_pitch
      - left_knee
      - left_ankle
      - right_hip_yaw
      - right_hip_roll
      - right_hip_pitch
      - right_knee
      - right_ankle
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

# Generación heurística de una sentadilla

La sentadilla se generó por prueba y error usando `joint_state_publisher_gui`. Se invirtieron los ejes de la pierna derecha por consistencia. Se creó un `main.launch.py` para evitar duplicación de nodos.

Se enviaron comandos con:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{...}"
sleep 6
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{...}"
sleep 6
```

El robot se spawnea tirado, por lo que no hay efecto de gravedad al bajar. Se validó visualmente en RViz y Gazebo.

[Haz clic para ver el video](imagenes/sentadilla.mp4)

**El pato es muy chiquito como para poder incrementar el movimiento.**
