# Control limo whith joystc

## Before install these pkgs

```bash
sudo apt update
sudo apt install ros-humble-joy \
                 ros-humble-joy-linux \
                 ros-humble-teleop-twist-joy
```


## 1) Conect your control joystick

```bash
ros2 run joy joy_node
```

### Verify if the joysticks is conecteded

```bash
ros2 run joy joy_enumerate_devices
```
### See your control inputs

```bash
ros2 topic echo /joy
```

## 2) joystick mapping

### 2.1) joystick mapping file .yaml 

```yaml
/**:
  ros__parameters:
    require_enable_button: true
    enable_button: 5 # Altere para o botão que você quer que seja o "Deadman Switch"
    enable_turbo_button: 7 # Altere para o botão do modo "Turbo"
    
    # Configurações de Eixos e Escalas
    axis_linear:
      x: 1 # Eixo para frente/trás (geralmente o Left Stick Y - índice 1 do Xbox)
      y: -1 # Desabilita (se não for holonômico)
    scale_linear:
      x: 0.5 # Velocidade linear máxima normal
    scale_linear_turbo:
      x: 1.0 # Velocidade linear máxima turbo
      
    axis_angular:
      yaw: 0 # Eixo para girar (geralmente o Left Stick X - índice 0 do Xbox)
    scale_angular:
      yaw: 0.5 # Velocidade angular máxima normal
    scale_angular_turbo:
      yaw: 1.0 # Velocidade angular máxima turbo
```
#### Comand bash 

```bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file <Directore of file .yaml>
```

### 2.2) With the prompt

```bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=2 \
  -p enable_button:=5 \
  -p scale_linear.x:=0.8
```


# Start Limo Robot

```bash
ros2 launch limo_base limo_base.lauch.py
```