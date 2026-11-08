# Guante de tele-operación · Práctica 3 PDS (Filtro de Kalman, UMNG)

Firmware ESP-IDF v6.1 del **guante** (Micro 1 de la guía): ESP32 + MPU6050 en la mano
del operador. Hace parte de un sistema de tres partes:

| Parte | Carpeta |
|---|---|
| Guante (este proyecto) | `~/laboratorio_final_guante` |
| Carro minisumo (MPU6050, encoders, PID) | `~/laboratorio_final_senales` |
| PC: ROS 2 Lyrical (EKF/UKF, tele-operación, app web :8090) | `~/ros2_ws/src/minisumo_teleop` |

## Qué hace el guante
- Lee el MPU6050 a 100 Hz (driver en `components/MPU6050`, copia del carro), calcula la orientación con
  un filtro de Mahony (giroscopio + acelerómetro; sesgo del giroscopio calibrado 1 s al arrancar) y
  aplica pasa-bajos EMA al giroscopio y a la aceleración lineal. Roll/pitch/yaw son
  **relativos a la postura neutra** (`g_zero_req`).
- **ESP-NOW → carro** (50 Hz, broadcast, `espnow_tx_task`): paquete `glove_packet_t`
  (`components/EspNowLink/include/espnow_link.h`) con flags armado/fusión, rpy, giro y
  aceleración. El carro convierte pitch → v y roll → ω (estrategia 2) y solo acepta
  paquetes con su mismo `CONFIG_ESPNOW_GROUP_ID`.
- **micro-ROS → PC** (WiFi, agente en Docker en el PC, UDP 8888): publica
  `glove/imu/raw`, `glove/imu/filtered`, `glove/rpy` (best effort, 50 Hz) y `glove/armed`;
  se suscribe a `glove/zero` (Empty) y `glove/arm` (Bool).
- **Botón BOOT (GPIO0)**: corto = armar/desarmar; 1 s = postura neutra. **LED GPIO2** = armado.
  Arranca siempre desarmado; no se puede armar sin fusión válida.
- El BNO055 original se quemó (2026-10-04) y se cambió por un MPU6050; el código del BNO055 está en el
  historial de esta conversación, no en el proyecto.

## Reglas al modificar
- `components/EspNowLink` está **copiado idéntico** en el carro
  (`~/laboratorio_final_senales/components/EspNowLink`). Si cambias el paquete, cambia
  ambas copias igual y sube `GLOVE_PKT_VERSION`; luego compila y flashea los dos ESP32.
- Los nombres de tópicos los usan los nodos del PC (`minisumo_teleop`): no los cambies sin
  actualizar también `hand_estimator.py`, `glove_teleop.py`, `dashboard.py` y `fake_hardware.py`.
- ESP-NOW necesita que guante y carro estén en el **mismo canal WiFi** (mismo router, o
  ambos sin router → canal `CONFIG_ESPNOW_FALLBACK_CHANNEL`).
- IDF compila con `-Werror`: las llamadas rcl/rclc deben consumir su retorno
  (`RCRETCHECK` / `RCSOFTCHECK`).
- Comentarios y textos del log en español, sin tildes en el código C (estilo actual).

## Compilar
```bash
source ~/.espressif/tools/activate_idf_v6.1.sh
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```
Si micro-ROS se recompila desde cero y falla con `ld: no se reconoce el modo de emulación:
elf_x86_64`, antes de `idf.py build`:
```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -vE '/[^/]*-elf/[^/]*-elf/bin$' | paste -sd:)
```

## Probar sin hardware
El PC tiene un simulador que imita los tópicos del guante y del carro (incluido el enlace
ESP-NOW): `ros2 launch minisumo_teleop teleop.launch.py sim:=true`. Úsalo con un
`ROS_DOMAIN_ID` distinto si el carro real está encendido (publica `/cmd_vel`).

## Historia
Este proyecto nació dentro de `~/laboratorio_final_senales/guante` y se movió a su propia
carpeta el 2026-10-03. La conversación completa en la que se diseñó (BNO055 → guante,
MPU6050 → carro, nodos EKF/UKF, app web, ESP-NOW) se puede retomar con `/resume`.
