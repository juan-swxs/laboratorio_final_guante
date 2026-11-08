# Guante de tele-operación (ESP32 + MPU6050) · Práctica 3

Firmware del **guante** (Micro 1). El carro está en `~/laboratorio_final_senales`.
El software del PC (ROS 2): `~/ros2_ws/src/minisumo_teleop`.

```
laboratorio_final_guante/
├── main/
│   ├── main.c               lectura del MPU6050 + filtro de Mahony, ESP-NOW al carro, micro-ROS al PC,
│   │                        botón de armado y LED
├── components/
│   └── EspNowLink/          paquete y enlace ESP-NOW (copia idéntica en el carro)
├── managed_components/      micro-ROS (lo descarga ESP-IDF)
└── sdkconfig.defaults       WiFi y agente micro-ROS
```

## Compilar y flashear
```bash
source ~/.espressif/tools/activate_idf_v6.1.sh
cd ~/laboratorio_final_guante
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```
Si alguna vez hay que recompilar micro-ROS desde cero (por ejemplo tras borrar
`managed_components/`) y falla con `ld: no se reconoce el modo de emulación: elf_x86_64`,
compila con este PATH (quita los `ld` internos de los toolchains del ESP32):
```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -vE '/[^/]*-elf/[^/]*-elf/bin$' | paste -sd:)
```

## Uso
- **Botón BOOT** (GPIO0): pulsación corta = armar / desarmar el mando directo;
  mantener 1 s = fijar la postura neutra. **LED** (GPIO2) encendido = armado.
- **ESP-NOW → carro** (50 Hz): inclinación relativa, giro y aceleración de la mano.
- **micro-ROS → PC**: `glove/imu/raw`, `glove/imu/filtered`, `glove/rpy`, `glove/armed`;
  recibe `glove/zero` y `glove/arm`.
- MPU6050: VCC→3V3, GND, SDA GPIO21, SCL GPIO22, dirección 0x68/0x69 (se detecta sola). La orientación se calcula en el ESP32 con un filtro de Mahony (giroscopio + acelerómetro); al arrancar hay que dejar el guante quieto ~1 s para calibrar el giroscopio. Sin magnetómetro el yaw es relativo y deriva (roll y pitch, que mueven el carro, no derivan).
