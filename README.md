# 🚗 ESP32 Logger v3.4 - Sistema de Telemetría Vehicular

## 📋 Descripción General

El **ESP32 Logger v3.4** es un sistema de telemetría vehicular avanzado que combina GPS, OBD-II/CAN, almacenamiento SD y transmisión WiFi para monitorear y registrar datos del vehículo en tiempo real. El sistema implementa una máquina de estados inteligente que optimiza la recolección y transmisión de datos según la ubicación del vehículo.

## ✨ Características Principales

- **GPS de Alta Precisión**: Monitoreo de ubicación, velocidad y altitud
- **Comunicación OBD-II/CAN**: Lectura de parámetros del motor en tiempo real
- **Almacenamiento Local**: Tarjeta SD para respaldo de datos
- **Transmisión WiFi**: Envío automático de datos al servidor
- **Geocerca Inteligente**: Detección automática de base/garaje
- **Máquina de Estados**: Gestión inteligente de recolección y transmisión
- **Modo Simulación**: Pruebas sin vehículo físico
- **Manejo de Errores**: Recuperación automática de fallos

## 🏗️ Arquitectura del Sistema

### Componentes Hardware
- **ESP32**: Microcontrolador principal
- **GPS NEO-6M**: Receptor GPS con antena activa
- **MCP2515**: Controlador CAN para comunicación OBD-II
- **Tarjeta SD**: Almacenamiento de datos local
- **Módulo WiFi**: Conectividad de red integrada

### Pines de Conexión
```cpp
#define SD_CS       33    // Chip Select para SD
#define CAN_CS      14    // Chip Select para CAN
#define CAN_INT     27    // Interrupción CAN
// GPS: Serial1 (pines 16, 17)
```

## 🔧 Configuración del Sistema

### Modo de Simulación
```cpp
#define USE_SIMULATION_MODE true
#define SIMULATE_OBD_FAILURE true  // Simula fallos OBD
```

**Opciones de Simulación:**
- `SIMULATE_OBD_FAILURE = true`: Todos los PIDs OBD retornan `null`
- `SIMULATE_OBD_FAILURE = false`: Datos OBD simulados con valores aleatorios

### Configuración WiFi
```cpp
const char* ssid     = "Base";
const char* password = "042116361";
const char* serverUrl = "http://localhost:port/api/v1/telemetria_axiscan";
```

### Configuración de Base (Geocerca)
```cpp
const double BASE_LAT = -2.09407;      // Latitud de la base
const double BASE_LON = -79.92911;     // Longitud de la base
const double BASE_RADIUS_M = 60.0;     // Radio de la base en metros
```

### Parámetros de Funcionamiento
```cpp
const double MIN_MOVE_DISTANCE_M = 20.0;           // Distancia mínima para nueva lectura
const long FLUSH_RETRY_INTERVAL_MS = 30000;       // Intervalo de reintento (30s)
const unsigned long OBD_PID_TIMEOUT_MS = 75;      // Timeout por PID OBD
const int BATCH_SIZE = 10;                        // Tamaño del lote en RAM
```

## 🎯 Máquina de Estados

### Estados del Sistema

#### 1. **STATE_COLLECTING** - Recolección de Datos
- **Actividad**: Monitorea GPS y recolecta datos OBD
- **Condición de Salida**: Vehículo entra en la base
- **Acción**: Inicia proceso de descarga de datos

#### 2. **STATE_FLUSHING_DATA** - Descarga de Datos
- **Actividad**: Envía datos pendientes al servidor
- **Condición de Salida**: Todos los datos enviados exitosamente
- **Reintentos**: Cada 30 segundos si falla la transmisión

#### 3. **STATE_IN_BASE_IDLE** - En Base (Inactivo)
- **Actividad**: Espera en la base sin datos pendientes
- **Condición de Salida**: Vehículo sale de la base
- **Acción**: Reinicia recolección de datos

### Transiciones de Estado
```
RECOLECTANDO → [Vehículo entra en base] → DESCARGANDO DATOS
     ↑                                           ↓
     ← [Vehículo sale de base] ← EN BASE (IDLE)
```

## 📊 Estructura de Datos

### Formato JSON de Telemetría
```json
{
  "gps": {
    "fix": true,
    "lat": -2.09407,
    "lng": -79.92911,
    "alt": 25.5,
    "speed_kmh": 45.2,
    "satellites": 8,
    "timestamp": "2024-01-15T14:30:25Z"
  },
  "obd": {
    "rpm": 2500,
    "speed": 65,
    "coolant": 90,
    "tps": 45,
    "fuel_level": 75,
    "voltage": 13.8
  },
  "ts": 1705327825000
}
```

### PIDs OBD-II Soportados
| PID | Descripción | Unidad | Fórmula de Conversión |
|-----|-------------|---------|----------------------|
| 0x0C | RPM del Motor | RPM | `(byte3 * 256 + byte4) / 4.0` |
| 0x0D | Velocidad del Vehículo | km/h | `byte3` |
| 0x05 | Temperatura del Refrigerante | °C | `byte3 - 40` |
| 0x11 | Posición del Acelerador | % | `(byte3 * 100.0) / 255.0` |
| 0x2F | Nivel de Combustible | % | `(byte3 * 100.0) / 255.0` |
| 0x42 | Voltaje del Sistema | V | `(byte3 * 256 + byte4) / 1000.0` |

## 🚀 Funcionamiento del Sistema

### Ciclo de Trabajo Principal
1. **Lectura GPS**: Cada 5 segundos
2. **Detección de Movimiento**: Compara con última posición registrada
3. **Recolección OBD**: Solicita y lee PIDs del motor
4. **Almacenamiento**: Guarda en SD y RAM
5. **Transmisión**: Envía lotes cuando está en la base

### Lógica de Geocerca
- **Dentro de la Base**: Inicia descarga de datos pendientes
- **Fuera de la Base**: Recolecta datos cada 20 metros de movimiento
- **Transición**: Cambio automático de estado según ubicación

### Gestión de Datos
- **Almacenamiento en Lotes**: Acumula hasta 10 lecturas en RAM
- **Respaldo en SD**: Guarda cada lectura en archivo JSONL
- **Transmisión Inteligente**: Envía datos cuando hay conectividad WiFi

## 🔌 Instalación y Configuración

### Requisitos de Hardware
- ESP32 DevKit o similar
- Módulo GPS NEO-6M
- MCP2515 + Transceptor CAN
- Tarjeta SD (formato FAT32)
- Antena WiFi (integrada en ESP32)

### Conexiones Físicas
```
ESP32 Pin 33 → SD Card CS
ESP32 Pin 14 → MCP2515 CS  
ESP32 Pin 27 → MCP2515 INT
ESP32 Pin 16 → GPS TX
ESP32 Pin 17 → GPS RX
```

### Configuración Inicial
1. **Cargar Firmware**: Subir código al ESP32
2. **Configurar WiFi**: Modificar credenciales en el código
3. **Configurar Base**: Ajustar coordenadas y radio de la geocerca
4. **Probar Simulación**: Verificar funcionamiento en modo simulación
5. **Desplegar**: Cambiar a modo producción

## 📱 Monitoreo y Debugging

### Salida Serial
```
=== Logger v3.4 - Compilación Corregida ===
!!! MODO SIMULACIÓN ACTIVADO !!!
--- Simulación de FALLO OBD activada (PIDs serán 'null') ---
✅ SD OK
✅ CAN OK (MCP2515 16MHz @500kbps)
Listo. Estado inicial: RECOLECTANDO
```

### Indicadores de Estado
- 🚗 **Movimiento detectado**: Vehículo se mueve más de 20m
- 📍 **DENTRO de la base**: Inicia descarga de datos
- ✅ **Datos enviados**: Transmisión exitosa
- ⚠️ **Fallo en envío**: Reintento programado
- 🚀 **FUERA de la base**: Reanuda recolección

## 🛠️ Mantenimiento y Troubleshooting

### Problemas Comunes

#### Error de Inicialización SD
- Verificar formato FAT32 de la tarjeta
- Comprobar conexión del pin CS
- Revisar alimentación de 3.3V

#### Fallos de Comunicación CAN
- Verificar terminación de 120Ω en el bus CAN
- Comprobar velocidad de 500kbps
- Revisar conexiones del transceptor

#### Pérdida de Conectividad WiFi
- Verificar credenciales de red
- Comprobar señal WiFi en la ubicación
- Revisar configuración del servidor

### Logs y Debugging
- **Serial Monitor**: 115200 baudios
- **Archivo SD**: `/telemetry.jsonl`
- **Estado del Sistema**: Monitoreo en tiempo real

## 📈 Optimizaciones y Mejoras Futuras

### Optimizaciones Sugeridas
- **Compresión de Datos**: Reducir tamaño de transmisión
- **Caché Inteligente**: Priorizar datos críticos
- **Reconexión WiFi**: Mejorar estabilidad de conexión
- **Validación de Datos**: Filtrar lecturas erróneas

### Funcionalidades Adicionales
- **Dashboard Web**: Interfaz de monitoreo local
- **Alertas**: Notificaciones de eventos críticos
- **Múltiples Vehículos**: Soporte para flota
- **Análisis Avanzado**: Machine Learning para predicciones

## 📄 Licencia y Créditos

**Desarrollado por**: Equipo de Desarrollo AxisCAN  
**Versión**: 3.4  
**Fecha**: 2024  
**Plataforma**: ESP32 + Arduino Framework  

---

## 🆘 Soporte Técnico

Para soporte técnico o reportar problemas:
- **Email**: soporte@axiscan.com
- **Documentación**: [docs.axiscan.com](https://docs.axiscan.com)
- **GitHub**: [github.com/axiscan/firmware](https://github.com/axiscan/firmware)

---

*"Monitoreo inteligente para vehículos del futuro"* 🚀
# Firmware-AxisCAN
