# Exoesqueleto de Asistencia a la Movilidad con Control Inteligente

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Platform](https://img.shields.io/badge/platform-ESP32-orange)
![Framework](https://img.shields.io/badge/framework-Arduino-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Firmware para un prototipo de exoesqueleto de pierna diseñado para la asistencia a la movilidad. El sistema utiliza una arquitectura de control en tiempo real para interpretar la intención de movimiento del usuario y proporcionar asistencia activa.

---

## Índice

*   [¿Qué es este proyecto?](#qué-es-este-proyecto)
*   [Características Principales](#características-principales)
*   [Arquitectura del Sistema](#arquitectura-del-sistema)
    *   [Capa de Adquisición (Sensores)](#capa-de-adquisición-sensores)
    *   [Capa de Predicción (Inteligencia)](#capa-de-predicción-inteligencia)
    *   [Capa de Actuación (Motores)](#capa-de-actuación-motores)
*   [Algoritmos Implementados](#algoritmos-implementados)
    *   [Dynamic Time Warping (DTW)](#dynamic-time-warping-dtw)
    *   [k-Nearest Neighbors (k-NN)](#k-nearest-neighbors-k-nn)
    *   [Controlador Proporcional (P)](#controlador-proporcional-p)
    *   [Otros Algoritmos](#otros-algoritmos)
*   [Hardware Requerido](#hardware-requerido)
*   [Software y Dependencias](#software-y-dependencias)
*   [Cómo Empezar](#cómo-empezar)
*   [Estructura del Código](#estructura-del-código)
*   [Líneas Futuras](#líneas-futuras)
*   [Licencia](#licencia)
*   [Autor](#autor)
*   [Agradecimientos](#agradecimientos)

---

## ¿Qué es este proyecto?

Este repositorio contiene el firmware completo para un prototipo de exoesqueleto de pierna de dos grados de libertad (cadera y rodilla). El objetivo principal del proyecto es desarrollar y validar una plataforma de hardware y software capaz de:
1.  **Detectar la intención de movimiento** del usuario a través de un análisis de la trayectoria de sus articulaciones.
2.  **Proporcionar asistencia activa y segura** mediante motores DC, ayudando al usuario a completar el movimiento deseado.
3.  **Implementar una lógica de control redundante**, utilizando múltiples sensores para confirmar la intención antes de actuar, aumentando así la seguridad.

El sistema está diseñado como una plataforma de investigación y validación tecnológica, con un enfoque en la recolección de datos y el desarrollo de algoritmos de control inteligentes.

## Características Principales

*   **Control en Tiempo Real:** Utiliza interrupciones de hardware y una estructura de bucle eficiente para una respuesta de baja latencia.
*   **Predicción de Intención Avanzada:** Implementa un clasificador basado en **Dynamic Time Warping (DTW)** y **k-Nearest Neighbors (k-NN)** para un reconocimiento de gestos robusto.
*   **Seguridad por Redundancia:** Requiere una "triple confirmación" (predicción cinemática + sensor de flexión + sensor EMG) antes de activar los motores.
*   **Asistencia Activa:** Utiliza un **controlador proporcional (P)** para guiar suavemente las articulaciones del usuario a través de trayectorias predefinidas.
*   **Modular y Extensible:** El código está organizado en módulos lógicos que facilitan su comprensión, mantenimiento y la adición de nuevas funcionalidades.
*   **Plataforma Abierta:** Desarrollado sobre un ESP32 con el framework de Arduino, lo que lo hace accesible y fácil de replicar.

## Arquitectura del Sistema

El sistema sigue una arquitectura jerárquica de tres capas, donde cada nivel tiene una responsabilidad clara.

<p align="center">
  <img src="URL_DE_TU_DIAGRAMA_DE_ARQUITECTURA" width="600" alt="Diagrama de Arquitectura"/>
</p>

### Capa de Adquisición (Sensores)
Responsable de la interacción con el hardware. Lee datos de los **encoders rotatorios** mediante interrupciones de hardware y muestrea periódicamente los sensores analógicos como el **sensor de flexión** y el **sensor EMG**.

### Capa de Predicción (Inteligencia)
El "cerebro" del sistema. Analiza los datos de los sensores para inferir la intención del usuario.
1.  **Detecta el inicio y fin** de un movimiento.
2.  **Graba la trayectoria** 2D (`cadera`, `rodilla`) del movimiento actual.
3.  **Clasifica la trayectoria** usando el algoritmo DTW/k-NN para predecir qué movimiento se está realizando.

### Capa de Actuación (Motores)
Traduce la predicción en una acción física.
1.  Verifica las señales de redundancia (Flex y EMG).
2.  Si la intención está confirmada, selecciona una secuencia de movimiento predefinida.
3.  Utiliza un **controlador de posición** para guiar los motores a través de los puntos de la trayectoria, proporcionando una asistencia suave y controlada.

## Algoritmos Implementados

### Dynamic Time Warping (DTW)
DTW es el algoritmo central para medir la similitud entre la trayectoria actual del usuario y las trayectorias expertas almacenadas. Su principal ventaja es la capacidad de comparar secuencias de diferentes longitudes y velocidades, encontrando la correspondencia óptima entre ellas. Esto lo hace ideal para analizar gestos humanos, que nunca son idénticos.

### k-Nearest Neighbors (k-NN)
Un algoritmo de clasificación intuitivo y no paramétrico. Una vez que DTW ha calculado la "distancia" a todos los movimientos conocidos, k-NN identifica los 'k' más cercanos (los más similares) y realiza una votación mayoritaria para decidir cuál es la predicción final.

### Controlador Proporcional (P)
Utilizado en la capa de actuación, es un controlador de bucle cerrado que mueve los motores a una posición objetivo. Calcula el error (`posición_objetivo - posición_actual`) y aplica una fuerza proporcional a este error, deteniéndose suavemente cuando el objetivo es alcanzado.

### Otros Algoritmos
- **Decodificación de Encoders por Tabla de Consulta:** Para una lectura ultra-rápida en las ISRs.
- **Filtro de Media Móvil:** Para suavizar las señales analógicas ruidosas (Flex, EMG).
- **Detección de Tendencia:** Para determinar si las señales Flex y EMG son crecientes, como parte de la lógica de seguridad.

## Hardware Requerido

El prototipo se ha construido con los siguientes componentes principales:
*   **Microcontrolador:** ESP32 (Modelo ESP-WROOM-32 o similar).
*   **Sensores de Posición:** 2x Encoders Rotatorios.
*   **Sensores de Intención:**
    *   1x Sensor de Flexión (Flex Sensor).
    *   1x Sensor de Electromiografía (EMG).
*   **Actuadores:** 2x Motores DC con caja reductora.
*   **Driver de Motores:** 1x Puente H L298N o similar.
*   **Feedback Visual:** 6x LEDs (Rojo, Amarillo, Verde para cada articulación).
*   **Alimentación:** Batería LiPo y reguladores de voltaje adecuados.
*   **Estructura Mecánica:** Impresa en 3D (PLA/PETG) o construida en aluminio.

## Software y Dependencias

El firmware está desarrollado para el **framework de Arduino**.
*   **Entorno de Desarrollo:** [Arduino IDE](https://www.arduino.cc/en/software) o [PlatformIO](https://platformio.org/) para VS Code.
*   **Configuración de la Placa:** Se debe configurar el entorno para la placa "ESP32 Dev Module".
*   **Librerías:** El código no requiere librerías externas, ya que utiliza funciones nativas del core de Arduino para ESP32 (`math.h`, `digitalWrite`, `analogRead`, etc.).

## Cómo Empezar

1.  **Clonar el Repositorio:**
    ```bash
    git clone https://github.com/TU_USUARIO/TU_REPOSITORIO.git
    ```
2.  **Configurar el Hardware:** Ensambla los componentes electrónicos siguiendo las definiciones de pines (`#define`) en la cabecera del fichero `.ino`.
3.  **Abrir en el IDE:** Abre el fichero `.ino` en tu Arduino IDE o PlatformIO.
4.  **Seleccionar la Placa y el Puerto:** Asegúrate de que "ESP32 Dev Module" y el puerto COM correcto estén seleccionados.
5.  **Ajustar Parámetros:** Antes de compilar, revisa la sección de `PARÁMETROS DE AJUSTE` y ajusta los umbrales `FLEX_ACTIVATION_THRESHOLD` y `EMG_ACTIVATION_THRESHOLD` según las lecturas de tus sensores.
6.  **Compilar y Subir:** Sube el código a tu ESP32.
7.  **Monitorizar:** Abre el Monitor Serie a **115200 baudios** para ver los datos de depuración y el estado del sistema en tiempo real.

## Estructura del Código

El código está contenido en un único fichero `.ino` y está organizado en módulos lógicos mediante comentarios:
*   **`MÓDULO 1: ALGORITMOS DE PREDICCIÓN`**: Contiene las funciones `dtw_distance` y `knn_classify`.
*   **`MÓDULO 2: SENSORES AUXILIARES`**: Lógica para procesar los sensores Flex y EMG.
*   **`MÓDULO 3: CONTROL DE MOTORES`**: Funciones para la asistencia activa, incluyendo `move_to_target` y `movementAction`.
*   **`MÓDULO 4: ISRs Y HARDWARE`**: Rutinas de interrupción y funciones de bajo nivel.
*   **`SETUP` y `LOOP`**: Funciones principales de Arduino que orquestan todo el sistema.

## Líneas Futuras

Este proyecto sienta las bases para futuras mejoras:
*   **Implementación de un Controlador PID Completo:** Mejorar el controlador P actual a un PID para una asistencia más suave y precisa.
*   **Calibración Automática:** Desarrollar una rutina de calibración al inicio para ajustar automáticamente los umbrales de los sensores.
*   **Grabación de Nuevos Gestos:** Implementar un modo "grabación" que permita al usuario enseñar nuevos movimientos al sistema.
*   **Conectividad Inalámbrica:** Usar el WiFi o Bluetooth del ESP32 para enviar datos de telemetría a una aplicación móvil o un dashboard web.

## Licencia

Este proyecto se distribuye bajo la **Licencia MIT**. Consulta el fichero `LICENSE` para más detalles.

## Autor

*   **Jose Morcillo Artigas** - [GitHub](https://github.com/pepmorart) | [Correo](mailto:pep.m.artigas@gmail.com)

## Agradecimientos

*   Agradecimientos a [Nombre del Tutor/a] por su guía y apoyo durante el desarrollo de este proyecto.
*   Inspirado por la comunidad de robótica de código abierto.
