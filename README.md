# Renatico

Proyecto para controlar un DARwIn-OP en Webots usando modelos de inteligencia artificial con el fin de detectar el riesgo (incendios forestales).

## Qué hace el proyecto

El mundo [`webots/worlds/forest_fire.wbt`](webots/worlds/forest_fire.wbt) simula un escenario de incendio forestal con un DARwIn-OP que:

1. **Camina** con un port a Python puro de la marcha ROBOTIS original ([`walking.py`](webots/controllers/darwin_patrol/walking.py)): cada paso resuelve la cinemática inversa de las piernas a partir de amplitudes de avance/giro y mueve los 20 joints vía `Motor.setPosition()`.
2. **Patrulla** de forma autónoma ([`darwin_patrol.py`](webots/controllers/darwin_patrol/darwin_patrol.py)): ciclo de andar 25 s y girar 8 s, partiendo desde la carretera.
3. **Detecta incendios**: cada fuego del mundo es un `Robot` con un `Emitter` en el canal 1 y alcance limitado (8 m). El robot recibe la señal con su `Receiver`; si está lo bastante cerca, **habla** por el `Speaker`, enciende los ojos en rojo e imprime `[alarm]`, re-alarmando cada 8 s mientras siga cerca.
4. **Telemetría opcional**: si el DARwIn-OP se marca con `supervisor TRUE`, el controlador imprime su posición cada 5 s.

## Estructura

- `webots/controllers/darwin_patrol/` — cerebro del robot: controlador de patrulla (`darwin_patrol.py`) y motor de marcha (`walking.py`).
- `webots/controllers/fire_beacon/` — controlador de los fuegos (envían la señal de radio).
- `webots/worlds/forest_fire.wbt` — mundo principal (carretera, bosque, 4 fuegos, "Starting point").

## Cómo correrlo

1. Requisito: [Webots](https://cyberbotics.com/) R2025a.
2. Abre `webots/worlds/forest_fire.wbt` en Webots y pulsa Play (icono ▶).

El robot arranca en la carretera, patrulla hacia el claro y al acercarse a un fuego dispara la alarma. Para ver la posición en consola puedes activar `supervisor TRUE` en el nodo `Darwin-op` del mundo.