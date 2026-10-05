# Guía rápida: comandos listos

Todo se ejecuta **dentro del contenedor**, desde esta carpeta:

```bash
cd ~/workspaces/dataset
```

---

## 0. Una sola vez (o después de recrear el contenedor): Vulkan

Si al colectar ves `Timed out waiting for binary to load` y el run sale vacío
(0 frames, sin imágenes), es que Unreal no encuentra el driver Vulkan de NVIDIA
(`VK_ERROR_INCOMPATIBLE_DRIVER`). Se arregla creando el ICD:

```bash
mkdir -p ~/.local/share/vulkan/icd.d
cat > ~/.local/share/vulkan/icd.d/nvidia_icd.json <<'EOF'
{
    "file_format_version" : "1.0.1",
    "ICD": { "library_path": "libGLX_nvidia.so.0", "api_version" : "1.4.312" }
}
EOF
```

---

## 1. Crear la ruta haciendo clic en el mapa

```bash
./create_route.sh mi_ruta                 # profundidad -1.5 m, 1 m/s
Z=-3 SPEED=0.5 ./create_route.sh mi_ruta  # otra profundidad / velocidad
```

Se abre el mapa (`map/mapa.png`) y colocas los waypoints con el ratón:

| Acción | Cómo |
|---|---|
| Añadir waypoint | **clic izquierdo** |
| Deshacer el último | **clic derecho** o tecla **`z`** |
| Guardar y salir | tecla **Enter** |
| Hacer zoom / moverse | lupa / cruz de la barra de herramientas (mientras están activas, los clics **no** añaden puntos: desactívalas para seguir colocando) |
| Volver a la vista completa | botón de la casa |
| Salir sin guardar | cerrar la ventana |

- El orden de los clics es el orden en que se recorren los puntos (se numeran en amarillo).
- Abajo se ven las **coordenadas del mundo** (x, y) bajo el ratón.
- La orientación (yaw) de cada punto se calcula sola: mira hacia el siguiente.
- Genera `waypoints/mi_ruta.csv` y `waypoints/mi_ruta.yaml`.

**Editar una ruta existente:** si `waypoints/mi_ruta.csv` ya existe,
`./create_route.sh mi_ruta` la abre con sus puntos cargados; puedes deshacer
(`z`) los últimos y añadir otros. Se guarda una copia de la versión anterior en
`waypoints/mi_ruta.csv.bak`. Para empezar de cero usa otro nombre.

## 2. Ver la ruta sobre el mapa (sin simulador)

```bash
python3 tools/plot_route_on_map.py --waypoints-csv waypoints/mi_ruta.csv --out results/mi_ruta.png
```

Esto **solo dibuja** una imagen; no colecta nada.

## 3. Probar el movimiento viendo el simulador (no graba)

```bash
./test_route.sh waypoints/mi_ruta.yaml --show-view
```

La posición real queda en `results/dryrun_ground_truth.csv`.

## 4. Colecta real (RGB, segmentación, sonar, IMU, DVL, ground truth...)

```bash
./start_collection.sh waypoints/mi_ruta.yaml --show-view --run-name prueba1
```

Sin `--show-view` corre sin ventana (más rápido). Al terminar, el script:

1. valida el dataset (Hz, huecos, NaN), y
2. guarda `results/prueba1_comparison.png` (ruta deseada vs real) e imprime la
   distancia mínima a cada waypoint.

## 5. Revisar lo grabado

```bash
R=dataset/colecta_simple/runs/prueba1
ls $R/segmentation | wc -l      # imágenes segmentadas
ls $R/rgb | wc -l               # imágenes RGB
python3 tools/validate_dataset.py $R
xdg-open $R/segmentation/$(ls $R/segmentation | head -1)   # ver una imagen
```

Estructura del run: `ground_truth/ imu/ dvl/ depth/ magnetometer/ rgb/ segmentation/ sonar/` + metadatos.

---

## Punto inicial (spawn): automático

**No hace falta configurar nada.** `start_collection.sh` y `test_route.sh`
generan solos `config/generated/simulator_<ruta>.yaml`, una copia de
`config/simulator_rov.yaml` con:

- `location` = x, y, z del **waypoint 1** del CSV
- `rotation` = yaw_deg del waypoint 1 (ya mirando hacia el primer tramo)

Así el ROV aparece sobre la ruta y avanza de frente. Si antes aparecía lejos,
tenía que ir de espaldas hasta el punto 1 y luego volver, y parecía que
"retrocedía" o que los puntos estaban desordenados.

Para usar el spawn fijo de `config/simulator_rov.yaml` en su lugar:

```bash
SPAWN_AT_START=0 ./start_collection.sh waypoints/mi_ruta.yaml
```

## Llegada real a cada punto (closed-loop)

Las rutas creadas con `create_route.sh` (y `mi_ruta.yaml`) traen
`closed_loop_waypoints: true`: el ROV **espera a llegar de verdad** (a menos de
`arrival_tolerance_m`) a cada punto antes de ir al siguiente. Sin eso, el plan
avanza por tiempo y, como el ROV real va a ~0.6 m/s aunque pidas 1 m/s, se
salta puntos.

Si a un punto no llega, avanza igual tras un timeout (3x la duración nominal
del tramo) para no quedarse atascado.

---

## Opciones útiles

| Qué | Cómo |
|---|---|
| Ver el simulador | añadir `--show-view` |
| Nombre del run | `--run-name prueba1` |
| Límite de tiempo (seguridad) | `MAX_DURATION=900 ./start_collection.sh ...` (por defecto 600 s simulados; la ruta termina sola antes) |
| Parar al llegar a un punto | `--stop-at-waypoint 4` |
| Tolerancia de llegada | `--waypoint-tolerance 2.0` (m) |
| Usar spawn fijo | `SPAWN_AT_START=0` |
| Grabar menos sensores | quitar writers de `config/dataset.yaml` (`rgb`, `sonar_*`...) |
| Otra config | `SIM_CONFIG=... DATASET_CONFIG=... ./start_collection.sh ...` |
| Todas las opciones | `python3 tools/collect_dataset.py --help` |

## Formato del CSV de waypoints

```csv
waypoint_id,x,y,z,yaw_deg,speed_mps,hold_time_s
1,-71.8909,-247.0227,-1.5,-16.11,1.0,0.0
2,-48.5727,-253.7591,-1.5,-5.19,1.0,2.0
```

| Columna | Unidad | Significado |
|---|---|---|
| `x`, `y` | m | posición absoluta en el mundo (mismo marco que `map/mapa.yaml`) |
| `z` | m | profundidad; negativo = bajo el agua |
| `yaw_deg` | grados | orientación en ese punto (-180..180) |
| `speed_mps` | m/s | velocidad del tramo que termina en este punto |
| `hold_time_s` | s | pausa al llegar |

El mapa cubre x ∈ [-140.5, 258.5], y ∈ [-444.5, -45.5]. Para convertir:

```bash
python3 tools/map_utils.py --map-yaml map/mapa.yaml --world 59 -245      # mundo -> píxel
python3 tools/map_utils.py --map-yaml map/mapa.yaml --pixel 1995 1995    # píxel -> mundo
```

## Problemas comunes

| Síntoma | Causa / solución |
|---|---|
| `Timed out waiting for binary to load`, run vacío | Falta el ICD de Vulkan → paso 0 |
| El ROV va de espaldas / "retrocede" | Spawn lejos del punto 1 → no usar `SPAWN_AT_START=0` |
| Se salta waypoints | Usar `closed_loop_waypoints: true` en el YAML de la ruta |
| La colecta se corta a mitad de ruta | Subir `MAX_DURATION` |
| El simulador tarda en arrancar | Normal: ~40 s cargando el mundo antes de moverse |
