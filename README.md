# Instrucciones de ejecución PSP-UAV

Benjamin Camus Valdés

## Compilación
```bash
make
```
Genera el ejecutable `./PSP-UAV`.

`make clean` elimina binarios previos.

## Ejecución
```bash
./PSP-UAV <ruta_instancia> <n_drones> <K_iter> <T_ticks> [--export]
```
- `<ruta_instancia>`: archivo `.txt` con la grilla (ej: `instancias/PSP-UAV_01_a.txt`).
- `<n_drones>`: cantidad máxima de drones disponibles.
- `<K_iter>`: iteraciones de Simulated Annealing.
- `<T_ticks>`: duración total (número de ticks a simular).
- `--export` (opcional): guarda CSV en `exported_data/` con rutas, bases, obstáculos y urgencias.

### Ejemplo
```bash
./PSP-UAV instancias/PSP-UAV_01_a.txt 3 3000 50 --export
```


## Salida
En consola se imprime la urgencia acumulada, la ventana T usada, la cantidad de drones, el tiempo de ejecución, las colisiones detectadas y la ruta completa de cada dron (indicando su base de origen). Con `--export` se generan los CSV mencionados en la carpeta `exported_data/`.


Se puede visualizar el resultado con el código de `visualization.ipynb`

## Automatización de pruebas
Se agregó `scripts/run_experiments.py` para crear runs por lotes y guardar resultados en `experiments/`. La configuración es en `scripts/experiments_config.json`

1. Ajustes del JSON:
   - En `defaults` se puede definir `instances`, `iterations`, `iterations_by_instance`, `drones`, `ticks`, `repeats`, `export_data`, `include_time_breakdown`, `export_csv`.
   - En `experiments` se agregan objetos con `name` y overrides de esos campos. 
2. Ejecutar:
   ```bash
   python3 scripts/run_experiments.py          # usa scripts/experiments_config.json
   # o con otro archivo:
   python3 scripts/run_experiments.py --config path/a/mi_config.json
   ```
   Si el binario `PSP-UAV` no existe, el script corre `make` automáticamente.
3. Por cada experimento se generan `run_TIMESTAMP_<nombre>.txt` y (si `export_csv=true`) `run_TIMESTAMP_<nombre>.csv` con columnas: `experiment,instance,instance_variant,drones,iterations,ticks,repeat,urgencia,colisiones,tiempo_s,...`
