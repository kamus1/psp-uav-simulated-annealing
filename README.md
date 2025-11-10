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
./PSP-UAV instancias/PSP-UAV_01_a.txt 5 3000 50 --export
```


## Salida esperada
En consola se imprime la urgencia acumulada, la ventana T usada, la cantidad de drones, el tiempo de ejecución y la ruta completa de cada dron (indicando su base de origen). Con `--export` se generan los CSV mencionados en la carpeta `exported_data/`.


Se puede visualizar el resultado con el código de `visualization.ipynb`
