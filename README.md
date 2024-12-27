
# Asistencia Robótica Para Hospitales

Trabajo en grupo para la asignatura Robótica de Servicios. Consiste en un prototipo simulado de un sistema de control de un robot móvil en una planta de un hospital.

El sistema presenta una interfaz gráfica (GUI) que permite al usuario controlar al robot por:
- Comandos de voz
- Comandos gestuales
- Selección de destino manual

Además, permite la petición de transporte de algunos objetos.




## Instalación

Para instalar y probar el sistema desarrollado es necesario crear un espacio de trabajo utilizando catkin:
```
mkdir tu_workspace
cd tu_workspace
mkdir src
cd src
git clone https://github.com/lserranoj05/navigation_stage.git
cd ..
catkin_make
source devel/setup.bash
```

Para lanzar la simulación, desde la misma terminal:
```
roslaunch navigation_stage simulacion.launch
```

Finalmente, para lanzar la GUI desde otra terminal:
```
cd tu_workspace/src/navigation_stage/src
python main.py
```
