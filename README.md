# MORAI - ROS sensor example


sensor example check activation of sensors.

## Requirement
### ROS
- Host system for running the simulator : windows 10 이상
- Vitual system(WSL2) for running the sensor example : Ubuntu 20.04
- Python : 3.8.10
- ROS : Noetic


### UDP
- Host system for running the simulator : windows 10 이상
- Guest system for running the sensor example : windows 10 이상
- Python : 3.8.10



## method
### ROS

    cd {example install path}/MORAI-SensorExample
    pip install -r src/sensor_example/requirements.txt
    catkin_make
    source devel/setup.bash
    python or python3 src/sensor_example/Morai_sensor_viewer.py


### UDP

    cd {example install path}/MORAI-SensorExample/src/sensor_example
    pip install -r requirements.txt
    python ./Morai_sensor_viewer.py
    

    
## More information

Please visit [our homepage](https://www.morai.ai/) and check out the MORAI Simulator!

Also you can check our manual at [MORAI Sim manual (EN)](https://help-morai-sim-en.scrollhelp.site/) / [MORAI Sim manual (KR)](https://help-morai-sim.scrollhelp.site/).
