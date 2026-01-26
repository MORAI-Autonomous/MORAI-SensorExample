export QT_API=pyside2
export LD_LIBRARY_PATH="/home/morai20/.local/lib/python3.8/site-packages/PySide2/Qt/lib:$LD_LIBRARY_PATH"
source /opt/ros/noetic/setup.bash
source ~/MORAI-SensorExample/devel/setup.bash
/usr/bin/python3 src/sensor_example/morai_sensor_viewer.py