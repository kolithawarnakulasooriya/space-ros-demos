## Demo Simulation for NASA Ingenuity Mars Helicopter

This is a demo package for Ingenuity Mars Helicopter developed from (NASA)[https://science.nasa.gov/mission/mars-2020-perseverance/ingenuity-mars-helicopter/]. Basic UAV functions has been added with the Mars environment.

<img src="https://github.com/kolithawarnakulasooriya/space-ros-demos/blob/kolitha-inginuty-demo/inginuity/images/img.png" width=50% height=50%>

## Services

1. Starting rotors : `ros2 service call /start_rotors std_srvs/SetBool "{data: True}"`
2. Stoping rotors : `ros2 service call /stop_rotors std_srvs/SetBool "{data: True}"`
