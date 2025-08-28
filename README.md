# microROS-demo

This repository provides a minimal demonstration of [micro-ROS](https://micro.ros.org) running on an **ESP32** microcontroller.  
The example implements a simple **LED control** application that can be commanded from a ROS 2 system using the micro-ROS agent.  

Both **serial (USB)** and **UDP (Wi-Fi)** transports are demonstrated, making this project a clear starting point for exploring micro-ROS integration with embedded devices.  

---

## Features
- Subscribes to `/led_cmd` to receive ON/OFF commands from ROS 2.  
- Publishes the LED status on `/led_state`.  
- Example for both **serial** and **Wi-Fi** transport modes.  
- Provides a simple, extendable foundation for more advanced robotics applications.  

---

## Requirements
- **Operating System**: Ubuntu 24.04 (tested)  
- **Hardware**: ESP32 development board  
- **Software**:
  - [ROS 2](https://docs.ros.org/) (tested with Jazzy) 
  - [PlatformIO](https://platformio.org/)  
    - Install using the instructions in this [gist](https://gist.github.com/thetrung/4c323357b6dedfa3abe7fe07bd1a3779) for Ubuntu 24.04  
    - Add the **VS Code extension** for PlatformIO for a full IDE experience 

---

## Setup & Usage

### 1. Flash the ESP32

Flash the ESP32 with PlatformIO. From the project root run:

```bash
pio run -t upload
```

This compiles and uploads the code defined in `src/main.cpp`.

Monitor the ESP32 serial output:

```bash
pio device monitor
```

### 2. Run the micro-ROS Agent

On your host machine, start the micro-ROS agent so it can communicate with the ESP32:

**Serial transport (USB):**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

**Wi-Fi transport (UDP):**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### 3. Send ROS 2 Commands

Once the agent is running and the ESP32 is flashed, you can interact with it through ROS 2 topics:

**Turn the LED ON:**
```bash
ros2 topic pub -1 /led_cmd std_msgs/msg/Bool "{data: true}"
```

**Turn the LED OFF:**
```bash
ros2 topic pub -1 /led_cmd std_msgs/msg/Bool "{data: false}"
```

**Echo the LED state:**
```bash
ros2 topic echo /led_state
```

### 4. Verify Communication

To verify that the ESP32 is properly connected and communicating with ROS 2:

**List available topics:**
```bash
ros2 topic list
```

You should see `/led_cmd` and `/led_state` in the output.

**Check topic information:**
```bash
ros2 topic info /led_cmd
ros2 topic info /led_state
```

## Topics

- `/led_cmd` - Subscribe to LED commands (std_msgs/msg/Bool)
- `/led_state` - Publish current LED state (std_msgs/msg/Bool)

## Troubleshooting

- If the ESP32 doesn't connect, check the serial port (`/dev/ttyUSB0` or `/dev/ttyACM0` on Linux)
- Ensure the micro-ROS agent is running before powering on the ESP32
- For Wi-Fi transport, make sure the ESP32 and host machine are on the same network
- Use `pio device monitor` to see debug output from the ESP32

## Hardware Setup

Connect an LED to the ESP32:
- LED positive (anode) → GPIO pin (default: GPIO2)
- LED negative (cathode) → GND through a 220Ω resistor



