
# ROS 2 - Beginner Projects

## 1. **Introductory Project: TurtleSim Controller**
   - **Objective**: Control the TurtleSim node using ROS 2.
   - **Tasks**:
     - Write a Python publisher to move the turtle using keyboard input.
     - Create a subscriber node to read and log the turtle's position.
   - **Concepts Applied**: Topics, Publishers, Subscribers.

---

## 2. **Basic Service Project: Light Controller**
   - **Objective**: Create a system to turn a virtual light ON/OFF using a custom service.
   - **Tasks**:
     - Define a custom service with `bool` request and response.
     - Implement a service server node to toggle the light.
     - Create a client node to send service requests.
   - **Concepts Applied**: Services, Service Server/Client.

---

## 3. **Custom Interfaces Project: Sensor Data Processor**
   - **Objective**: Simulate sensor data streaming and processing.
   - **Tasks**:
     - Define a custom message for sensor data (e.g., temperature, humidity).
     - Create a publisher node to publish simulated sensor data.
     - Implement a subscriber node to process and log the data.
   - **Concepts Applied**: Custom Messages, Publishers, Subscribers.

---

## 4. **Parameterized Node: Robot Speed Controller**
   - **Objective**: Control a robot's speed dynamically using parameters.
   - **Tasks**:
     - Create a node that reads a parameter (speed) on startup.
     - Implement parameter declaration and dynamic updating.
     - Simulate robot movement using the updated speed value.
   - **Concepts Applied**: Parameters, Node Settings.

---

## 5. **Launch Files Project: Multi-Node System**
   - **Objective**: Automate the startup of multiple nodes using launch files.
   - **Tasks**:
     - Write launch files to start a publisher and subscriber node together.
     - Include parameters in the launch file for dynamic configuration.
   - **Concepts Applied**: Launch Files, Node Configuration.

---

## 6. **Debugging Project: ROS 2 Node Monitor**
   - **Objective**: Monitor and debug nodes in a running ROS 2 system.
   - **Tasks**:
     - Launch nodes in the TurtleSim simulation.
     - Use `rqt_graph` and debugging tools to inspect node interactions.
   - **Concepts Applied**: Debugging Tools.

---

## 7. **Integrated Project: Autonomous TurtleSim Navigator**
   - **Objective**: Create a simple autonomous navigation system for TurtleSim.
   - **Tasks**:
     - Use topics to publish velocity commands.
     - Use services to reset the turtle's position.
     - Utilize parameters to define navigation constraints.
     - Automate node launches using a launch file.
   - **Concepts Applied**: Topics, Services, Parameters, Launch Files.
