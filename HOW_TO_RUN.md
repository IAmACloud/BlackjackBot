1. make or go to your catkin workspace directory:
   ```bash
   cd ~/catkin_ws
   ```

Note: if you want to rename the package, change the name of the `beginner_tutorial` folder in the instructions to your desired package name.

2. Download or clone the repository containing the `beginner_tutorial` package.

3. Make a new package (if you haven't already):
   ```bash
   catkin_create_pkg beginner_tutorial
   ```

4. Copy the `beginner_tutorial` folder into the `src` directory of your catkin workspace:
   ```bash
   cp -r /path/to/your/beginner_tutorial ~/catkin_ws/src/

5. build your workspace:
   ```bash
   catkin_make
   ```
6. source your workspace:
   ```bash
   source devel/setup.bash
   ```

7. run the nodes:
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   roscore

   --NEW TERMINAL--
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun beginner_tutorial camera_node.py

   --NEW TERMINAL--
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun beginner_tutorial vision_node.py

   --NEW TERMINAL--
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun beginner_tutorial main.py
   ```