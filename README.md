# slam_algorithm_comparasion

* launching data collection nodes in the package directory

    ```bash
    roslaunch slam_algorithm_comparasion all_listeners.launch
    ```

* to initialize a topic that receives `base_footprint` position data from the simulator, include the gazebo plugin in the `urdf` file of the robot description

    ```xml
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>10</updateRate>
        <bodyName>base_footprint</bodyName>
        <topicName>ground_truth/state</topicName>
        <gaussianNoise>0.01</gaussianNoise>
        <frameName>world</frameName>
        <xyzOffsets>0 0 0</xyzOffsets>
        <rpyOffsets>0 0 0</rpyOffsets>
    </plugin>
    ```
