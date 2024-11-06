# layered_hardware_gz
A ros2_control layer implementation for joints of a virtual robot in the Ignition Gazebo simulator. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware/tree/jazzy) to understand the layered scheme.

## Hardware Plugin: layered_hardware_gz/LayeredHardwareGazeboSim
### Hardware parameters
___layers___ (yaml, required)
* sequence of layers to be loaded by this ros2_control hardware

___layers[*].name___ (string, required)
* arbitary name of the layer

___layers[*].type___ (string, required)
* valid type name of the layer
* the type must be exported to the layered_hardware package if its base class is `layered_hardware::LayerInterface`, or to the layered_hardware_gz package if its base class is [`layered_hardware_gz::GazeboSimLayerInterface`](include/layered_hardware_gz/gz_layer_interface.hpp)

### Example of `ros2_control` tag in your robot description
```xml
<ros2_control name="LayeredHardwareGazeboSim" type="system">
    <hardware>
        <plugin>layered_hardware_gz/LayeredHardwareGazeboSim</plugin>
        <param name="layers">
            - name: example_layer_1
              type: layered_hardware/ExampleLayer
            - name: example_layer_2
              type: layered_hardware_gz/GzExampleLayer
            - name: example_layer_3
              ...
            ...
        </param>
        ...
    </hardware>
    ...
</ros2_control>
```

## Layer plugin: layered_hardware_gz/GazeboJointLayer
* sends commands to joints in the Gazebo simulator within `write()` function
* fetches joints' states within `read()` function
* switches joints' operating modes within `perform_command_mode_swtich()` function when controllers using associated interfaces activate

### Hardware parameters
___<layer_name>___ (yaml, required)
* map of parameter names and values for this layer

___<layer_name>.joints___ (map<string, map>, required)
* map of parameters for each joint

___<layer_name>.joints.<joint_name>.initial_position___ (double, default: 0.0)
* joint position is initialized with this value by the layer

___<layer_name>.joints.<joint_name>.operation_mode_map___ (map<string, string>, required)
* map to joint's operating mode names from associated interface names (typically joint interfaces)
* possible operating mode names are 'effort', 'position', & 'velocity'

### Example of parameter description
```yaml
<param name="example_gazebo_sim_joint_layer">
    joints:
        example_joint_1:
            initial_position: 0.5
            operation_mode_map:
                example_joint_1/position: position
                ...
        example_joint_2:
            ...
</param>
```

## Example
* [single joint](examples/single_joint)