# ros2_yaml_parser
ROS2 example on how to parse complex yaml files to be used as parameters for a Node based on python.

## Dependencies

This using this package depends on `pyyaml` and `anytree `. You can easily install them both using the following command:

```sh
pip install pyyaml
pip install anytree
```

## ALERT

The code parses the contents of the yaml file as a dictionary.

**Alert:** ⚠️ The first item in the yaml file has to be `ros__parameters:` for the code to work.


The first item in the yaml file is parsed as an object, not as a dictionary by default when using the `anytree` importer. I solved it by forcing the user to use `ros__parameters:` as the first variable and then ignoring it.


## Code Testing

You can easily download this package and test it by downloading it into your `src`

You can easily test the package using the following command to execute the launchfile:
```sh
ros2 launch ros2_yaml_parser yaml_test.launch.py
```

Or check the code for other spots by using another spot label as an input argument:
```sh
ros2 launch ros2_yaml_parser yaml_test.launch.py spot:="corner2"
```

