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

## FAST DEPLOYMENT SNIPPETS

#### Launch
```py
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

DeclareLaunchArgument(name = "parameters_file_path",
                        description ="--params-file (name of the spot to identify)",
                        default_value = os.path.join(get_package_share_directory('ros2_yaml_parser'), 'config', 'spots.yaml')),
arguments=[
            '--params-file', LaunchConfiguration("parameters_file_path"),
            '--spot', LaunchConfiguration("spot_to_find")],

```

#### Script

```py
import argparse
import os
from ament_index_python.packages import get_package_share_directory
from anytree.importer import DictImporter
import yaml

# PARSE ARGUMENTS
parser = argparse.ArgumentParser()
parser.add_argument('--params-file', type=str, default=os.path.join(get_package_share_directory('ros2_yaml_parser'), 'config', 'spots.yaml'), help='Size argument')
args, _ = parser.parse_known_args()
parsed_dictionary = yaml_to_dictionary(args.params_file) 

def yaml_to_dictionary(file_path):
    with open(file_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    parsed_data = DictImporter().import_(yaml_data)
    return parsed_data.ros__parameters
```
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

