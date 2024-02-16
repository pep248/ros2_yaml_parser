#!/usr/bin/env python3
import argparse
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from anytree.importer import DictImporter
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml


class YamlParserNode(Node):

    def __init__(self,node_name) -> None:
        super().__init__(node_name)
        
            # PARSE ARGUMENTS
        parser = argparse.ArgumentParser()
        # args.params_file
        parser.add_argument('--params-file', type=str, default=os.path.join(get_package_share_directory('ros2_yaml_parser'), 'config', 'spots.yaml'), help='Size argument')
        # args.spot
        parser.add_argument('--spot', type=str, default='corner1', help='Square argument')
        args, _ = parser.parse_known_args()
        
        
        # PARSE DICTIONARY
        parsed_dictionary = yaml_to_dictionary(args.params_file) 
        

        # ACCESS ITS VALUES
        # children names and amount example 
        print(parsed_dictionary['children'])
        #{'names': [{'name': 'a'}, {'name': 'b'}, {'name': 'c'}], 'amount': 3}
        for child in parsed_dictionary['children']['names']:
            print(child['name']) 
            # a
            # b
            # c
        print(parsed_dictionary['children']['amount'])
        # 3
        
        # spots example
        for spot in parsed_dictionary['spots']:
            if spot['spot']['label'] == args.spot:
                print(spot['spot']['coordinates'])
                #{'translation': {'x': -0.797222, 'y': -1.18766, 'z': 5.68798e-05}, 'rotation': {'x': 0.000976602, 'y': -0.0022919, 'z': 0.999974, 'w': -0.00679845}}
        

def main(args=None):
    
    # START ROS NODE
    rclpy.init(args=args)
    node = YamlParserNode('yaml_parser_test_node')

    # Create executor and add node
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Run the nodes within the executor
        # executor.spin()
        pass # we won't be spinning the node because we only want to execute the constructor once
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def yaml_to_dictionary(file_path):
    with open(file_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)
    parsed_data = DictImporter().import_(yaml_data)
    return parsed_data.ros__parameters
    

if __name__ == '__main__':
    main()