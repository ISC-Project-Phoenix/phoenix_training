#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

3/9/2023: modified for Phoenix
@author: jaerock
"""

###############################################################################
#

import os
from pathlib import Path
import yaml
import sys

class Config:
    oscar_path = Path(__file__).resolve().parents[1]
    if oscar_path is None:
        exit("ERROR: Config files not found")

    config_name = oscar_path + '/config/' + 'config.yaml'

    with open(config_name) as file:
        config_yaml = yaml.load(file, Loader=yaml.FullLoader)
        print('=======================================================')
        print('               Configuration Settings')
        print('=======================================================')
        neural_net_yaml_name = config_yaml['neural_net']
        print('Neural Net:     \t' + neural_net_yaml_name)
        data_collection_yaml_name = config_yaml['data_collection']
        print('Data Collection:\t' + data_collection_yaml_name)
        run_neural_yaml_name = config_yaml['run_neural']
        print('Run Neural:     \t' + run_neural_yaml_name)
        print('\n')

    # neural_net
    neural_net_yaml = oscar_path + '/config/neural_net/' + neural_net_yaml_name + '.yaml'
    with open(neural_net_yaml) as file:
        neural_net = yaml.load(file, Loader=yaml.FullLoader)

    # data_collection
    data_collection_yaml = oscar_path + '/config/data_collection/' + data_collection_yaml_name + '.yaml'
    with open(data_collection_yaml) as file:
        data_collection = yaml.load(file, Loader=yaml.FullLoader)

    # run_neural
    run_neural_yaml = oscar_path + '/config/run_neural/' + run_neural_yaml_name + '.yaml'
    with open(run_neural_yaml) as file:
        run_neural = yaml.load(file, Loader=yaml.FullLoader)

    def __init__(self): # model_name):
        pass
    
    @staticmethod
    def summary():
        print('=======================================================')
        print('                 System Configuration ')
        print('=======================================================')
        print('                  ::: Neural Net :::')
        print(yaml.dump(Config.neural_net))
        print('                ::: Data Collection :::')
        print(yaml.dump(Config.data_collection))
        print('                  ::: Run Neural :::')
        print(yaml.dump(Config.run_neural))


if __name__ == '__main__':
    Config.summary()