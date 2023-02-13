# hypervisor
From package `[hypervisor](https://github.com/ISC-Project-Phoenix/hypervisor)'
# File
`./src/hypervisor.py`

## Summary
Project Phoenix hypervisor node. This manages the collection of data, the trimming of the data, and the training of the neural network model.

## Topics

### Subscribes
- `/order_66`: Kills the ROS2 child process when received, and saves the score provided into a map for data trimming.
- `/run_folder`: Path to the folder that the current run is saved to

## Params
- `data_path`: Directory that runs are being saved in. Defaults to ./training_data
- `model_path`: Directory that contains the JSON and .h5 for the Keras model. Application will exit will fatal error if this path is not set
- `max_runs`: The maximum number of runs that hypervisor will do before training the AI. Defaults to 5