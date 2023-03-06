import rclpy
from rclpy import Node
import os
import subprocess
import shutil
import glob
import pandas as pd
from statistics import fmean, stdev

from std_msgs.msg import UInt16, String


class TrainingManager():
    def __init__(self):
        pass


class DataManager():
    def stddev(self, runs: dict) -> dict:
        mean = fmean(runs.values)
        std_dev = stdev(runs.values)
        data_folders = {}
        for name, score in runs:
            if score < mean - 2 * std_dev:
                if self.logger is not None:
                    self.logger.info(f'Run {name} is being trimmed. Score is {score}')
            else:
                data_folders[name] = score
        return data_folders
            
    
    '''
    This class will handle the data related to the neural network. 

    At the end of every data collection loop, this class will take the runs dictionary, and determine which runs will be used for the neural network training
    The runs dictionary contains the score, where the keys are the names of the folder where the run data is saved. If the score of the run is less then 
    two standard deviations from the mean, the run should be trimmed and not included in the training data set. The method of trimming, in this case the 2 std dev,
    should utilize a ROS parameter that decides which trimming method is used. This will allow us to easily use a different trimming method just by changing the
    ros parameter. Once we have determined which runs can be used for training, we need to put the images into a folder located in tmp and combine the csv files
    into one csv file and place this in the same folder as the images. This folder is the data folder for the AI to train on. 

    FOR ALTERNATE TRIMMING METHODS, WE MUST DEFINE THE METHOD ABOVE, THEN ADD AN ENTRY TO THE TRIMMING METHODS DICT. ALL TRIMMING METHODS SHOULD RETURN A DICT WITH THE
    RUNS TO BE TRAINED ON
    '''
    trimming_methods = {'stddev':stddev}
    def __init__(self, trim_method: String, logger=None) -> None:
        self.logger = logger
        if trim_method not in self.trimming_methods.keys:
            if logger is not None:
                logger.info(f'Trimming method {trim_method} not found! Defaulting to stddev')
            trim_method = 'stddev'
        self._trim_method = trim_method
        if logger is not None:
            logger.info(f'Trimming method set to {trim_method}')
        
    

    def trim_data(self, runs: dict):
        '''
        Need to determine if the training folder created in tmp should be overwritten with new data or not. Assuming yes for now.
        '''
        data = self.trimming_methods[self._trim_method](runs)
        # Temp line for intellisense to work
        data = self.stddev(runs)
        
        if os.path.isdir('/tmp/training_data/'):
            shutil.rmtree('/tmp/training_data/')
        try:
            os.makedirs('/tmp/training_data/')
        except FileExistsError:
            if self.logger is not None:
                self.logger.warning('Unable to create new training data folder in tmp. Folder already exists!')

        csv_files = []
        for path, score in data.items:
            try:
                # Copying all files from the data folders excluding the csv, so should be just the images.
                shutil.copytree(path, '/tmp/training_data/', ignore=shutil.ignore_patterns('*.csv'), dirs_exist_ok=True)
            except:
                # Exit if files couldn't be transferred
                if self.logger is not None:
                    self.logger.fatal(f'Unable to copy the files from {path} to /tmp/training_data/')
                exit(-1)
            csv_files.append(glob.glob('*.csv', root_dir=path))
        
        csv_concat = pd.concat([pd.read_csv(file) for file in csv_files], ignore_index=True)
        csv_concat.to_csv('/tmp/training_data/images.csv', index=False)
        if self.logger is not None:
            self.logger.info("CSV files have been concatanated and written to /tmp/training_data/images.csv")


class HypervisorNode(Node):
    def __init__(self, testing=False) -> None:
        self.testing = testing
        if not self.testing:
            super().__init__('hypervisor')
            
            self.order_sub = self.create_subscription(
                UInt16,
                '/order_66',
                self._listener_cb,
                10)
            
            self.folder_sub = self.create_subscription(
                String,
                '/run_folder',
                self._folder_cb,
                1
            )

            self.timer = self.create_timer(1200, self._timer_cb)
        
        # Gets the path to the data folder
        data_path = self.declare_parameter('data_path', './training_data')
        self.data_folder = os.path.normcase(os.path.normpath(data_path.value))

        self.get_logger().info(f'Using the directory {self.data_folder} as the data folder')

        # Gets the path to the keras model
        model_path = self.declare_parameter('model_path')
        if model_path is None:
            self.get_logger().fatal('Keras model path not set!')
            exit(-1)
        self.model = model_path.value

        self.get_logger().info(f'Using the model {self.model} as the initial keras model')

        # Setting up the maximum number of runs, run counter, and the runs dictionary
        self.max_runs = self.declare_parameter('max_runs', 5).value
        self.complete_runs = 0
        self.runs = {}

        self.get_logger().info(f"Maximum number of runs is set to {self.max_runs}")

        # Variables needed throughout the different functions
        self.current_run_folder = None
        self.current_run_name = None
        self.average_score = 0

        trim = self.declare_parameter('trim_method')

        self.data_manager = DataManager(trim, self.get_logger())

        if not self.testing:
            # Start the first data collection, the rest will be handled in callbacks
            self._launch_ros()

    
    def _listener_cb(self, msg: UInt16) -> None:
        self.get_logger().info(f'Run finished with score of {msg.data}')
        self.complete_runs += 1

        if not self.testing:
            self.ros_process.terminate()
            try:
                self.timer.reset()
            except:
                self.get_logger().fatal('Timer was unable to be reset!')
                exit(-1)
        
        if self.current_run_name is None:
            self.get_logger().error(f'Current run name not set, unable to trim run {self.complete_runs}')
        else:
            # Save the runs folder and associated score for trimming
            self.runs[self.current_run_folder] = msg.data

        # Start collecting more data if not at max runs yet, otherwise start the NN training
        if self.complete_runs < self.max_runs:
            self._launch_ros()
        else:
            # Commented out lines below need to be replaced by function calls to the classes, need to save the new model path
            self.data_manager.trim_data(self.runs)
            # self.model = self._train_model()

            avg_score = fmean(self.runs.values())
            if avg_score == self.average_score:
                self.get_logger().info(f'Average score of {avg_score} has not changed, exiting training')
                exit(0)
            self.average_score = avg_score

    
    def _folder_cb(self, msg: String) -> None:
        self.current_run_folder = msg.data
        self.current_run_name = msg.data[msg.data.rfind('/'):]
        self.get_logger().info(f'Current run folder is {self.current_run_folder}')
    

    def _timer_cb(self) -> None:
        # When this callback is triggered, it means the run ran longer than 20 minutes which points to a failure of the training cycle.
        self.get_logger().fatal('Run didn\'t finish within 20 minutes!')
        exit(-1)
        

    def _launch_ros(self) -> None:
        # This function will launch the ROS2 training process and save it as a field. Must not block, otherwise the callbacks won't work
        # Still need to implement random maps as launch argument to Popen function
        ros = shutil.which('ros2')
        if ros is None:
            self.get_logger().fatal('Ros2 not found in system!')
            exit(-1)
        try:
            # Prior to next line, need to implement the random gazebo map
            self.ros_process = subprocess.Popen([ros, 'launch', 'phoenix_training', 'training.py.launch', f'model:=\'{self.model}\''])
        except OSError:
            self.get_logger().fatal('Ros2 subprocess failed to start')
            exit(-1)
        self.get_logger().info(f'Ros2 process started with PID {self.ros_process.pid}')


def main(args=None) -> None:
    rclpy.init(args=args)

    hypervisor = HypervisorNode()
    rclpy.spin(hypervisor)

    hypervisor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
