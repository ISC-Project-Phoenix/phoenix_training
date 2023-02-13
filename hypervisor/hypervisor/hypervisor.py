import rclpy
from rclpy import Node
import os
import subprocess
import shutil
from statistics import fmean

from std_msgs.msg import UInt16, String


class TrainingManager():
    def __init__(self):
        pass


class DataManager():
    def __init__(self):
        pass


class HypervisorNode(Node):
    def __init__(self) -> None:
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

        # Variables needed throughout the different functions
        self.current_run_folder = None
        self.current_run_name = None
        self.average_score = 0

        # Start the first data collection, the rest will be handled in callbacks
        self._launch_ros()

    
    def _listener_cb(self, msg: UInt16) -> None:
        self.get_logger().info(f'Run finished with score of {msg.data}')
        self.complete_runs += 1

        self.ros_process.terminate()
        try:
            self.timer.reset()
        except:
            self.get_logger().fatal('Timer was unable to be reset!')
            exit(-1)
        
        if self.current_run_name is None:
            self.get_logger().error(f'Current run name not set, unable to trim run {self.complete_runs}')
        else:
            # Save the runs name and associated score for trimming
            self.runs[self.current_run_name] = msg.data

        # Start collecting more data if not at max runs yet, otherwise start the NN training
        if self.complete_runs < self.max_runs:
            self._launch_ros()
        else:
            # Commented out lines below need to be replaced by function calls to the classes, need to save the new model path
            # self._trim_data()
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
