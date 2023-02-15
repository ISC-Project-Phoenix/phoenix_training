# run_mgr
From package '[phoenix_training](https://github.com/ISC-Project-Phoenix/phoenix_training)'
# File
`./src/run_mgr.cpp`

## Summary 
 Manages the per-run state of phoenix training. This involves collecting the score, as well as ending the run.

Score is currently collected by viewing a colored mask via a camera in gazebo.

## Topics

### Publishes
- `/order_66`: Published to when run is done. This occurs when the score is zero, or the finish line is seen. Contains the runs score.

### Subscribes
- `/run_folder`: Folder to write the final score.txt to. This must be published to before the end of the run.
- `/camera/score/rgb`: Camera viewing the good areas mask.

