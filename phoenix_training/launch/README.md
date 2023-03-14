The launch files for phoenix training attempt to use the files from phoenix_gazebo as much as possible. This is done
to make it harder for each of phoenixes launch files to de-sync over time.

The only nodes that should be added launch files in this directory are nodes that are also in this same git repo, such as
run_mgr, since they will be used nowhere else.

Hypervisor will use these launch files internally to spawn a run, so nothing here is designed to be run itself (outside of testing).
To start training you should simply run hypervisor directly.