# workspaces/

Each ROS 2 workspace lives as a subfolder here (for now just
`biguasim_ws/`). `docker/run_container.sh` mounts this whole folder into the
container at `/home/biguauser/workspaces`, so edits on the host are visible
inside immediately without rebuilding the image.

Each workspace's `src/` is versioned with the repo; `build/`, `install/`,
`log/` are gitignored (see `.gitignore` here).
