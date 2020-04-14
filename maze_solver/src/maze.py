import os
import signal
import subprocess
import time
# The os.setsid() is passed in the argument preexec_fn so
# it's run after the fork() and before  exec() to run the shell.

maze_solver_cmd = "roslaunch maze_solver amazing.launch"
maze_solver_process = subprocess.Popen(maze_solver_cmd, stdout=subprocess.PIPE, shell=True, executable='/bin/bash', preexec_fn=os.setsid) 

teleop_cmd = "roslaunch racecar teleop.launch"
teleop_process = subprocess.Popen(teleop_cmd, stdout=subprocess.PIPE, shell=True, executable='/bin/bash', preexec_fn=os.setsid) 

wall_follower_cmd = "roslaunch wall_follower wall_follower_maze.launch"
wall_follower_process = subprocess.Popen(wall_follower_cmd, stdout=subprocess.PIPE, shell=True, executable='/bin/bash', preexec_fn=os.setsid) 

time.sleep(60)
os.killpg(os.getpgid(wall_follower_process.pid), signal.SIGTERM)

pure_pursuit_cmd = "roslaunch maze_solver pure_pursuit.launch"
pure_pursuit_process = subprocess.Popen(pure_pursuit_cmd, stdout=subprocess.PIPE, shell=True, executable='/bin/bash', preexec_fn=os.setsid) 