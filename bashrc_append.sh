
# local function to source a file, if it exists
function sourceifexists {
  if [ -e "$1" ]; then
    source "$1"
    echo "sourced $1"
  fi
}

# exported function to source the workspace
function sw {
  sourceifexists /opt/ros/foxy/setup.bash
  sourceifexists /root/lab1_ws/install/local_setup.bash
  sourceifexists /root/lab2_ws/install/local_setup.bash
  sourceifexists /root/lab3_ws/install/local_setup.bash
  sourceifexists /root/lab4_ws/install/local_setup.bash
  sourceifexists /root/lab5_ws/install/local_setup.bash
  sourceifexists /root/lab6_ws/install/local_setup.bash
  sourceifexists /root/lab7_ws/install/local_setup.bash
  sourceifexists /root/lab8_ws/install/local_setup.bash
  sourceifexists /root/sim_ws/install/local_setup.bash
}
export -f sw  # export to make it available
sw            # cource the workspace
