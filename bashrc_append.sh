
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
  sourceifexists /lab1_ws/install/local_setup.bash
  sourceifexists /lab2_ws/install/local_setup.bash
  sourceifexists /lab3_ws/install/local_setup.bash
  sourceifexists /lab4_ws/install/local_setup.bash
  sourceifexists /lab5_ws/install/local_setup.bash
  sourceifexists /lab6_ws/install/local_setup.bash
  sourceifexists /lab7_ws/install/local_setup.bash
  sourceifexists /lab8_ws/install/local_setup.bash
  sourceifexists /sim_ws/install/local_setup.bash
}
export -f sw  # export to make it available
sw            # cource the workspace
