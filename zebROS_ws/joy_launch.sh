# Would actually go at the bottom of .bashrc
# Runs script to detect joysticks

python ~/2019Offseason/zebROS_ws/src/controller_node/scripts/joy_test.py "home/User/dev/input/js0"

if [ "$result" == "Joystick found" ]; then
    export JOYSTICK_ZERO="true"
else
    export JOYSTICK_ZERO="false"
fi

python ~/2019Offseason/zebROS_ws/src/controller_node/scripts/joy_test.py "home/User/dev/input/js1"

if [ "$result" == "Joystick found" ]; then
    export JOYSTICK_ONE="true"
else
    export JOYSTICK_ONE="false"
fi
