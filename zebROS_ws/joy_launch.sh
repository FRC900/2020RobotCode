# Would actually go at the bottom of .bashrc
# Runs script to detect joysticks

output=$(ls /dev/input/js0)

if [ "$output" == "/dev/input/js0" ]; then
    export JOYSTICK_ZERO="true"
else
    export JOYSTICK_ZERO="false"
fi

output=$(ls /dev/input/js0)

if [ "$output" == "/dev/input/js1" ]; then
    export JOYSTICK_ONE="true"
else
    export JOYSTICK_ONE="false"
fi
