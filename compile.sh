# Project build script 
#
# Two separate binaries:
# - drone_sys;
# - operator;

set -e
mkdir -p build

SRCS="drone_sys.c flight_ctrl.c telemetry.c gps_ctrl.c accelerometer.c battery.c watchdog.c"
CC=gcc
CFLAGS="-Wall -Wextra -O2"
LDFLAGS="-lm"

echo "Compiling drone_sys..."
$CC $CFLAGS -I.             \
    $SRCS                   \
    -o build/drone_sys      \
    $LDFLAGS

echo "Compiling operator..."
$CC $CFLAGS -I. operator.c -o build/operator $LDFLAGS

echo "Done."

