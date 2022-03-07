#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hao/motion_tracking_design/src/yolact_ros_msgs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hao/motion_tracking_design/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hao/motion_tracking_design/install/lib/python2.7/dist-packages:/home/hao/motion_tracking_design/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hao/motion_tracking_design/build" \
    "/usr/bin/python2" \
    "/home/hao/motion_tracking_design/src/yolact_ros_msgs/setup.py" \
    egg_info --egg-base /home/hao/motion_tracking_design/build/yolact_ros_msgs \
    build --build-base "/home/hao/motion_tracking_design/build/yolact_ros_msgs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hao/motion_tracking_design/install" --install-scripts="/home/hao/motion_tracking_design/install/bin"
