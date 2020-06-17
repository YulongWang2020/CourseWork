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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/lorne/ros_wkspace_asgn4/src/assignment4/urdf_parser_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lorne/ros_wkspace_asgn4/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lorne/ros_wkspace_asgn4/install/lib/python2.7/dist-packages:/home/lorne/ros_wkspace_asgn4/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lorne/ros_wkspace_asgn4/build" \
    "/usr/bin/python" \
    "/home/lorne/ros_wkspace_asgn4/src/assignment4/urdf_parser_py/setup.py" \
    build --build-base "/home/lorne/ros_wkspace_asgn4/build/assignment4/urdf_parser_py" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/lorne/ros_wkspace_asgn4/install" --install-scripts="/home/lorne/ros_wkspace_asgn4/install/bin"
