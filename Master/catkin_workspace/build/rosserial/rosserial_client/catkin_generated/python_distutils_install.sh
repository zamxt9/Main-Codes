#!/bin/sh -x

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

cd "/home/ubuntu/catkin_workspace/src/rosserial/rosserial_client"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/ubuntu/catkin_workspace/install/lib/python2.7/dist-packages:/home/ubuntu/catkin_workspace/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/catkin_workspace/build" \
    "/usr/bin/python" \
    "/home/ubuntu/catkin_workspace/src/rosserial/rosserial_client/setup.py" \
    build --build-base "/home/ubuntu/catkin_workspace/build/rosserial/rosserial_client" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ubuntu/catkin_workspace/install" --install-scripts="/home/ubuntu/catkin_workspace/install/bin"
