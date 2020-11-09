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

echo_and_run cd "/home/zmc/Desktop/master_arbeit/src/preparing"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zmc/Desktop/master_arbeit/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zmc/Desktop/master_arbeit/install/lib/python2.7/dist-packages:/home/zmc/Desktop/master_arbeit/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zmc/Desktop/master_arbeit/build" \
    "/usr/bin/python2" \
    "/home/zmc/Desktop/master_arbeit/src/preparing/setup.py" \
     \
    build --build-base "/home/zmc/Desktop/master_arbeit/build/preparing" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zmc/Desktop/master_arbeit/install" --install-scripts="/home/zmc/Desktop/master_arbeit/install/bin"
