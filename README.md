# RTSimIndy7 Task

This source is based on Ubuntu20.04, Xenomai-3.2.3

## Prerequisite
```
$ sudo apt-get install libeigen3-dev libpoco-dev libjsoncpp-dev
```

## build
```
$ mkdir build && cd build
$ cmake ../ && make -j($nproc)
$ sudo -s
$ ./RTIndy7_01
```

