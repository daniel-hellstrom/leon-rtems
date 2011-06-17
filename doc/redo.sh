#!/bin/bash
make clean
make distclean
../bootstrap
./configure --enable-maintainer-mode
cd tools; make
cd ..
make info
make all
