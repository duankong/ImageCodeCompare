#!/bin/bash
# Run this from within a bash shell

curdir=$PWD
./clean.sh

cd $curdir/lencod
make

cd $curdir/ldecod
make
