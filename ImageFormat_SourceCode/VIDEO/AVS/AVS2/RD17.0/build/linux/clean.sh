#!/bin/bash
# Run this from within a bash shell
curdir=$PWD

# remove *.exe
bindir=$curdir/../../source/bin
rm -rf $bindir/lencod.exe
rm -rf $bindir/ldecod.exe

# remove dependencies & tags
rm -rf $curdir/lencod/dependencies
rm -rf $curdir/lencod/tags
rm -rf $curdir/ldecod/dependencies
rm -rf $curdir/ldecod/tags

# remove obj
objdir=$curdir/lencod/obj
rm -rf $objdir
objdir=$curdir/ldecod/obj
rm -rf $objdir
