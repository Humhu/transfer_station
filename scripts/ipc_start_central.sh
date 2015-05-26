#! /bin/bash

if [ -e ~/boeing/ipc/bin/Linux-3.13 ]
then
  cd ~/boeing/ipc/bin/Linux-3.13
fi

export CENTRALHOST=$1
./central
