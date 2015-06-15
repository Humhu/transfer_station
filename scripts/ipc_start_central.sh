#! /bin/bash

HINTS=""
HINTS+=" $HOME/boeing/ipc/bin/Linux-3.13"
HINTS+=" $HOME/Software/ipc/bin/Linux-3.13"
HINTS+=" $HOME/Software/ipc/bin/Linux-3.16"

FILE="$(find ${HINTS} -name central -executable -print -quit 2> /dev/null)"

DIR=$(dirname ${FILE})

cd ${DIR}

./central $*
