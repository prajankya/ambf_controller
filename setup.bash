#!/bin/bash

if [ ! -d ".env" ]; then
  echo "Virtual Environment not found, creating new Virtual ENV..."
  printf "\n"

  virtualenv -p python3 .env
  pip3 install -r requirements.txt

  printf "\n\n\n"
fi

if [ -d ".env" ]; then
  if [ -z ${PYTHON_OLD+x} ]; then

    PYTHON_OLD=$PYTHONPATH
    export PYTHON_OLD
    unset PYTHONPATH
    echo "Activating..";
    . .env/bin/activate
  else
    echo "You are already in Virtual Environment";
    echo "Deactivating. ...";
    PYTHONPATH=$PYTHON_OLD
    unset PYTHON_OLD

    deactivate

    export PYTHONPATH
  fi
fi

echo "done."
