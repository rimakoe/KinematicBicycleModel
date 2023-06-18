#!/bin/bash

python3 -V
python3 -m venv ./venv --system-site-packages # If not created, creating virtualenv
./venv/bin/python3 -m pip install -r ./requirements.txt # Install specific packages
