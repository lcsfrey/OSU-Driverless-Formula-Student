#!/bin/bash

cd ../src/ssds/
pip install -r ./requirements.txt
echo "export PYTHONPATH=$PYTHONPATH:$PWD" >> ~/.bashrc
