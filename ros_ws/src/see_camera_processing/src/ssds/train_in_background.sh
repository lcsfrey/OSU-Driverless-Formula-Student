#!/bin/bash

if [ -z $1 ]
then
    echo "first argument must be config name"
    exit 1
fi

if [ -z $2 ]
then
    OUTPUT_LOG='train_log.txt'
else
    OUTPUT_LOG=$2
fi

mkdir -p logs
python train.py --cfg=$1 > logs/$OUTPUT_LOG 2>&1 &
