#!/bin/bash

#CONDA_ENV='DV'
#CONDA_ENV_DIR='~/anaconda3/envs/':${CONDA_ENV}

#DV_DIR='~/driverless_dev/'
#PY_ENV_DIR=${DV_DIR}:'DV_env/'

pip install --user \
      future \
      numpy \
      protobuf \
      typing \
      hypothesis \
      pyyaml \
      pybind11 \
      cython
      
sudo apt-get install -y --no-install-recommends \
      libgflags-dev \
      cmake

sudo apt-get install -y --no-install-recommends \
      build-essential \
      git \
      libgoogle-glog-dev \
      libgtest-dev \
      libiomp-dev \
      libleveldb-dev \
      liblmdb-dev \
      libopencv-dev \
      libopenmpi-dev \
      libsnappy-dev \
      libprotobuf-dev \
      openmpi-bin \
      openmpi-doc \
      protobuf-compiler \
      python-dev \
      python-pip  

pip install --user numpy pyyaml mkl mkl-include setuptools cffi typing
pip install --user -c mingfeima mkldnn 


echo "Installing Pytorch..."
PYTORCH_DIR="../src/pytorch"
git clone https://github.com/pytorch/pytorch.git $PYTORCH_DIR

cd $PYTORCH_DIR
git submodule update --init --recursive
#git submodule sync
USE_OPENCV=1 python setup.py install
#python setup.py build

#python setup.py install
 
#export CMAKE_PREFIX_PATH=CMAKE_PREFIX_PATH:${CONDA_ENV_DIR}
#export CMAKE_PREFIX_PATH=CMAKE_PREFIX_PATH:${PY_ENV_DIR} 

python -c 'from caffe2.python import core' 2>/dev/null && echo "CAFFE2 INSTALL SUCCESS" || echo "ERROR: CAFFE2 INSTALL FAILURE"
python -c 'from caffe2.python import workspace; print("Number of GPUs:", workspace.NumCudaDevices())'


      


