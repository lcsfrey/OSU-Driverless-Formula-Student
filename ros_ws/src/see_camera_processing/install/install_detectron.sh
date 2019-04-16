!#/bin/bash

echo "Installing COCO API..."
COCOAPI="../src/cocoapi"
git clone https://github.com/cocodataset/cocoapi.git $COCOAPI
cd $COCOAPI/PythonAPI
# Install into global site-packages
#make install
# Alternatively, if you do not have permissions or prefer
# not to install the COCO API into global site-packages
python setup.py install --user
cd ../..

echo "Installing Detectron..."
DETECTRON="detectron"
#mkdir -p $DETECTRON
#git clone https://github.com/facebookresearch/detectron $DETECTRON
cd $DETECTRON
ls
pip install -r requirements.txt
make
python detectron/tests/test_spatial_narrow_as_op.py

