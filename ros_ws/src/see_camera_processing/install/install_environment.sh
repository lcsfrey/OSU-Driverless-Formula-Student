!#/bin/bash

DV_DIR='~/driverless_dev'
DV_ENV='DV_env'

echo "Creating driverless_dev dir in home directory..."
mkdir $DV_DIR
cd $DV_DIR

sudo apt-get install \
    python-setuptools 
    python-dev 
    build-essential

pip install --user pipenv

echo "Installing DV_env virtual environment..."
python -m virtualenv DV_env

echo "-----------------------------------------------------------\n"
echo "creating alias DV to enter vitrual environment..."
echo "alias DV='source ":${DV_DIR}:${DV_ENV}:"bin/activate'" >> ~/.bashrc
source ~/.bashrc
echo "Finished creating environment.\nActivating virtual environment"
DV
echo "Shortcut created."
echo "In the future, enter DV into terminal to activte virtual environment\n"
echo "Enter deactivate into terminal to leave the virtual envirment"

