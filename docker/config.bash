#!/bin/bash

apt update
apt install nano -y
apt install git -y
apt install python3-pip -y
apt upgrade python3-pip -y 
python3 -m pip install -U pip
python3 -m pip install --upgrade setuptools pip
python3 -m pip install numpy
python3 -m pip install netifaces
python3 -m pip install pyparticleio
python3 -m pip install pandas
python3 -m pip install matplotlib
python3 -m pip install scikit-build
python3 -m pip install opencv-python
python3 -m pip install open3d --ignore-installed PyYAML
python3 -m pip install scipy
python3 -m pip install ruamel.yaml
python3 -m pip install pathlib
python3 -m pip install cvxopt
