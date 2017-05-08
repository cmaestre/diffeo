Added new parameter file (../parameters/diffeoPars.xml) which is parsed using tinyxml (sudo apt-get install libtinyxml-dev)

To build the interface you need cython (sudo -H pip install cython) and then run 
python2 setup.py build_ext --inplace
