echo "Installing prerequisites ..."
cd src/perception/dialogsystem/CMU
sudo apt-get install gcc automake autoconf libtool bison swig python-dev libpulse-dev
echo "Installing sphinxbase ..."
mkdir sphinx-source
cd sphinx-source/
git clone https://github.com/cmusphinx/sphinxbase.git
./autogen.sh
make
sudo make install
echo "Compiling package ..."
catkin build dialogsystem
echo "registring package ..."
source devel/setup.sh
echo "changing nodes to executables ..."
echo "Enter the su password ..."
sudo chmod +777 src/perception/dialogsystem/nodes/*.*
