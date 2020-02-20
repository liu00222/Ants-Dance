#!/bin/bash


mkdir -p build
cd build

GENERATORS=`cmake --help | grep Generates | grep Visual | wc -l`

echo $GENERATORS

if [ $GENERATORS -eq 0 ]; then
	CC="gcc"
	CXX="g++"

	if [ -d "/soft/gcc/7.1.0/Linux_x86_64/bin" ]
	then
		CC="/soft/gcc/7.1.0/Linux_x86_64/bin/gcc"
		CXX="/soft/gcc/7.1.0/Linux_x86_64/bin/g++"
                echo $CC $CXX
		#exit 1
	fi

    export CC=$CC
    export CXX=$CXX
    cmake .. -DCMAKE_INSTALL_PREFIX=../../.. -DAUTOBUILD_DEPENDENCIES=ON -DAUTOBUILD_EXECUTE_NOW=ON
else
	if [ -z "$1" ];
	then
		echo 
		echo "List of generators:"
		echo "---------------"
		cmake --help | grep Generates | grep Visual
		echo
		echo "Please specify one of the generators based on your version of Visual Studio."
		echo
		echo "Usage: ./setup.sh \"Visual Studio 12 Win64\""
		exit 1
	else
		cmake .. -G "$1" -DCMAKE_INSTALL_PREFIX=../../.. -DAUTOBUILD_DEPENDENCIES=ON -DAUTOBUILD_EXECUTE_NOW=ON -DCMAKE_GENERATOR_PLATFORM=x64
	fi
fi

cmake --build . --target install
