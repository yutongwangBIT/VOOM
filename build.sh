echo "Configuring and building Thirdparty libraries ..."
echo "BoW2 ..."

cd Thirdparty
cd DBoW2
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ../..


echo "BoW2 ..."
cd g2o
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ../..

echo "Osmap ..."
cd Osmap
sh build.sh
cd ../..

echo "Extracting vocabulary file ..."
cd Vocabulary
tar -xvf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building OA-SLAM ..."
mkdir build
cd build
cmake ..
make -j10
