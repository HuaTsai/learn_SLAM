# learn_SLAM
視覺SLAM十四講

## Install Sohpus: nontemplate (double) version
```
git clone https://github.com/strasdat/Sophus.git
git checkout a621ff
vim sophus/so2.cpp
change line 32 and 33 to unit_complex_ = std::complex<double>(1,0);
mkdir build
cd build
cmake ..
make
```
Need not `make install`