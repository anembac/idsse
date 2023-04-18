# IDS Sim Env

## Setup
1. Install the crypto++ library (for example using `sudo apt install libcrypto++8 libcrypto++-dev libcrypto++-utils`)
2. Make sure you've installed sumo 1.8.0, ezcar2x and ns-3 version 3.35

## Build
 ```mkdir build && cd build 
cmake -GNinja -DBUILD_DOC=OFF -DWITH_STATIC_CHECKS=OFF -DCMAKE_INSTALL_PREFIX=~/kurser/thesis/local ..
ninja
ninja install
```