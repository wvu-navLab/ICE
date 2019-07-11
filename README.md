# Incremental Covariance Estimation for Robust State Estimation


<br/>
<br/>

## How to Install


### 1) Requirements/Recommendations

#### Required
* Boost -->  ```` sudo apt-get install libboost-all-dev ````
* CMake -->  ```` sudo apt-get install cmake ````
* OpenMP --> ```` sudo apt install libomp-dev ````


### 2) Clone repository to local machine  
````bash

git clone https://github.com/wvu-navLab/ICE.git

````

### 3) Build

````bash

cd ICE
./build.sh

````

### 4) Test
````bash
cd examples
chmod +x run_all.sh
./run_all.sh
````

This will write all of the generated results to the *test* directory (*../test*). 
