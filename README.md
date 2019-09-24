# Robust Incremental State Estimation through Covariance Adaptation

## Overview

This repository contains the software release for "Robust Incremental State Estimation through Covariance Adaptation". The objective of the software release is described through the associated abstract.
<br/>
<br/>


> Recent advances in the fields of robotics and automation have spurred significant interest in robust state estimation. To enable robust state estimation, several methodologies have been proposed. One such technique, which has shown promising performance, is the concept of iteratively estimating a Gaussian Mixture Model (GMM), based upon the state estimation residuals, to characterize the measurement uncertainty model. Through this iterative process, the measurement uncertainty model is more accurately characterized, which enables robust state estimation through the appropriate de-weighting of erroneous observations. This approach, however, has traditionally requires a batch estimation framework to enable the estimation of the measurement uncertainty model, which is not advantageous to most real-time robotic applications. Within this paper, we propose a novel extension to the measurement uncertainty model estimation paradigm. Specifically, we propose an efficient, incremental extension of the methodology. The incremental covariance estimation (ICE) approach, as detailed within this paper, is evaluated on several collected data sets, where it is shown to provide a significant increase in localization accuracy, when compared to other state-of-the-art algorithms. 


<br/>
<br/>

This software benefits from several open-source software packages.
* [*Georgia Tech Smoothing And Mapping (GTSAM)*](https://bitbucket.org/gtborg/gtsam/src/develop/) -- contains factor graph based state estimation algorithms
	* GTSAM was updated for GNSS signal processing within
	    *  [*PPP-BayesTree*](https://github.com/wvu-navLab/PPP-BayesTree) -- contains pseudorange and carrier-phase factors
	    *  [*RobustGNSS*](https://github.com/wvu-navLab/RobustGNSS) -- contains robust GNSS models
* [*GPS Toolkit (GPSTk)*](http://www.gpstk.org/bin/view/Documentation/WebHome) -- contains GNSS observation modeling tools
* [*libcluster*](https://github.com/dsteinberg/libcluster) -- contains variatinal clustering algorithms


<br/>
<br/>
<br/>

<!--
If you utilze this software for an academic purpose, please consider using the following citation:
-->
<!--
```
@article{ watson2019robust,
        title={Robust Incremental State Estimation through Covariance Adaptation},
        author={Watson, Ryan M and Gross, Jason N and Taylor, Clark N and Leishman, Robert C},
        journal={arXiv preprint},
        year={2019}
       }
```
-->

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

git clone https://github.com/wvu-navLab/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation.git

````

### 3) Build

````bash

cd Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation
./build.sh

````

### 4) Test
````bash
cd examples
chmod +x run_all.sh
./run_all.sh
````

This will write all of the generated results to the *test* directory (*../test*). To duplicate the plots generated within the paper run the following command. (Note: this assumes that you have a matlab alias set. See this [link](https://www.mathworks.com/matlabcentral/answers/98220-how-do-i-create-a-shortcut-or-link-to-matlab)  for instructions. ).
