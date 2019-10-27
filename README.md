# Approximate POMDP Planning Online (APPL Online) Toolkit

[Copyright &copy; 2014-2019 by National University of Singapore](http://motion.comp.nus.edu.sg/).

APPL Online is a C++ implementation of the IS-DESPOT algorithm for online POMDP planning [1], [2]. It takes as input a POMDP model specified by a C++ API for interfacing directly with a blackbox simulator. 

For bug reports and suggestions, please email <yuanfu@comp.nus.edu.sg>.

[1] Luo, Y., Bai, H., Hsu, D., & Lee, W. S. (2019). [**Importance sampling for online planning under uncertainty**](https://journals.sagepub.com/doi/full/10.1177/0278364918780322). The International Journal of Robotics Research, 38(2–3), 162–181. 

[2] Luo, Y., Bai, H., Hsu, D., & Lee, W. S. (2016). [**Importance sampling for online planning under uncertainty**](https://motion.comp.nus.edu.sg/2018/04/23/online-pomdp-planning/). In Algorithmic Foundations of Robotics XII – Proc. Int. Workshop on the Algorithmic Foundations of Robotics (WAFR). 2016.

## Table of Contents

* [Requirements](#requirements)
* [Download](#download)
* [Installation](#installation)
* [Quick Start](#quick-start)
* [Documentation](#documentation)
* [Using IS-DESPOT with External Systems](#using-despot-with-external-systems)
* [Package Contents](#package-contents)
* [CMakeLists](#cmakelists)
* [Acknowledgements](#acknowledgements)
* [Bugs and Suggestions](#bugs-and-suggestions)
* [Release Notes](#release-notes)

## Requirements

Tested Operating Systems:

<!--| Linux 14.04| OS X (10.1)  | Windows  |
|:------------- |:-------------:|: -----:|
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)| [![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges) | Not Supported |-->

| Linux       | OS X
| :-------------: |:-------------:|
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)      | [![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges) 

Tested Compilers: gcc | g++ 4.2.1 or above

Tested Hardware: Intel Core i7 CPU, 2.0 GB RAM

## Download

Clone the repository from Github (**Recommended**):
```bash
$ git clone https://github.com/luo-yuanfu/is-despot.git
```
OR manually download the [Zip Files](https://github.com/luo-yuanfu/is-despot/archive/master.zip). For instructions, use this online Github README. 

## Installation

Compile using `make`:
```bash
$ cd is-despot
$ make
```

(Optional): If you prefer using `CMake` see the [CMakeLists](#cmakelists) section.

## Quick Start

IS-DESPOT can be used to solve a POMDP specified in **C++** according to the API. We will illustrate this on the [Asymmetric Tiger](https://bigbird.comp.nus.edu.sg/m2ap/wordpress/wp-content/uploads/2017/01/wafr16a.pdf) problem.

To run Asymmetric Tiger specified in [C++](doc/cpp_model_doc), compile and run: 
```bash
$ cd is-despot/examples/cpp_models/asymmetric_tiger
$ make
$ ./tiger --runs 2
```

This command computes and simulates IS-DESPOT's policy for `N = 2` runs and reports the
performance for the Asymmetric Tiger problem specified in C++. See [doc/Usage.txt](doc/usage.txt) for more options.


## Documentation

Documentation can be found in the "[doc](doc/)" directory. 

For a description of our example domains and more POMDP problems see [the POMDP page](http://www.pomdp.org/examples/).

## Using IS-DESPOT with External Systems

Like DESPOT, IS-DESPOT can be used with external systems. An example of integrating DESPOT with an external Gazebo simulator can be found in [the DESPOT tutorials page](https://github.com/AdaCompNUS/despot_tutorials.git). To use IS-DESPOT with external systems, follow the instruction there and replace the despot package with this is-despot package. Note that you also need to specify the importance distribution to use IS-DESPOT.

## Package Contents

```
Makefile                  Makefile for compiling the solver library
README.md                 Overview
include                   Header files
src/core                  Core data structures for the solvers
src/solvers               Solvers, including despot, pomcp and aems
src/pomdpx                Pomdpx and its parser
src/util                  Math and logging utilities
license                   Licenses and attributions
examples/cpp_models       POMDP models implemented in C++
examples/pomdpx_models    POMDP models implemented in pomdpx
doc/pomdpx_model_doc      Documentation for POMDPX file format
doc/cpp_model_doc         Documentation for implementing POMDP models in C++
doc/usage.txt             Explanation of command-line options
doc/eclipse_guide.md      Guide for using Eclipse IDE for development
```

## CMakeLists

**(Optional)**

If you are interested in integrating IS-DESPOT into an existing CMake project or using an IDE for editing, we provide a [CMakeLists.txt](CMakeLists.txt).

To install IS-DESPOT libraries and header files into your system directory:
```bash
$ cd is-despot
$ mkdir build; cd build
$ cmake ../
$ make
$ sudo make install
```

To integrate IS-DESPOT into your project, add this to your `CMakeLists.txt` file:

```CMake
find_package(Despot CONFIG REQUIRED)

add_executable("YOUR_PROJECT_NAME"
  <your_src_files>
)

target_link_libraries("YOUR_PROJECT_NAME"
  despot
)
```

## Acknowledgements

The package is implemented based on [DESPOT code](https://github.com/AdaCompNUS/despot).
Pocman implementation and memorypool.h in the package are based on David Silver's [POMCP code](http://www0.cs.ucl.ac.uk/staff/D.Silver/web/Applications.html)

## Bugs and Suggestions
Please use the issue tracker.

## Release Notes
2016/07/23 Initial release.

2019/10/27 Public release.

