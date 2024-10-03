Hybrid_HMD_Tracking_RS
======================

This library contains implementations of 6DoF tracking approaches which merge two tracking sources for a virtual reality headset. One source (the so-called inside-out tracking) is always available, but it is relative to an arbitrary reference frame. The other source (the so-called outside-in tracking) is fixed with respect to a global reference frame, but it may become temporarily unavailable.

The code was used for the article:

```
R. Monica, D. L. Rizzini and J. Aleotti,
"Adaptive Complementary Filter for Hybrid Inside-Out Outside-In HMD Tracking With Smooth Transitions,"
IEEE Transactions on Visualization and Computer Graphics,
doi: 10.1109/TVCG.2024.3464738
```

Several tracking approaches have been implemented. They are interchangeable as they follow a common interface (`src/tracking_interface.h`).
In particular, the approach proposed in the article is mostly contained in the file `src/directed_complementary_tracking.cpp`.

This repository also contains scripts to generate Fig. 11, Fig. 12, Fig. 13, Fig. 16 and Fig. 17 of the article, as required by the [Graphics Replicability Stamp Initiative](https://www.replicabilitystamp.org/).

The software is released under the 3-clause BSD license. For portability, the repository also contains a copy of the [Eigen3](https://eigen.tuxfamily.org/) library, which is released under the Mozilla Public License v. 2.0, and [Manif](https://artivis.github.io/manif/), which is released under the MIT license.

The dataset collected in the article is also available at <https://rimlab.ce.unipr.it/~rmonica/hybrid_hmd_tracking_dataset.zip>, and it is released under a Creative Commons Attribution 4.0 International License.

Installation
------------

The tracking library is written in C++ and it is cross-platform. It should compile on both Windows and Linux using CMake.

Scripts to generate the figures of the paper require Linux (tested on Ubuntu 22.04 and 24.04) and the following dependencies:

**Dependencies**

- g++
- cmake
- bash
- gnuplot
- R (Rscript executable)
- curl
- unzip

Installation using Ubuntu APT:
```
sudo apt install g++ cmake gnuplot-x11 r-base curl unzip
```

**Build**

Standard CMake build:

```
mkdir build
cd build
cmake ..
make
```

These commands are also listed in the script `build.sh`.

Replicability stamp
-------------------

This code can be used to replicate the results presented in Figures 11, 12, 13, 16 and 17 of the article
```
R. Monica, D. L. Rizzini and J. Aleotti,
"Adaptive Complementary Filter for Hybrid Inside-Out Outside-In HMD Tracking With Smooth Transitions," 
IEEE Transactions on Visualization and Computer Graphics,
doi: 10.1109/TVCG.2024.3464738
```

Run the script `replicability_stamp.sh`.
The script does the following:

1. Downloads the dataset of the user study from <https://rimlab.ce.unipr.it/~rmonica/hybrid_hmd_tracking_dataset.zip>.
2. Parses the dataset and collects the discontinuities reported by the users as a function of the current tracking approach.
3. Generates Fig. 11, Fig. 12 and Fig. 13.
4. Simulates the tracking methods presented in the paper on each user trajectory recorded in the dataset.
5. Uses the mean position and rotation error to produce Fig. 16 and Fig. 17.

