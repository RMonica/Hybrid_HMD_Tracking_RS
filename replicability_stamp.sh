#!/bin/bash

# - download dataset -

curl -O https://rimlab.ce.unipr.it/~rmonica/hybrid_hmd_tracking_dataset.zip
unzip -o hybrid_hmd_tracking_dataset.zip -d traj_test_new

# - compute discontinuity detection fraction -

cd traj_test_new/elab
./parse_all.sh
./fit_all_sigmoids.sh

cd ../..

scripts/plot_r1.sh
scripts/plot_r2.sh
scripts/plot_r2_alpha.sh

# - run tracking approach on all trajectories and compute errors -

scripts/run_all_errors2.sh

cd errors_data

./plot.sh

cd ..

cp errors_data/plot_cubi.eps ./fig_16.eps
cp errors_data/plot_prismi.eps ./fig_17.eps


