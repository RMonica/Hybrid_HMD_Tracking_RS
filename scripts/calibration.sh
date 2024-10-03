#!/bin/bash

PREFIX=$1

../build/tracking_parser_extract_matrices ${PREFIX} m_motive.txt m_oculus.txt || exit 1

ros2 run rimlab_extrinsic_calibration solve_eye_in_hand_from_file "m_motive.txt" "m_oculus.txt" --sampling 1 --min-rot-diff 2.0 --min-transl-diff 0.0 --invertB --invertA --ransac --ransac-iterations 100 --ransac-sample-size 4 --ransac-rot-th 1.0 --ransac-transl-th 0.01 --refinement-iterations 1 --output "correction.txt"
