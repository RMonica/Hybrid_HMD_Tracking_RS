#!/usr/bin/gnuplot --persist

set term postscript enhanced color eps font 'Times-Roman,32' size 16,16
set output "plot.eps"

set multiplot layout 3, 2

set format y "%06.3f"

set xrange [1:1400]
#set xrange [200:1000]
#set xrange [500:700]

set yrange [0:0.25]

NUM_NUM=11

array NAMES[NUM_NUM]
NAMES[1]="OCULUS ORIGIN"
NAMES[2]="CONSTR COMPLEMENTARY"
NAMES[3]="COMPLEMENTARY"
NAMES[4]="MOTIVE FIRST"
NAMES[5]="DOUBLE EXP"
NAMES[6]="EXP"
NAMES[7]="PARTICLE FILTER"
NAMES[8]="KALMAN FILTER"
NAMES[9]="CONSTRAINED KALMAN FILTER"
NAMES[10]="DIR COMPLEMENTARY"
NAMES[11]="OCULUS FIRST"
array FILENAMES[NUM_NUM]
FILENAMES[1]="oculus_origin"
FILENAMES[2]="constr_complementary"
FILENAMES[3]="complementary"
FILENAMES[4]="motive_first"
FILENAMES[5]="double_exp"
FILENAMES[6]="exp"
FILENAMES[7]="particle_filter"
FILENAMES[8]="kalman_filter"
FILENAMES[9]="constrained_kalman_filter"
FILENAMES[10]="dir_complementary"
FILENAMES[11]="oculus_first"
array COLORS[NUM_NUM]
COLORS[1]="red"
COLORS[2]="orange"
COLORS[3]="green"
COLORS[4]="blue"
COLORS[5]="white"
COLORS[6]="white"
COLORS[7]="white"
COLORS[8]="magenta"
COLORS[9]="white"
COLORS[10]="purple"
COLORS[11]="black"

DIR="../../../traj_04052022/traj8/"

set title "position error"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($2) with lines lc rgb COLORS[i] dt 2 title NAMES[i]
     
set yrange [0:20]
     
set title "rotation error"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($3 * 180 / 3.14) with lines lc rgb COLORS[i] dt 2 title NAMES[i]

set yrange [0:0.025]

set title "position delta"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($4) with lines lc rgb COLORS[i] dt 2 title NAMES[i]

set yrange [0:20]

set title "rotation delta"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($5 * 180 / 3.14) with lines lc rgb COLORS[i] dt 2 title NAMES[i]
     
set yrange [0:0.015]

set title "position vibration"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($6) with lines lc rgb COLORS[i] dt 2 title NAMES[i]

set yrange [0:5]

set title "rotation vibration"
plot for [i=1:NUM_NUM] DIR."errors_".FILENAMES[i].".txt" using 1:($7 * 180 / 3.14) with lines lc rgb COLORS[i] dt 2 title NAMES[i]



