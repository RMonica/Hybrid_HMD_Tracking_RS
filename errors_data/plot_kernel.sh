#!/usr/bin/env gnuplot

ERROR_PLOT_PREFIX=system("echo \"$ERROR_PLOT_PREFIX\"")

set term eps enhanced color size 9cm, 4cm font 'Times,10'
set output "plot_".ERROR_PLOT_PREFIX.".eps"
#set term pdf enhanced color size 9cm, 6cm font 'Times,10'
#set output "plot_".ERROR_PLOT_PREFIX.".pdf"

set multiplot layout 1, 2

set tmargin 2
set bmargin 3.25
set lmargin 5
set rmargin 2

#set format y "%06.3f"

NUM_NUM=6

array NAMES[NUM_NUM]
NAMES[1]="No motion capture"
NAMES[2]="EKF only"
NAMES[3]="EKF + Compl. filter ᾶ=3.627·10^{-3}"
NAMES[4]="EKF + Compl. filter ᾶ=0.404·10^{-3}"
NAMES[5]="EKF + Adaptive compl. filter"
NAMES[6]="EKF + Smoothing filter"
array FILENAMES[NUM_NUM]
FILENAMES[1]="oculus_first"
FILENAMES[2]="kalman_filter"
FILENAMES[3]="complementarykwparam_alpha0.003627"
FILENAMES[4]="complementarykwparam_alpha0.000404"
FILENAMES[5]="constr_complementarykw"
FILENAMES[6]="directed_complementarykw"
array COLORS[NUM_NUM]
COLORS[1]="black"
COLORS[2]="orange"
COLORS[3]="green"
COLORS[4]="dark-green"
COLORS[5]="blue"
COLORS[6]="red"

DIR="./"

LINE_SIZE=3
DASH_TYPE=1

REVEAL_AT=200

set xrange [-100:400]

set key font ",8"
set key left Left reverse

set xlabel "Frame number" offset 0, 0.25

unset key

set title "Position error (m)" offset 0, -0.5
set yrange [-0.01:0.25]

set obj rect from -100, graph 0 to 0, graph 1 fc rgb "black" fs solid 0.20 noborder
set arrow from 0, graph 0 to 0, graph 1 nohead dt 3 lw 1.5 lc rgb "#555555"
set arrow from graph 0, first 0 to graph 1, first 0 nohead dt 3 lw 1.5 lc rgb "#555555"

set tics front

plot for [i=1:NUM_NUM] DIR."errors_".ERROR_PLOT_PREFIX."_".FILENAMES[i].".txt" using ($1-REVEAL_AT):($2) with lines lc rgb COLORS[i] dt DASH_TYPE lw LINE_SIZE title NAMES[i]
     
set key

set yrange [-0.5:13]
     
set title "Rotation error (deg)" offset 0, -0.5
plot for [i=1:NUM_NUM] DIR."errors_".ERROR_PLOT_PREFIX."_".FILENAMES[i].".txt" using ($1-REVEAL_AT):($3 * 180 / 3.14) with lines lc rgb COLORS[i] dt DASH_TYPE lw LINE_SIZE title NAMES[i]

set yrange [-0.005:0.140]

set key
set key at -86,0.13

set ytics 0.03

set title "Position difference (m)" offset 0, -0.5
#plot for [i=1:NUM_NUM] DIR."errors_".ERROR_PLOT_PREFIX."_".FILENAMES[i].".txt" using ($1-REVEAL_AT):($4) with lines lc rgb COLORS[i] dt DASH_TYPE lw LINE_SIZE title NAMES[i]

set yrange [-0.25:5]

unset key 

set ytics auto

set title "Rotation difference (deg)" offset 0, -0.5
#plot for [i=1:NUM_NUM] DIR."errors_".ERROR_PLOT_PREFIX."_".FILENAMES[i].".txt" using ($1-REVEAL_AT):($5 * 180 / 3.14) with lines lc rgb COLORS[i] dt DASH_TYPE lw LINE_SIZE title NAMES[i]



