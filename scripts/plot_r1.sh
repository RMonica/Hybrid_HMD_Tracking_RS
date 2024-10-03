#!/usr/bin/env gnuplot

set term eps enhanced size 20cm, 8cm font 'Times,10'
set output "fig_11.eps"

DIR="traj_test_new/elab/"

set multiplot layout 2, 5

set lmargin 4
set bmargin 3.1
set tmargin 3.5

set yrange [-0.1:1.1]

unset key

sigmoid(x, a, b, c) = a * (1 - exp(-(b * x)**c))

sigmoid_from_file(x, filename)=(\
  sigmoid_a = system("cat ".filename." | awk '{print $1}'"),\
  sigmoid_b = system("cat ".filename." | awk '{print $2}'"),\
  sigmoid_c = system("cat ".filename." | awk '{print $3}'"),\
  sigmoid(x, sigmoid_a, sigmoid_b, sigmoid_c)\
)

#sigmoid_from_file(x, "sigmoid_cubi_MODE_COMPLEMENTARY_PARAM_ALPHA.txt") with lines,

INTERS_SIZE=0.7
INTERS_TYPE=13
INTERS_COLOR="orange"
SIGMOID_COLOR="orange"
SIGMOID_DT=1
SIGMOID_SIZE=3
LINE_SIZE=2
LINE_SIZE2=5
LINE_COLOR="black"
UB_COLOR="red"
LB_COLOR="blue"
LB5_COLOR="dark-green"
LB1_COLOR="dark-cyan"
POINT_TYPE2=13
POINT_SIZE2=0.5

pthreshold005= system("cat ".DIR."pthresholds_prismi.txt | cut -d \" \" -f 1")
pthreshold001= system("cat ".DIR."pthresholds_prismi.txt | cut -d \" \" -f 2")

set obj 104 rect from graph 0, graph 0 to graph 1, first pthreshold005 fc rgb "blue" fs solid 0.50 noborder
set obj 105 rect from graph 0, first pthreshold005 to graph 1, first pthreshold001 fc rgb "green" fs solid 0.50 noborder

set tics front


# ----------- PRISMS --------------

MAX_X=1.1
set xrange [0:MAX_X]

set xlabel "{/Times-Italic C}^{ ls} (mm)"

set title "Static translation tolerance (Task 1)"
plot DIR."gnuplot_prismi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt" using ($1*1000):($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR title "Detection fraction",\
     DIR."gnuplot_prismi_MOTIVE_FIRST.txt" using (0):($1/$2):(MAX_X):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE title "EKF only",\
     DIR."gnuplot_prismi_OCULUS_FIRST.txt" using (0):($1/$2):(MAX_X):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE title "No motion capture"
#     DIR."pthresholds_prismi.txt" using (0):($1):(MAX_X):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE title "No motion capture ({/Times-Italic p} {/Helvetica <} 0.05)",\
#     DIR."pthresholds_prismi.txt" using (0):($2):(MAX_X):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE title "No motion capture ({/Times-Italic p} {/Helvetica <} 0.01)"

set xrange [0:0.35]

set key top left Left reverse
set key font ",8"
set key at 0.01, 0.85

set xlabel "{/Times-Italic C}^{ ll}"

set title "Dynamic translation tolerance (Task 1)"
plot DIR."gnuplot_prismi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR title "Detection fraction",\
     DIR."gnuplot_prismi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE title "EKF only",\
     DIR."gnuplot_prismi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE title "No motion capture",\
     sigmoid_from_file(x, DIR."sigmoid_prismi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT title "Weibull function",\
     DIR."inters_prismi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR notitle
#     DIR."pthresholds_prismi.txt" using (0):($1):(0.35):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE title "{/Times-Italic p} {/Helvetica <} 0.05",\
#     DIR."pthresholds_prismi.txt" using (0):($2):(0.35):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE title "{/Times-Italic p} {/Helvetica <} 0.01",\ #


unset key

set xrange [0:0.11]

set xlabel "{/Times-Italic C}^{ rs} (deg)"

set title "Static rotation tolerance (Task 1)"
plot DIR."gnuplot_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_prismi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_prismi_OCULUS_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_prismi.txt" using (0):($1):(0.11):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_prismi.txt" using (0):($2):(0.11):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:0.35]

set xlabel "{/Times-Italic C}^{ rr}"

set title "Dynamic rotation tolerance (Task 1)"
plot DIR."gnuplot_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_prismi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_prismi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_prismi.txt" using (0):($1):(0.35):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_prismi.txt" using (0):($2):(0.35):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:11]

set xlabel "{/Times-Italic C}^{ rc} (deg/m)"

set title "Curvature tolerance (Task 1)"
plot DIR."gnuplot_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_CURV_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_prismi_MOTIVE_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_prismi_OCULUS_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_CURV_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_prismi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_CURV_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_prismi.txt" using (0):($1):(11):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_prismi.txt" using (0):($2):(11):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

# ----------- CUBES --------------

unset obj 104
unset obj 105

pthreshold005= system("cat ".DIR."pthresholds_cubi.txt | cut -d \" \" -f 1")
pthreshold001= system("cat ".DIR."pthresholds_cubi.txt | cut -d \" \" -f 2")

set obj 104 rect from graph 0, graph 0 to graph 1, first pthreshold005 fc rgb "blue" fs solid 0.5 noborder
set obj 105 rect from graph 0, first pthreshold005 to graph 1, first pthreshold001 fc rgb "green" fs solid 0.5 noborder

MAX_X=1.1
set xrange [0:MAX_X]

set xlabel "{/Times-Italic C}^{ ls} (mm)"

set title "Static translation tolerance (Task 2)"
plot DIR."gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt" using ($1*1000):($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(MAX_X):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(MAX_X):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x/1000, DIR."sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt" using ($1*1000):2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_cubi.txt" using (0):($1):(0.35):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_cubi.txt" using (0):($2):(0.35):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:0.35]

set xlabel "{/Times-Italic C}^{ ll}"

set title "Dynamic translation tolerance (Task 2)"
plot DIR."gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_cubi.txt" using (0):($1):(0.35):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_cubi.txt" using (0):($2):(0.35):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:0.11]

set xlabel "{/Times-Italic C}^{ rs} (deg)"

set title "Static rotation tolerance (Task 2)"
plot DIR."gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_cubi.txt" using (0):($1):(0.11):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_cubi.txt" using (0):($2):(0.11):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:0.35]

set xlabel "{/Times-Italic C}^{ rr}"

set title "Dynamic rotation tolerance (Task 2)"
plot DIR."gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE,\
     sigmoid_from_file(x, DIR."sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt") with lines lw SIGMOID_SIZE lc rgb SIGMOID_COLOR dt SIGMOID_DT,\
     DIR."inters_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:2 with points ps INTERS_SIZE pt INTERS_TYPE lc rgb INTERS_COLOR
#     DIR."pthresholds_cubi.txt" using (0):($1):(0.35):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_cubi.txt" using (0):($2):(0.35):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE

set xrange [0:11]

set xlabel "{/Times-Italic C}^{ rc} (deg/m)"

set title "Dynamic curvature tolerance (Task 2)"
plot DIR."gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_CURV_LIMIT.txt" using 1:($2/$3) with linespoints pt POINT_TYPE2 ps POINT_SIZE2 lw LINE_SIZE2 lc rgb LINE_COLOR,\
     DIR."gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead lc rgb UB_COLOR lw LINE_SIZE,\
     DIR."gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead lc rgb LB_COLOR lw LINE_SIZE
#     DIR."pthresholds_cubi.txt" using (0):($1):(11):(0) with vectors nohead lc rgb LB5_COLOR lw LINE_SIZE,\
#     DIR."pthresholds_cubi.txt" using (0):($2):(11):(0) with vectors nohead lc rgb LB1_COLOR lw LINE_SIZE
