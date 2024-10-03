#!/usr/bin/gnuplot --persist

set term eps enhanced size 8, 8 font 'Times,10'
set output "gnuplot_cubi.eps"

set multiplot layout 3, 3

set yrange [-0.1:1.1]

unset key

sigmoid(x, a, b, c) = a * (1 - exp(-(b * x)**c))

sigmoid_from_file(x, filename)=(\
  sigmoid_a = system("cat ".filename." | awk '{print $1}'"),\
  sigmoid_b = system("cat ".filename." | awk '{print $2}'"),\
  sigmoid_c = system("cat ".filename." | awk '{print $3}'"),\
  sigmoid(x, sigmoid_a, sigmoid_b, sigmoid_c)\
)

set xrange [0:0.006]

#sigmoid_from_file(x, "sigmoid_cubi_MODE_COMPLEMENTARY_PARAM_ALPHA.txt") with lines,\

set title "COMPLEMENTARY ALPHA"
plot "gnuplot_cubi_MODE_COMPLEMENTARY_PARAM_ALPHA.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.006):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.006):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_COMPLEMENTARY_PARAM_ALPHA.txt") with lines,\
     "inters_cubi_MODE_COMPLEMENTARY_PARAM_ALPHA.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.006):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.006):(0) with vectors nohead lc "red"

set xrange [0:0.11]

set title "CONSTR COMPLEMENTARY ROTATION LIMIT"
plot "gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.11):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt") with lines,\
     "inters_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.11):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.11):(0) with vectors nohead lc "red"

set xrange [0:0.0011]

set title "CONSTR COMPLEMENTARY TRANSLATION LIMIT"
plot "gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.0011):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.0011):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt") with lines,\
     "inters_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.0011):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.0011):(0) with vectors nohead lc "red"

set xrange [0:0.35]

set title "CONSTR COMPLEMENTARY ROTATION DYN LIMIT"
plot "gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt") with lines,\
     "inters_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_DYN_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.35):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.35):(0) with vectors nohead lc "red"

set xrange [0:11]

set title "CONSTR COMPLEMENTARY ROTATION CURV LIMIT"
plot "gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_RO_PARAM_ROTATION_CURV_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(11):(0) with vectors nohead,\
     "pthresholds_cubi.txt" using (0):($1):(11):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(11):(0) with vectors nohead lc "red"

set xrange [0:0.35]

set title "CONSTR COMPLEMENTARY TRANSLATION DYN LIMIT"
plot "gnuplot_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.35):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt") with lines,\
     "inters_cubi_MODE_CONSTR_COMPLEMENTARY_TO_PARAM_TRANSLATION_DYN_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.35):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.35):(0) with vectors nohead lc "red"

set xrange [0:0.2]

set title "MODE DIR COMPLEMENTARY ROTATION DYN2 LIMIT"
plot "gnuplot_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_ROTATION_DYN2_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.2):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.2):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_ROTATION_DYN2_LIMIT.txt") with lines,\
     "inters_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_ROTATION_DYN2_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.2):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.2):(0) with vectors nohead lc "red"

set xrange [0:6]

set title "MODE DIR COMPLEMENTARY ROTATION CURV2 LIMIT"
plot "gnuplot_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_ROTATION_CURV2_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(6):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(6):(0) with vectors nohead,\
     "pthresholds_cubi.txt" using (0):($1):(6):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(6):(0) with vectors nohead lc "red"

set xrange [0:0.6]

set title "MODE DIR COMPLEMENTARY TRANSLATION DYN2 LIMIT"
plot "gnuplot_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_TRANSLATION_DYN2_LIMIT.txt" using 1:($2/$3) with lines,\
     "gnuplot_cubi_MOTIVE_FIRST.txt" using (0):($1/$2):(0.6):(0) with vectors nohead,\
     "gnuplot_cubi_OCULUS_FIRST.txt" using (0):($1/$2):(0.6):(0) with vectors nohead,\
     sigmoid_from_file(x, "sigmoid_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_TRANSLATION_DYN2_LIMIT.txt") with lines,\
     "inters_cubi_MODE_DIRECTED_COMPLEMENTARY_PARAM_TRANSLATION_DYN2_LIMIT.txt" using 1:2 with points,\
     "pthresholds_cubi.txt" using (0):($1):(0.6):(0) with vectors nohead lc "dark-red",\
     "pthresholds_cubi.txt" using (0):($2):(0.6):(0) with vectors nohead lc "red"
