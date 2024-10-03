#!/bin/bash

function get_base_names {
  NEWFILES=""
  for file in $1
    do
    newfile="${DIR}/`basename $file`"
    NEWFILES="${NEWFILES} ${newfile}"
    done
  echo -n ${NEWFILES}
}

MODES=(
       "COMPLEMENTARY KW PARAM_ALPHA 0.003627"
       "COMPLEMENTARY KW PARAM_ALPHA 0.000404"
       "DIRECTED_COMPLEMENTARY KW"
       "CONSTR_COMPLEMENTARY KW"
       "OCULUS_FIRST"
       "KALMAN_FILTER"
)

DIR="traj_test_new"
DIR_PARSER="./build"

FILES_PRISMI=`cat "${DIR}/file_prismi.txt"`
NEWFILES_PRISMI=`get_base_names "$FILES_PRISMI"`
FILES_PRISMI_ALPHA=`cat "${DIR}/file_prismi_alpha.txt"`
NEWFILES_PRISMI_ALPHA=`get_base_names "$FILES_PRISMI_ALPHA"`
FILES_PRISMI_SHORT2=`cat "${DIR}/file_prismi_short2.txt"`
NEWFILES_PRISMI_SHORT2=`get_base_names "$FILES_PRISMI_SHORT2"`

for i in ${!MODES[@]}
  do
  mode=${MODES[i]}
  MODE_LOWERCASE=${mode,,}
  MODE_NOSPACES=`echo "$MODE_LOWERCASE" | sed 's/ //g'`
  ${DIR_PARSER}/tracking_parser_compute_errors "MODE_${mode}" 200 600 ${NEWFILES_PRISMI} -s MODE_CONSTR_COMPLEMENTARY_RO -s MODE_CONSTR_COMPLEMENTARY_TO -s MODE_MOTIVE_FIRST \
                                -newgroup ${NEWFILES_PRISMI_ALPHA} -s MODE_COMPLEMENTARY \
                                -newgroup ${NEWFILES_PRISMI_SHORT2} -s MODE_DIRECTED_COMPLEMENTARY -s MODE_MOTIVE_FIRST \
                                -o errors_data/errors_prismi_${MODE_NOSPACES}.txt \
                                -st 1,2,4 -sto errors_data/example_prismi_${MODE_NOSPACES}_ -sti errors_data/sub_trajectory_info_prismi.txt
  done

FILES_CUBI=`cat "${DIR}/file_cubi.txt"`
NEWFILES_CUBI=`get_base_names "$FILES_CUBI"`
FILES_CUBI_ALPHA=`cat "${DIR}/file_cubi_alpha.txt"`
NEWFILES_CUBI_ALPHA=`get_base_names "$FILES_CUBI_ALPHA"`
FILES_CUBI_SHORT2=`cat "${DIR}/file_cubi_short2.txt"`
NEWFILES_CUBI_SHORT2=`get_base_names "$FILES_CUBI_SHORT2"`

for i in ${!MODES[@]}
  do
  mode=${MODES[i]}
  MODE_LOWERCASE=${mode,,}
  MODE_NOSPACES=`echo "$MODE_LOWERCASE" | sed 's/ //g'`
  ${DIR_PARSER}/tracking_parser_compute_errors "MODE_${mode}" 200 600 ${NEWFILES_CUBI} -s MODE_CONSTR_COMPLEMENTARY_RO -s MODE_CONSTR_COMPLEMENTARY_TO -s MODE_MOTIVE_FIRST \
                                -newgroup ${NEWFILES_CUBI_ALPHA} -s MODE_COMPLEMENTARY \
                                -newgroup ${NEWFILES_CUBI_SHORT2} -s MODE_DIRECTED_COMPLEMENTARY -s MODE_MOTIVE_FIRST \
                                -o errors_data/errors_cubi_${MODE_NOSPACES}.txt \
                                -st 1,3,5 -sto errors_data/example_cubi_${MODE_NOSPACES}_ -sti errors_data/sub_trajectory_info_cubi.txt
  done
