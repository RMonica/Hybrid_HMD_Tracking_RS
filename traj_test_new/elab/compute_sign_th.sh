#!/usr/bin/env Rscript

args = commandArgs(trailingOnly=TRUE)

if (length(args) != 3)
{
  stop("Three arguments required: oculus_first_filename num_samples output_file", call. = FALSE)
}

oculus_first_file = args[1]
num_samples = args[2]
filename_out = args[3]

num_samples = strtoi(num_samples)

inters_data = read.csv(oculus_first_file, header = FALSE, colClasses = c("numeric", "numeric"), sep="\t")
oculus_samples = as.double(inters_data[2])
yes_oculus_samples = as.double(inters_data[1])
print("Oculus samples:")
print(oculus_samples)
print("Yes oculus samples:")
print(yes_oculus_samples)

i_for_p_less_than_05 = num_samples
i_for_p_less_than_01 = num_samples
i_for_p_less_than_005 = num_samples
i_for_p_less_than_001 = num_samples
for (i in 1:num_samples) {
  ft = fisher.test(matrix(c(i, num_samples-i, yes_oculus_samples, oculus_samples-yes_oculus_samples), ncol=2))
  print(ft$p.value)
  if (ft$p.value < 0.05 && i_for_p_less_than_05 == num_samples) {
    i_for_p_less_than_05 = i
  }
  if (ft$p.value < 0.01 && i_for_p_less_than_01 == num_samples) {
    i_for_p_less_than_01 = i
  }

  if (ft$p.value < 0.005 && i_for_p_less_than_005 == num_samples) {
    i_for_p_less_than_005 = i
  }

  if (ft$p.value < 0.001 && i_for_p_less_than_001 == num_samples) {
    i_for_p_less_than_001 = i
  }
}

fout <- file(filename_out, "w")
writeLines(sprintf("%f %f %f %f", (i_for_p_less_than_05/num_samples), 
                                  (i_for_p_less_than_01/num_samples), 
                                  (i_for_p_less_than_005/num_samples), 
                                  (i_for_p_less_than_001/num_samples)), con = fout)
close(fout)
