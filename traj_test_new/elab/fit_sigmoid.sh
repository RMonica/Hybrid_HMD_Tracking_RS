#!/usr/bin/env Rscript

args = commandArgs(trailingOnly=TRUE)

if (length(args) != 7)
{
  stop("Seven arguments required: filename init_a init_b init_c filename_out intersection_file_out oculus_first_file", call. = FALSE)
}

filename = args[1]
init_a = args[2]
init_b = args[3]
init_c = args[4]
filename_out = args[5]
intersection_file_out = args[6]
oculus_first_file = args[7]

init_b = as.double(init_b)
init_c = as.double(init_c)

print(paste("Processing file: ", filename, sep=""))

data = read.csv(filename, header = FALSE, colClasses = c("numeric", "numeric", "numeric"), sep="\t")
names(data) <- c('x','y','tot')
if (substr(init_a, 1, 1) != "f")
{
  init_a = as.double(init_a)
  #result = nls((y/tot) ~ a/(1 + exp(-b * (x - m))), start=list(a=init_a, b=init_b, m=init_c), data=data, trace=TRUE)
  result = nls((y/tot) ~ a * (1 - exp(-(x*b)**c)), start=list(a=init_a, b=init_b, c=init_c), data=data, trace=TRUE)
  myvector <- coef(result)
  a <- myvector[1]
  b <- myvector[2]
  c <- myvector[3]
} else
{
  forced_a = substr(init_a, 2, 100000)
  forced_a <- as.double(forced_a)
  print(paste("Parsing used forced a = ", forced_a, sep=""))
  #result = nls((y/tot) ~ forced_a/(1 + exp(-b * (x - m))), start=list(b=init_b, m=init_c), data=data, trace=TRUE)
  result = nls((y/tot) ~ forced_a * (1 - exp(-(x*b)**c)), start=list(b=init_b, c=init_c), data=data, trace=TRUE)
  myvector <- coef(result)
  a <- forced_a
  b <- myvector[1]
  c <- myvector[2]
}

print(myvector)

fout <- file(filename_out, "w")
writeLines(sprintf("%f %f %f", a, b, c), con = fout)
close(fout)

inters_data = read.csv(oculus_first_file, header = FALSE, colClasses = c("numeric", "numeric"), sep="\t")
inters = inters_data[1] / inters_data[2]

yinters = inters
#xinters = -log(a / yinters - 1)/b + c
xinters = (-log(-(yinters / a - 1)))**(1/c)/b

fout <- file(intersection_file_out, "w")
writeLines(sprintf("%f %f", xinters, yinters), con = fout)
close(fout)
