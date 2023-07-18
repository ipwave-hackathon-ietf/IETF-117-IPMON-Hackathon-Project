#! /bin/sh

./configure

make MODE=release

cd examples/veins
  # for i in 0 1 2 3 4 5 6 7 8 9 
  for i in 0 1 2 3 4 
  do
	for j in 1 2 3 4 5 6 7 8 9 10 11
	do
		./run -u Cmdenv -f omnetpp.ini -r $i -c Lamda_$j
	
		mv results/collprobres.csv results/collprobres"$i"_"$j".csv
		mv results/accidentres.csv results/accidentres"$i"_"$j".csv

	done
  done
date


