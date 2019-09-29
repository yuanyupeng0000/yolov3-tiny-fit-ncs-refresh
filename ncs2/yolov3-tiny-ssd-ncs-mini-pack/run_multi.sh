#!/bin/bash
total_num=$1
for i in $(seq 1 $total_num) 
do  
./run.sh & 
done
