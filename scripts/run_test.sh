#!/bin/bash
sh_dir=$( dirname -- "$( readlink -f -- '$0'; )"; )
cd $sh_dir && cd ../build

declare -a detector=(SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT)
declare -a descriptor=(BRIEF ORB FREAK AKAZE SIFT)

for i in ${detector[@]}; do
  for j in ${descriptor[@]}; do
    ./test "$i" "$j"
  done
done
