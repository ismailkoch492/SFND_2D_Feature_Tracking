#!/bin/bash
sh_dir=$( dirname -- "$( readlink -f -- "$0" )"; )
cd $sh_dir && cd ../build

run_project() {
  local detector=(SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT)
  local descriptor=(BRIEF ORB FREAK AKAZE SIFT)
  local chkDet="false"
  local chkDesc="false"
  
  if [ "$#" -eq 0 ]; then
    echo "Running the program with the default args: Det:ORB, Desc:FREAK"
    ./2D_feature_tracking ORB FREAK
  elif [ "$#" -eq 1 ]; then
    for arg in "${detector[@]}"; do
      arg="${arg// /}"
      if [ "$arg" == "$1" ]; then
        chkDet="true"
        break
      fi
    done

    if [ "$chkDet" == "true" ]; then
      echo "Running the program with: Det:$1, Desc:FREAK"
      ./2D_feature_tracking "$1" FREAK
    else
      echo "Add the appropriate detector type, $1 is invalid. The valid types are ${detector[@]}"
    fi
  else
    for arg in "${detector[@]}"; do
      arg="${arg// /}"
      if [ "$arg" == "$1" ]; then
        chkDet="true"
        break
      fi
    done

    for arg in "${descriptor[@]}"; do
      arg="${arg// /}"
      if [ "$arg" == "$2" ]; then
        chkDesc="true"
        break
      fi
    done

    if [[ "$chkDet" == "true" && "$chkDesc" == "true" ]]; then
      echo "Running the program with: Det:$1, Desc:$2"
      ./2D_feature_tracking "$1" "$2"
    elif [[ "$chkDet" != "true" && "$chkDesc" != "true" ]]; then
      echo "Add the appropriate detector type, $1 is invalid. The valid types are: ${detector[@]}"
      echo "Add the appropriate descriptor type, $2 is invalid. The valid types are: ${descriptor[@]}"
    elif [ "$chkDet" != "true" ]; then
      echo "Add the appropriate detector type, $1 is invalid. The valid types are: ${detector[@]}"
    elif [ "$chkDesc" != "true" ]; then
      echo "Add the appropriate descriptor type, $2 is invalid. The valid types are: ${descriptor[@]}"
    fi
  fi
}

run_project "$@"
