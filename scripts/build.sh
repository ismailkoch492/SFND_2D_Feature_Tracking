sh_dir=$( dirname -- "$( readlink -f -- '$0'; )"; )
cd $sh_dir
rm -rf ../build
mkdir -p ../build
cd ../build
cmake .. && make