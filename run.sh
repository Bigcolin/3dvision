export FPATH='/Users/kyan/Projects/3dpcl/data/dragon_stand'
export WPATH='/Users/kyan/Projects/3dpcl/pcl0'

cmake --build $WPATH/build/.
$WPATH/build/pcd_test 1.0 1.5, 2.0