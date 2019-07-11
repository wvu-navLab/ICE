#!/bin/bash

CURRDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd && cd .. )"
DIR="$(dirname "$CURRDIR")"

TDIR="$DIR/test"
DDIR="$DIR/data"
BDIR="$DIR/examples/build"

D_LQ_1="$DDIR/gtsam/drive_1_lq.gtsam"
D_LQ_2="$DDIR/gtsam/drive_2_lq.gtsam"
D_LQ_3="$DDIR/gtsam/drive_3_lq.gtsam"
D_HQ_1="$DDIR/gtsam/drive_1_hq.gtsam"
D_HQ_2="$DDIR/gtsam/drive_2_hq.gtsam"
D_HQ_3="$DDIR/gtsam/drive_3_hq.gtsam"

cd "$TDIR"


#######################
#### RUN ALL TESTS ####
#######################


#######################
## L2
#######################
rm -rf "$TDIR/lq/l2" "$TDIR/hq/l2"

echo "running L2 1/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_1" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/lq/l2/01/" > /dev/null

echo "running L2 2/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_1" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/hq/l2/01/" > /dev/null

echo "running L2 3/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_2"  --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/lq/l2/02/" > /dev/null

echo "running L2 4/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_2" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/hq/l2/02/" >  /dev/null

echo "running L2 5/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_3" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/lq/l2/03/" >  /dev/null

echo "running L2 6/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_3" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 0 --dir "$TDIR/hq/l2/03/"  >  /dev/null

#######################
## DCS
#######################
rm -rf "$TDIR/dcs/mm" "$TDIR/dcs/mm"

echo "running DCS 1/6"
"$BDIR/test_gnss_dcs" -i "$D_LQ_1" --writeECEF --writeENU --writeBias --kernelWidth 2.5 --dir "$TDIR/lq/dcs/01/" > /dev/null

echo "running DCS 2/6"
"$BDIR/test_gnss_dcs" -i "$D_HQ_1" --writeECEF --writeENU --writeBias --kernelWidth 2.5 --dir "$TDIR/hq/dcs/01/" > /dev/null

echo "running DCS 3/6"
"$BDIR/test_gnss_dcs" -i "$D_LQ_2" --writeECEF --writeENU --writeBias --kernelWidth 2.5 --dir "$TDIR/lq/dcs/02/" > /dev/null

echo "running DCS 4/6"
"$BDIR/test_gnss_dcs" -i "$D_HQ_2" --writeECEF --writeENU --writeBias --kernelWidth 2.5 --dir "$TDIR/hq/dcs/02/" > /dev/null

echo "running DCS 5/6"
"$BDIR/test_gnss_dcs" -i "$D_LQ_3" --writeECEF --writeENU --writeBias --kernelWidth 2.5 --dir lq/dcs/03/ > /dev/null

echo "running DCS 6/6"
"$BDIR/test_gnss_dcs" -i "$D_HQ_3" --writeECEF --writeENU --writeBias --kernelWidth 2.50 --dir "$TDIR/hq/dcs/03/" > /dev/null


#######################
## Max-Mix
#######################
rm -rf "$TDIR/lq/mm" "$TDIR/hq/mm"

echo "running MM 1/6"
"$BDIR/test_gnss_maxmix" -i "$D_LQ_1" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/lq/mm/01/" > /dev/null

echo "running MM 2/6"
"$BDIR/test_gnss_maxmix" -i "$D_HQ_1" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/hq/mm/01/" > /dev/null

echo "running MM 3/6"
"$BDIR/test_gnss_maxmix" -i "$D_LQ_2" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/lq/mm/02/" > /dev/null

echo "running MM 4/6"
"$BDIR/test_gnss_maxmix" -i "$D_HQ_2" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/hq/mm/02/" > /dev/null

echo "running MM 5/6"
"$BDIR/test_gnss_maxmix" -i "$D_LQ_3" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/lq/mm/03/" > /dev/null

echo "running MM 6/6"
"$BDIR/test_gnss_maxmix" -i "$D_HQ_3" --writeECEF --writeENU --writeBias --mixWeight 1e-6 --dir "$TDIR/hq/mm/03/" > /dev/null


#######################
## BCE
#######################
rm -rf "$TDIR/lq/bce" "$TDIR/hq/bce"

echo "running BCE 1/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_1" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/lq/bce/01/" > /dev/null

echo "running BCE 2/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_1" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/hq/bce/01/" > /dev/null

echo "running BCE 3/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_2"  --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/lq/bce/02/" > /dev/null

echo "running BCE 4/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_2" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/hq/bce/02/" >  /dev/null

echo "running BCE 5/6"
"$BDIR/test_gnss_bce" -i "$D_LQ_3" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/lq/bce/03/" >  /dev/null

echo "running BCE 6/6"
"$BDIR/test_gnss_bce" -i "$D_HQ_3" --writeECEF --writeENU --writeBias --phaseScale 100 --robustIter 100 --dir "$TDIR/hq/bce/03/"  >  /dev/null

clear
echo -e "\n\n\n\n ----------------------------------------------- \n"
echo -e " run done. :-)  All results were written to ../test"
echo -e  "\n ----------------------------------------------- \n\n\n"
