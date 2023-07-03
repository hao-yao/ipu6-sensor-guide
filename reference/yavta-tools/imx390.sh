export PATH=$PWD:$PWD/jpeg:$PATH
export LD_LIBRARY_PATH=$PWD/jpeg:$LD_LIBRARY_PATH

FRAME_NUM=30
RESOLUTION=1920x1200
FMT=SGRBG10_1X10
VFMT=SGBRG10

WIDTH=$(echo ${RESOLUTION} | awk -F 'x' '{print $1}')
HEIGHT=$(echo ${RESOLUTION} | awk -F 'x' '{print $2}')

media-ctl -r

media-ctl -V "\"imx390 a\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"imx390 b\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"imx390 c\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"imx390 d\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"TI960 a\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"TI960 a\":1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"TI960 a\":2 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"TI960 a\":3 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"TI960 a\":4 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI-2 1\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI-2 1\":1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":2 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":3 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":4 [fmt:$FMT/${RESOLUTION}]"

media-ctl -l "\"imx390 a\":0 -> \"TI960 a\":0[1]"
media-ctl -l "\"imx390 b\":0 -> \"TI960 a\":1[1]"
media-ctl -l "\"imx390 c\":0 -> \"TI960 a\":2[1]"
media-ctl -l "\"imx390 d\":0 -> \"TI960 a\":3[1]"

media-ctl -l "\"TI960 a\":4 -> \"Intel IPU6 CSI-2 1\":0[1]"
media-ctl -l "\"Intel IPU6 CSI-2 1\":1 -> \"Intel IPU6 CSI2 BE SOC 0\":0[1]"

media-ctl -l "\"Intel IPU6 CSI2 BE SOC 0\":1 -> \"Intel IPU6 BE SOC capture 0\":0[5]"
media-ctl -l "\"Intel IPU6 CSI2 BE SOC 0\":2 -> \"Intel IPU6 BE SOC capture 1\":0[5]"
media-ctl -l "\"Intel IPU6 CSI2 BE SOC 0\":3 -> \"Intel IPU6 BE SOC capture 2\":0[5]"
media-ctl -l "\"Intel IPU6 CSI2 BE SOC 0\":4 -> \"Intel IPU6 BE SOC capture 3\":0[5]"

rm frame-*.bin frame-*.bin.jpg

CAPTURE_DEV=$(media-ctl -e "Intel IPU6 BE SOC capture 0")
./yavta -c${FRAME_NUM} -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -Fframe-a-#.bin -f $VFMT &

CAPTURE_DEV=$(media-ctl -e "Intel IPU6 BE SOC capture 1")
./yavta -c${FRAME_NUM} -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -Fframe-b-#.bin -f $VFMT

CAPTURE_DEV=$(media-ctl -e "Intel IPU6 BE SOC capture 2")
./yavta -c${FRAME_NUM} -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -Fframe-c-#.bin -f $VFMT &

CAPTURE_DEV=$(media-ctl -e "Intel IPU6 BE SOC capture 3")
./yavta -c${FRAME_NUM} -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -Fframe-d-#.bin -f $VFMT

for oneframe in `ls *.bin`; do
	echo "Converting $oneframe..."
	jpeg_encoder $oneframe $VFMT $WIDTH $HEIGHT
done
