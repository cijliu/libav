#!/bin/bash
PWD=`pwd`
ARCH=linux
OS=linux
ENCODERS=rawvideo
INPUT_DEVICES=v4l2
CROSS_PREFIX=arm-himix100-linux-
INSTALL_DIR=$PWD/build/ 
function configure(){
    $PWD/configure                  \
    --enable-gpl                    \
    --enable-version3               \
    --enable-pthreads               \
    --disable-x86asm                \
    --disable-indevs                \
    --enable-indev=$INPUT_DEVICES   \
    --disable-encoders              \
    --disable-decoders              \
    --enable-encoder=$ENCODERS      \
    --disable-hwaccels              \
    --disable-muxers                \
    --disable-demuxers              \
    --disable-protocols             \
    --disable-filters               \
    --disable-parsers               \
    --enable-parser=h264            \
    --enable-cross-compile          \
    --cross-prefix=$CROSS_PREFIX    \
    --arch=$ARCH                    \
    --target-os=$OS                 \
    --prefix=$INSTALL_DIR           \
    --disable-avconv                \
    --disable-avprobe
}

######################parse arg###################################
b_arg_device=0
b_arg_cross_prefix=0
for arg in $@
do
	if [ $b_arg_device -eq 1 ] ; then
		b_arg_device=0;
		INPUT_DEVICES=$arg;
	fi
    if [ $b_arg_cross_prefix -eq 1 ] ; then
		b_arg_cross_prefix=0;
		CROSS_PREFIX=$arg;
	fi
	case $arg in
		"-indev")
			b_arg_device=1;
			;;
        "-cross_prefix")
			b_arg_cross_prefix=1;
			;;
	esac
done
#######################parse arg end########################
configure
