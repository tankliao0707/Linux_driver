#!/bin/bash
insmod "../driver/gy_521.ko"
sleep 2
chmod 777 "/dev/gy_521"
./App $1
rmmod "gy_521"
