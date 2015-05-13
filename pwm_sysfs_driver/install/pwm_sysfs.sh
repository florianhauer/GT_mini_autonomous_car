#! /bin/sh
#

case "$1" in
  start)
	echo "PWM SYSFS Start"
	echo 0 > /sys/devices/platform/tegra-pwm/pwm/pwmchip0/export
	echo 2 > /sys/devices/platform/tegra-pwm/pwm/pwmchip0/export
	sleep 1
	chmod -R 777 /sys/devices/platform/tegra-pwm/pwm/pwmchip0/pwm0
	chmod -R 777 /sys/devices/platform/tegra-pwm/pwm/pwmchip0/pwm2
	;;

  stop)
	echo "PWM SYSFS Stop"
	echo 0 > /sys/devices/platform/tegra-pwm/pwm/pwmchip0/unexport
	echo 2 > /sys/devices/platform/tegra-pwm/pwm/pwmchip0/unexport
	;;
esac

exit 0;
