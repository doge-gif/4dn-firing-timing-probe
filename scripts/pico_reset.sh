#!/usr/bin/env bash
# Force a running 4dn-probe gate to BOOTSEL via the CDC 1200-baud touch.
# Works whenever core0 is alive (services USB) -- no physical button needed.
set -u
TTY=""
for t in /dev/ttyACM*; do
	[ -e "$t" ] || continue
	udevadm info -q property -n "$t" 2>/dev/null | grep -q 'ID_VENDOR_ID=2e8a' && {
		TTY="$t"
		break
	}
done
[ -z "$TTY" ] && {
	echo "no 4dn CDC (2e8a) found"
	exit 1
}
echo "1200-baud touch on $TTY"
stty -F "$TTY" 1200
for _ in $(seq 1 20); do
	lsblk -pnr -o NAME,LABEL | grep -q RPI-RP2 && {
		echo "-> BOOTSEL"
		exit 0
	}
	sleep 1
done
echo "-> did not reach BOOTSEL"
exit 1
