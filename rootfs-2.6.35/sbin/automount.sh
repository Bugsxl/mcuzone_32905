#! /bin/sh

if [ "$1" == "" ]; then
	echo "parameter is none" > /tmp/error.txt
	exit 1
fi

MNT=$1
if echo "$1" | grep mmcblk; then
	if echo "$1" | grep p[25]; then
		MNT=sdcard2
	else
		MNT=sdcard
	fi
else
	if echo "$1" | grep sd; then
		if echo "$1" | grep [25]; then
			MNT=nandcard2
		else
			MNT=nandcard
		fi
	fi
fi

mounted=`mount | grep $1 | wc -l`
#echo "par=$1,mounted=$mounted,MNT=$MNT" > /dev/console

# mounted, assume we umount
if [ $mounted -ge 1 ]; then
	echo "R/mnt/$1" >> /tmp/usbmnt.log
	echo "R/mnt/$1" > /tmp/fifo.1
	if ! umount -l "/mnt/$1"; then
		exit 1
	else
		rm -f "/mnt/$MNT"
		echo "[Umount FS]: /dev/$1 -X-> /mnt/$MNT" > /dev/console
	fi

	if ! rmdir "/mnt/$1"; then
		exit 1
	fi
# not mounted, lets mount under /mnt
else
	if ! mkdir -p "/mnt/$1"; then
		exit 1
	fi

#try jffs2 first
	if ! mount -t jffs2 "/dev/$1" "/mnt/$1"; then
#try vfat
		if ! mount -t vfat -o noatime,shortname=mixed,utf8 "/dev/$1" "/mnt/$1"; then
# failed to mount, clean up mountpoint
			if ! rmdir "/mnt/$1"; then
				exit 1
			fi
			exit 1
		else
			ln -s /mnt/$1 /mnt/$MNT
			echo "[Mount VFAT]: /dev/$1 --> /mnt/$MNT" > /dev/console
			echo "A/mnt/$1" >> /tmp/usbmnt.log
			echo "A/mnt/$1" > /tmp/fifo.1
		fi
	else
		echo "[Mount JFFS2]: /dev/$1 --> /mnt/$MNT" > /dev/console
		echo "A/mnt/$1" >> /tmp/usbmnt.log
		echo "A/mnt/$1" > /tmp/fifo.1
	fi
fi

