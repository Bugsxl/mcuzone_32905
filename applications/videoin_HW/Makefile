.SUFFIXES : .x .o .c .s

ROOT = /usr/local/arm_linux_4.2/
LIB = $(ROOT)/lib/gcc/arm-linux/4.2.1
LIB1 =$(ROOT)/arm-linux/lib
INCSYS:=$(ROOT)/arm-linux/sys-include
INC :=$(ROOT)/arm-linux/include

#ROOT =
#LIB =
#LIB1 =
#INCSYS:= .
#INC := .


CC=arm-linux-gcc -O2 -I$(INC) -I$(INCSYS) -Wall --static
#CC=gcc -O2 -I$(INC) -I$(INCSYS) --static
WEC_LDFLAGS=-L$(LIB) -L$(LIB1) -L.
STRIP=arm-linux-strip
#STRIP=strip

TARGET = vin_demo
SRCS := vin_demo.c JpegEnc.c dsc.c V4L.c exif.c
LIBS= -lc -lgcc -lc -lpthread

all: 
	$(CC) $(WEC_LDFLAGS) $(SRCS) -o $(TARGET) $(LIBS)
	$(STRIP) $(TARGET) 
	#cp $(TARGET) ../../rootfs-2.6.35/usr/
	#cp $(TARGET) ../../initramfs/usr/.
	echo "Please copy execution file to storage"

clean:
	rm -f *.o 
	rm -f *.x 
	rm -f *.flat
	rm -f *.map
	rm -f temp
	rm -f *.img
	rm -f $(TARGET)	
	rm -f *.gdb
	rm -f *.bak
