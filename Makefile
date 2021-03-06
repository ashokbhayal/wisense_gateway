
################################################
#Makefile for gw.exe
#rkris@wisense.in
################################################

CC=gcc
CFLAGS=-I.
DEPS = *.h
OBJ=gw.o
TARGET=gw.exe
LIBS=-lm -lpthread
TS=n
TS_DIR=./libthingspeak/src
TS_LIB=$(TS_DIR)/ts.a

JSON=n
JSON_DIR=./jansson-2.8/
JSON_LIB=$(JSON_DIR)/src/.libs/libjansson.a

OTHER_LIBS=

RM=rm -f


# [Overriding Variables]
# An argument that contains '=' specifies the value of a variable: ' v = x ' sets 
# the value of the variable v to x . If you specify a value in this way, all ordinary 
# assignments of the same variable in the makefile are ignored; we say they have been 
# overridden by the command line argument.


ifeq ($(JSON),y)
TARGET=gw_json.exe
OTHER_LIBS += $(JSON_LIB)
CFLAGS += -DJSON_ENC_ENA -I$(JSON_DIR)/src
else
ifeq ($(TS),y)
TARGET=gw_ts.exe
OTHER_LIBS += $(TS_LIB)
CFLAGS += -D__THINGSPEAK_SUPPORT_ENA__ -I$(TS_DIR)
else
TARGET=gw.exe
endif
endif

# $@ is the left side of :
# $^ is the right right side of :
# $< is the first item in the dependencies list (right side of :)
#
# DEPS is all the *.h files in this directory

%.o: %.c $(DEPS)
	$(CC) -c $< $(CFLAGS) -o $@

all: $(TARGET) $(OTHER_LIBS)

$(TS_LIB): FORCE
	cd $(TS_DIR) && make


$(JSON_LIB): FORCE
	cd $(JSON_DIR) && make

DUMMY: FORCE
	touch gw.c

FORCE:

$(TARGET): DUMMY $(OBJ) $(OTHER_LIBS) 
	$(CC) $(OBJ) $(LIBS) $(OTHER_LIBS) -o $@

clean:
	rm -rf *.o
	rm -rf *.exe



