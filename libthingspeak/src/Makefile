CC=gcc
AR=ar
ARFLAGS=rcs
DEBUG=
CFLAGS=-c -Wall -pedantic -I. $(DEBUG)
DEPS=*.h
LDFLAGS=
SOURCES=thingspeak.c ts_http.c ts_time.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=test
STATIC_LIB=ts.a

all: $(STATIC_LIB)
	
$(STATIC_LIB): $(OBJECTS) 
	$(AR) $(ARFLAGS) $(STATIC_LIB) $(OBJECTS)

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(STATIC_LIB)
