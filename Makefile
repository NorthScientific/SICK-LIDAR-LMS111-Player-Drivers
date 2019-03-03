LIBPATH = 
INCPATH = libplayercore
INCPATH += -Ilibplayerxdr

CFLAGS = -Dlinux32 -Wall -fpic -g3 -I$(INCPATH) `pkg-config --cflags playercore`
LDFLAGS += -Wl,-rpath,$(LIBPATH) #-L$(LIBPATH)
CC = gcc
OBJ = sicklms100.o
OBJ += lms100_cola.o

TARGET = libsicklms100.so

all: $(TARGET)

clean:
	rm -f *.o *.so.0

$(TARGET): $(OBJ)
	$(CC) $? -shared -o $@ $(LDFLAGS)

%.o: %.cc
	$(CC) $(CFLAGS) -c $? -o $@
