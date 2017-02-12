CXXFLAGS =	-O2 -g -fmessage-length=0 -Wno-unused-result -std=c++11

CXX = g++

OBJS = test.o

SRCS = test.cpp Undistort.cpp

TARGET = run.o


all:	$(TARGET)

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS)

debug:
	$(CXX) -D_DEBUG -o $(TARGET) $(SRCS)

clean:
	rm -f $(OBJS) $(TARGET)

run:
	make clean
	make all
	./run.o
