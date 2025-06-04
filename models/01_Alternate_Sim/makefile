CXX = g++
CXXFLAGS = -std=c++17 -Wall

OBJS = sattelite_runtime.o sim_object.o SatteliteSim.o 
TARGET = sim_exe

all: $(TARGET)

$(TARGET): $(OBJS)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
ifeq ($(OS),Windows_NT)
	del /Q $(TARGET).exe *.o 2> NUL || exit 0
else
	rm -f $(TARGET) *.o
endif