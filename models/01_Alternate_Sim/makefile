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
	rm -f $(TARGET) *.o