# Compiler
CXX = g++

# Source directories
SRC_DIR = src
TEST_DIR = test

# Executables
XENON_TANK_EXEC = xenon_tank_program
PROPULSION_EXEC = Propulsion_System_program
THRUSTER_EXEC = thruster_program

# Source files
XENON_TANK_SRC = $(SRC_DIR)/xenon_tank.cpp $(TEST_DIR)/tank_test.cpp
PROPULSION_SRC = $(SRC_DIR)/hall_thruster.cpp $(SRC_DIR)/xenon_tank.cpp $(SRC_DIR)/Propulsion_System.cpp $(TEST_DIR)/Propulsion_System_test.cpp
THRUSTER_SRC = $(SRC_DIR)/hall_thruster.cpp $(TEST_DIR)/hall_thruster_test.cpp

# Compilation rules
all: $(XENON_TANK_EXEC) $(PROPULSION_EXEC) $(THRUSTER_EXEC)

$(XENON_TANK_EXEC): $(XENON_TANK_SRC)
	$(CXX) $^ -o $@

$(PROPULSION_EXEC): $(PROPULSION_SRC)
	$(CXX) $^ -o $@

$(THRUSTER_EXEC): $(THRUSTER_SRC)
	$(CXX) $^ -o $@

# Run rules
run_xenon_tank: $(XENON_TANK_EXEC)
	./$(XENON_TANK_EXEC)

run_propulsion: $(PROPULSION_EXEC)
	./$(PROPULSION_EXEC)

run_thruster: $(THRUSTER_EXEC)
	./$(THRUSTER_EXEC)

# Clean rule
clean:
	rm -f $(XENON_TANK_EXEC) $(PROPULSION_EXEC) $(THRUSTER_EXEC) $(filter-out read_me.txt, $(wildcard *.txt))

