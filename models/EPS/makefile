# Compiler
CXX = g++

# Source directories
SRC_DIR = src
TEST_DIR = test

# Executables
BATT_EXEC = batttest_program
BUS_EXEC = bus_program
SOLAR_CELL_EXEC = solar_cell_program
SOLAR_TEST_EXEC = solar_test_program

# Source files
BATT_SRC = $(SRC_DIR)/Battery.cpp $(TEST_DIR)/battery_test.cpp
BUS_SRC = $(SRC_DIR)/bus.cpp $(TEST_DIR)/bus_test.cpp
SOLAR_CELL_SRC = $(SRC_DIR)/solar_cell.cpp $(TEST_DIR)/solar_cell_test.cpp
SOLAR_TEST_SRC = $(SRC_DIR)/Solar_Power_System.cpp $(TEST_DIR)/solar_power_test.cpp

# Compilation rules
all: $(BATT_EXEC) $(BUS_EXEC) $(SOLAR_CELL_EXEC) $(SOLAR_TEST_EXEC)

$(BATT_EXEC): $(BATT_SRC)
	$(CXX) $^ -o $@

$(BUS_EXEC): $(BUS_SRC)
	$(CXX) $^ -o $@

$(SOLAR_CELL_EXEC): $(SOLAR_CELL_SRC)
	$(CXX) $^ -o $@

$(SOLAR_TEST_EXEC): $(SOLAR_TEST_SRC)
	$(CXX) $^ -o $@

# Run rules
run_batt: $(BATT_EXEC)
	./$(BATT_EXEC)

run_bus: $(BUS_EXEC)
	./$(BUS_EXEC)

run_solar_cell: $(SOLAR_CELL_EXEC)
	./$(SOLAR_CELL_EXEC)

run_solar_test: $(SOLAR_TEST_EXEC)
	./$(SOLAR_TEST_EXEC)

# Clean rule
clean:
	rm -f $(BATT_EXEC) $(BUS_EXEC) $(SOLAR_CELL_EXEC) $(SOLAR_TEST_EXEC) $(filter-out read_me.txt, $(wildcard *.txt)) $(wildcard *.png)
