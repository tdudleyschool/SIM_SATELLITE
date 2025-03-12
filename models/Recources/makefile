# Compiler
CXX = g++

# Source files
SRC_DIR = src

# Executable
FORCES_EXEC = forces_program

# Source files
FORCES_SRC = $(SRC_DIR)/force_torque_tracker.cpp $(SRC_DIR)/forces_test.cpp $(SRC_DIR)/functions.cpp

# Compilation rule
all: $(FORCES_EXEC)

$(FORCES_EXEC): $(FORCES_SRC)
	$(CXX) $^ -o $@

# Run rule
run_forces: $(FORCES_EXEC)
	./$(FORCES_EXEC)

# Clean rule
clean:
	rm -f $(FORCES_EXEC)


