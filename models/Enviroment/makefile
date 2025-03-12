# Compiler
CXX = g++

# Source directories
SRC_DIR = src
TEST_DIR = test

# Executables
GRAV_FORCE_EXEC = grav_force_program
CELESTIAL_EXEC = celestial_program

# Source files
GRAV_FORCE_SRC = $(SRC_DIR)/gravitational_force.cpp $(TEST_DIR)/grav_force_test.cpp
CELESTIAL_SRC = $(SRC_DIR)/cilestial_body.cpp $(TEST_DIR)/celestial_test.cpp

# Compilation rules
all: $(GRAV_FORCE_EXEC) $(CELESTIAL_EXEC)

$(GRAV_FORCE_EXEC): $(GRAV_FORCE_SRC)
	$(CXX) $^ -o $@

$(CELESTIAL_EXEC): $(CELESTIAL_SRC)
	$(CXX) $^ -o $@

# Run rules
run_grav_force: $(GRAV_FORCE_EXEC)
	./$(GRAV_FORCE_EXEC)

run_celestial: $(CELESTIAL_EXEC)
	./$(CELESTIAL_EXEC)

# Clean rule
clean:
	rm -f $(GRAV_FORCE_EXEC) $(CELESTIAL_EXEC) $(filter-out read_me.txt, $(wildcard *.txt)) $(wildcard *.png)

