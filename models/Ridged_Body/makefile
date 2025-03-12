# Compiler
CXX = g++

# Source files
SRC_DIR = src
TEST_DIR = test

# Executables
RIDGED_BODY_EXEC = test_ridged_program
SATELLITE_EXEC = test_program

# Source files
RIDGED_BODY_SRC = $(SRC_DIR)/Ridged_Body.cpp $(SRC_DIR)/Satellite_Box.cpp $(TEST_DIR)/Ridged_Body_test.cpp
SATELLITE_SRC = $(SRC_DIR)/Satellite_Box.cpp $(TEST_DIR)/Sattelite_test.cpp

# Compilation rules
all: $(RIDGED_BODY_EXEC) $(SATELLITE_EXEC)

$(RIDGED_BODY_EXEC): $(RIDGED_BODY_SRC)
	$(CXX) $^ -o $@

$(SATELLITE_EXEC): $(SATELLITE_SRC)
	$(CXX) $^ -o $@

# Run rules
run_ridged: $(RIDGED_BODY_EXEC)
	./$(RIDGED_BODY_EXEC)

run_satellite: $(SATELLITE_EXEC)
	./$(SATELLITE_EXEC)

# Clean rule
clean:
	rm -f $(RIDGED_BODY_EXEC) $(SATELLITE_EXEC) $(filter-out read_me.txt, $(wildcard *.txt))
