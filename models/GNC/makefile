# Compiler
CXX = g++

# Source directories
SRC_DIR = src
TEST_DIR = test

# Executables
GYRO_EXEC = gyrotest_program
ACCEL_EXEC = acceltest_program

# Source files
GYRO_SRC = $(SRC_DIR)/gyroscope.cpp $(TEST_DIR)/gyroscope_test.cpp
ACCEL_SRC = $(SRC_DIR)/accelerometer.cpp $(TEST_DIR)/accelerometer_test.cpp

# Compilation rules
all: $(GYRO_EXEC) $(ACCEL_EXEC)

$(GYRO_EXEC): $(GYRO_SRC)
	$(CXX) $^ -o $@

$(ACCEL_EXEC): $(ACCEL_SRC)
	$(CXX) $^ -o $@

# Run rules
run_gyro: $(GYRO_EXEC)
	./$(GYRO_EXEC)

run_accel: $(ACCEL_EXEC)
	./$(ACCEL_EXEC)

# Clean rule
clean:
	rm -f $(GYRO_EXEC) $(ACCEL_EXEC) $(filter-out read_me.txt, $(wildcard *.txt)) $(wildcard *.png)
