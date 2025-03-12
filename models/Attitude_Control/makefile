# Compiler
CXX = g++

# Source files
SRC_DIR = src
TEST_DIR = test

# Executables
MOTOR_TEST_EXEC = motortest_program
WHEEL_TEST_EXEC = wheeltest_program

# Source files
MOTOR_TEST_SRC = $(SRC_DIR)/motor.cpp $(SRC_DIR)/control_wheels.cpp $(TEST_DIR)/motor_test.cpp
WHEEL_TEST_SRC = $(SRC_DIR)/control_wheels.cpp $(TEST_DIR)/control_wheels_test.cpp

# Compilation rules
all: $(MOTOR_TEST_EXEC) $(WHEEL_TEST_EXEC)

$(MOTOR_TEST_EXEC): $(MOTOR_TEST_SRC)
	$(CXX) $^ -o $@

$(WHEEL_TEST_EXEC): $(WHEEL_TEST_SRC)
	$(CXX) $^ -o $@

# Run rules
run_motor: $(MOTOR_TEST_EXEC)
	./$(MOTOR_TEST_EXEC)

run_wheel: $(WHEEL_TEST_EXEC)
	./$(WHEEL_TEST_EXEC)

# Clean rule
clean:
	rm -f $(MOTOR_TEST_EXEC) $(WHEEL_TEST_EXEC)

