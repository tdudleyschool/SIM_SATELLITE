# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -pthread -lws2_32

BIN = Timekeeper.exe missionprocessor.exe EPS.exe ACS.exe Propulsion.exe ForcesTorques.exe RidgedBody.exe

# Detect OS
ifeq ($(OS),Windows_NT)
	TERM_CMD = start cmd /K
	RUN_CMD = start cmd /K
	KILL_CMD = taskkill /F /IM
	EXT = .exe
else
	TERM_CMD = gnome-terminal -- bash -c
	RUN_CMD = gnome-terminal -- bash -c
	KILL_CMD = pkill -f
	EXT =
	CXXFLAGS += -lws2_32
endif

all: parallel_build

parallel_build:
	@$(foreach bin,Timekeeper missionprocessor EPS ACS Propulsion ForcesTorques RidgedBody,\
		$(TERM_CMD) "make $(bin)$(EXT)" & \
	)

Timekeeper.exe: Actor/Actor.cpp Timekeeping/main_timekeeper.cpp Timekeeping/Timekeeper.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

missionprocessor.exe: Actor/Actor.cpp Controller/MissionProcessor.cpp Controller/main_missionprocessor.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

EPS.exe: Actor/Actor.cpp EPS/main_EPS.cpp EPS/ElectricalPowerSystem.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

ACS.exe: Actor/Actor.cpp ACS/main_ACS.cpp ACS/AttitudeControlSystem.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

Propulsion.exe: Actor/Actor.cpp Propulsion_System/main_Propulsion.cpp Propulsion_System/PropulsionSystem.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

ForcesTorques.exe: Actor/Actor.cpp ForcesAndTorques/main_ForcesTorques.cpp ForcesAndTorques/ForceTorqueTracker.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

RidgedBody.exe: Actor/Actor.cpp Body/main_ridgedbody.cpp Body/RidgedBodyModule.cpp
	$(CXX) $^ -o $@ $(CXXFLAGS)

run:
	@$(foreach bin,Timekeeper missionprocessor EPS ACS Propulsion ForcesTorques RidgedBody,\
		$(RUN_CMD) "$(bin)$(EXT); exec bash" & \
	)

stop:
ifeq ($(OS),Windows_NT)
	@$(foreach bin,Timekeeper.exe missionprocessor.exe EPS.exe ACS.exe Propulsion.exe ForcesTorques.exe RidgedBody.exe,\
		$(KILL_CMD) $(bin) /T || exit 0;\
	)
else
	@$(foreach bin,Timekeeper missionprocessor EPS ACS Propulsion ForcesTorques RidgedBody,\
		$(KILL_CMD) "./$(bin)$(EXT)";\
	)
endif

clean:
	del /Q $(BIN) 2>nul || rm -f $(BIN)

