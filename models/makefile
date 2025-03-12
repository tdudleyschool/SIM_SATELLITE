# Subdirectories containing Makefiles
SUBDIRS = Attitude_Control Enviroment EPS GNC Propulsion Recources Ridged_Body

# Default target: Run make in all subdirectories
all:
	@for dir in $(SUBDIRS); do \
		echo "Building in $$dir..."; \
		$(MAKE) -C $$dir; \
	done

# Run clean in all subdirectories
clean:
	@for dir in $(SUBDIRS); do \
		echo "Cleaning in $$dir..."; \
		$(MAKE) -C $$dir clean; \
	done
