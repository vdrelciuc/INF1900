################################################################################
##### Makefile to compile the static library
################################################################################

EXEC = librobot.a 

################################################################################
##### Compilation settings and variables
################################################################################

include ../MakefileCommonVariables

AR = avr-ar
ARFLAGS = rcs

################################################################################
##### Targets
################################################################################

.PHONY: all
all: $(BUILD_DIR)/$(EXEC)

# Creation de l'archive
$(BUILD_DIR)/$(EXEC): $(OBJS)
	@echo -n "Building static library"
    ifeq ($(debug),1)
		@echo -n " in debug mode"
    endif
	@echo ": $@"
	@$(AR) $(ARFLAGS) $@ $^
	@echo -n "Receiver: "
	@$(MAKE) -s -C ../receiver clean
	@echo -n "Transmitter: "
	@$(MAKE) -s -C ../transmitter clean

include ../MakefileCommonTargets