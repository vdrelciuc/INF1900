################################################################################
##### Main Makefile responsible for building everything in the subdirectories
#####
##### Note: this Makefile calls other Makefiles in subdirectories
################################################################################

################################################################################
##### Compilation settings and variables
################################################################################

# Build transmitter robot program if tx=1
ifeq ($(tx),1)
	EXEC_DIR = transmitter
else
	EXEC_DIR = receiver
endif

################################################################################
##### Targets
################################################################################

.PHONY: all
all: lib program

# Clean program build directories and rebuild library
.PHONY: lib
lib:
	@echo "Library:"
	@$(MAKE) -s -C lib
	@echo "-----"

# Build robot program
.PHONY: program
program:
	@echo "Program:"
	@$(MAKE) -s -C $(EXEC_DIR)

# Install program on microcontroller
.PHONY: install
install: all
	@echo "Installing:"
	@$(MAKE) -s -C $(EXEC_DIR) install
    ifeq ($(debug),1) # Automatically start USART read if in debug
		@echo "Reading from USART:"
		@serieViaUSB -l
    endif

# Clean build files
.PHONY: clean
clean:
	@echo -n "Receiver: "
	@$(MAKE) -s -C receiver clean
	@echo -n "Transmitter: "
	@$(MAKE) -s -C transmitter clean
	@echo -n "Library: "
	@$(MAKE) -s -C lib clean

# Make documentation
.PHONY: doc
doc:
	doxygen doc/Doxyfile