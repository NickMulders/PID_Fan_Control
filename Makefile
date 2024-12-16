# Simple Makefile for PID_GPIO_Fan_Control_CPP

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11

# C Compiler for WiringPi
CC = gcc
CFLAGS = -Wall -Wextra -O2

# Include directories
INCDIRS = include \
          libs/wiringPi/wiringPi \
          libs/wiringPi/devLib \
          libs/wiringPi/gpio \
          libs/wiringPi/wiringPiD
CXXFLAGS += $(addprefix -I, $(INCDIRS))
CFLAGS += $(addprefix -I, $(INCDIRS))

# Libraries
LIBS = -lpthread  # Removed -lwiringPi as we'll link the static library directly

# Directories
SRCDIR = src
OBJDIR = build
BINDIR = build
WIRINGPI_DIR = libs/wiringPi

# WiringPi build variables
WIRINGPI_LIBDIR = $(OBJDIR)/libs
WIRINGPI_LIB = $(WIRINGPI_LIBDIR)/libwiringPi.a

# Collect WiringPi C sources (excluding examples and test)
WIRINGPI_SOURCES = $(shell find $(WIRINGPI_DIR) -type f -name "*.c" \
                           ! -path "*/examples/*" \
                           ! -path "*/test/*")

# Convert .c sources to .o objects in build/libs/
WIRINGPI_OBJECTS = $(WIRINGPI_SOURCES:$(WIRINGPI_DIR)/%.c=$(WIRINGPI_LIBDIR)/%.o)

# Source and object files for main project
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# Target executable
TARGET = $(BINDIR)/pid_fan_control

# Default target
all: $(TARGET)

# WiringPi static library
$(WIRINGPI_LIB): $(WIRINGPI_OBJECTS)
	@echo "Creating WiringPi static library..."
	@mkdir -p $(WIRINGPI_LIBDIR)
	ar rcs $@ $^

# Compile WiringPi object files
$(WIRINGPI_LIBDIR)/%.o: $(WIRINGPI_DIR)/%.c
	@echo "Compiling WiringPi source: $<"
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

# Linking main project
$(TARGET): $(OBJECTS) $(WIRINGPI_LIB)
	@echo "Linking main application..."
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) -L$(WIRINGPI_LIBDIR) -lwiringPi $(LIBS)

# Compile main project object files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@echo "Compiling source: $<"
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(OBJDIR)/*.o $(TARGET) $(WIRINGPI_LIBDIR)/*.o $(WIRINGPI_LIB)

# Run the application
run:
	./build/pid_fan_control

# Phony targets
.PHONY: all clean run
