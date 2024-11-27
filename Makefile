# Simple Makefile for PID_GPIO_Fan_Control_CPP

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11

# Libraries
LIBS = -lwiringPi -lpthread

# Directories
SRCDIR = src
INCDIR = include
OBJDIR = build
BINDIR = build

# Source and object files
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# Target
TARGET = $(BINDIR)/pid_fan_control

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compilation
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -c -o $@ $<

# Clean
clean:
	rm -rf $(OBJDIR)/*.o $(TARGET)

# Run
run:
	sudo ./build/pid_fan_control

# Phony targets
.PHONY: all clean run
