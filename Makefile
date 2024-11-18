# Makefile for PID_GPIO_Fan_Control_CPP

# Compiler
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11

# Libraries
LIBS = -lwiringPi -lpthread

# Directories
SRCDIR = src
INCDIR = include
OBJDIR = build
BINDIR = build

# Source and Object files
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))

# Executable name
TARGET = $(BINDIR)/pid_fan_control

# Default target
all: $(TARGET)

# Link
$(TARGET): $(OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile
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
.PHONY: all clean
