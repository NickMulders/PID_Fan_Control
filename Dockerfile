# Stage 1: Build
FROM debian:bullseye AS builder

# Set environment variables for non-interactive builds
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y \
    g++ \
    gcc \
    make \
    libpthread-stubs0-dev \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory inside the container
WORKDIR /usr/src/app

# Copy the entire project into the container
COPY . .

# Run the build
RUN make clean
RUN make all

# Verify build artifacts
RUN ls -l ./build
RUN ls -l ./build/libs

# Stage 2: Runtime
FROM debian:bullseye

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libpthread-stubs0-dev \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /usr/src/app

# Copy the built application and static library from the builder stage
COPY --from=builder /usr/src/app/build/pid_fan_control ./build/pid_fan_control
COPY --from=builder /usr/src/app/build/libs/libwiringPi.a ./build/libs/libwiringPi.a

# Ensure the executable has execute permissions
RUN chmod +x ./build/pid_fan_control

# Define the command to run the application directly
CMD ["./build/pid_fan_control"]
