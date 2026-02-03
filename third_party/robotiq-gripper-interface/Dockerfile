FROM ubuntu:20.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install G++, CMake, Boost, and GTest
RUN apt-get clean \
    && apt-get update \
    && apt-get -y install --no-install-recommends \
        g++ \
        cmake \
        make \
        libboost-all-dev \
        libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy package in
COPY . /robotiq-gripper-interface
