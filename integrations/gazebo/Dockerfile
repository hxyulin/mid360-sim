# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg cmake g++ pkg-config && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' > /etc/apt/sources.list.d/gazebo.list && \
    apt-get update && apt-get install -y --no-install-recommends libgz-sim8-dev libgz-physics7-bullet gz-sim8-cli && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends make cppzmq-dev && rm -rf /var/lib/apt/lists/*
