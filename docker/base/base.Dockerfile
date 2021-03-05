FROM ubuntu:20.04@sha256:3093096ee188f8ff4531949b8f6115af4747ec1c58858c091c8cb4579c39cc4e

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgomp1 \
        python3.8 \
        python3-distutils \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1 \
    && update-alternatives --config python \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 \
    && update-alternatives --config python3 \
    && rm -rf /var/lib/apt/lists/*
