FROM python:3.10-slim

# Install dependencies from apt
RUN apt update && apt install -y --no-install-recommends \
    git \
    wget \
    build-essential

WORKDIR /workspace

RUN GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/TendTo/Formula-Student-Driverless-Simulator.git --depth 1

WORKDIR /workspace/Formula-Student-Driverless-Simulator

RUN pip install .

ENTRYPOINT [ "fsds-ros" ]
