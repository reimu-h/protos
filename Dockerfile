# Build and test environment for protos (`make docker-build`, `make docker-test`).
# Debian 12 "bookworm" fixes the toolchain major versions: GCC 12, binutils
# 2.40, GRUB 2.06 and QEMU 7.2. Packages still receive Debian security and
# point-release updates.
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/tmp

RUN apt-get update && apt-get install -y --no-install-recommends \
    binutils \
    g++ \
    grub-common \
    grub-efi-amd64-bin \
    grub-pc-bin \
    grub2-common \
    libgmock-dev \
    libgtest-dev \
    make \
    mtools \
    ovmf \
    python3 \
    qemu-system-x86 \
    xorriso \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
