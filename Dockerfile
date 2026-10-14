# Build and test environment for protos (`make docker-build`, `make docker-test`).
# Debian 12 "bookworm" fixes the toolchain major versions: GCC 12, binutils
# 2.40, GRUB 2.06 and QEMU 7.2. The base image is pinned by digest; the
# packages below come from the bookworm archive at build time, so they still
# receive Debian security and point-release updates.
#
# To update the base image, replace the digest with the image index digest (the
# "Digest:" line) printed by
#   docker buildx imagetools inspect debian:bookworm-slim
FROM debian:bookworm-slim@sha256:7c7b2c966bc9ee8cedfeef67e0e279108992c77681fa595db4a9d65c06ccc587

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
