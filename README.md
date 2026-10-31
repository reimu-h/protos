# protos

protos is a small x86-64 SMP kernel in freestanding C++20. It has:

- GRUB boot (Multiboot 1 and 2) on BIOS and UEFI machines;
- multiprocessor support (xAPIC and x2APIC) with a preemptive, work-stealing
  scheduler;
- user-mode ELF programs with system calls, among them a shell and a USB
  keyboard driver (xHCI, with hubs).

## Build and run

Building needs GNU make, a native x86-64 GCC (not Clang or an `x86_64-elf`
cross compiler), binutils, GRUB with its BIOS and UEFI modules, xorriso, mtools
and QEMU; the unit tests also need GoogleTest. The `Dockerfile` lists the Debian
packages, and when a tool is missing, `make` builds in that Docker image
instead.

```sh
make                  # build/kernel.iso (Multiboot 2) and build/kernel-mb1.iso
make run SELFTEST=0   # boot into the shell in QEMU (Ctrl-A X quits)
make run              # boot with the self-tests, then a stress test loop
make test             # unit tests, then QEMU boot tests
```

## Security

User programs are isolated from each other and from the kernel, with two known
exceptions: there is no KPTI or other mitigation for speculative-execution
attacks such as Meltdown, and drivers program DMA without an IOMMU, so a driver
process is effectively as privileged as the kernel.

## License

MIT; see [LICENSE](LICENSE).
