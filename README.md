# protos

protos is a small x86-64 SMP kernel in freestanding C++20. It has:

- GRUB boot (Multiboot 1 and 2) on BIOS and UEFI machines;
- multiprocessor support (xAPIC, x2APIC, and I/O APIC) with a preemptive,
  work-stealing scheduler;
- user-mode ELF programs with system calls, PCI MSI/MSI-X and legacy ISA
  device/IRQ ownership, a shell, a USB keyboard driver (`usbkbd`: xHCI, with
  hubs), and an `i8042` PS/2 keyboard driver (`ps2kbd`).

## Build and run

Building needs GNU make, a native x86-64 GCC (not Clang or an `x86_64-elf`
cross compiler), binutils, GRUB with its BIOS and UEFI modules, xorriso, mtools
and QEMU; the boot tests also use OVMF (UEFI firmware) and `python3`, and the
unit tests need GoogleTest. The `Dockerfile` lists the Debian packages, and when
a tool is missing, `make` builds in that Docker image instead.

```sh
make                  # build/kernel.iso (Multiboot 2) and build/kernel-mb1.iso
make run SELFTEST=0   # boot into the shell in QEMU (Ctrl-A X quits)
make run              # boot with the self-tests, then a stress test loop
make test             # unit tests, then QEMU boot tests
```

CI checks that the C++ sources follow `.clang-format` with clang-format 21.1.8,
since other versions format some code differently; to format with it, run
`pipx run --spec clang-format==21.1.8 clang-format -i <files>`.

## Security

User programs are isolated from each other and from the kernel, with two known
exceptions: there is no KPTI or other mitigation for speculative-execution
attacks such as Meltdown, and user-space drivers (`kUserCapDriver`) program DMA
without an IOMMU, control PCI Bus Master and configuration space, and issue
kernel-proxied ISA port `0x60`/`0x64` I/O, so a driver process is effectively as
privileged as the kernel.

## License

MIT; see [LICENSE](LICENSE).
