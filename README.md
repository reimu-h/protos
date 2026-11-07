# protos

protos is a small x86-64 SMP kernel in freestanding C++20. It has:

- GRUB boot (Multiboot 1 and 2) on BIOS and UEFI machines;
- multiprocessor support (xAPIC, x2APIC, and I/O APIC) with a preemptive,
  work-stealing scheduler;
- user-mode ELF programs with system calls, synchronous IPC, PCI MSI/MSI-X and
  legacy ISA device/IRQ ownership, an in-memory filesystem server (`ramfs`) with
  POSIX-like file and directory operations that starts with the files of
  `rootfs/`, a shell (`sh`), a USB keyboard driver (`usbkbd`: xHCI, with hubs),
  and an `i8042` PS/2 keyboard driver (`ps2kbd`).

## Build and run

Building needs GNU make, a native x86-64 GCC (not Clang or an `x86_64-elf`
cross compiler), binutils, GNU tar 1.28 or newer, GRUB with its BIOS and UEFI
modules, xorriso, mtools and QEMU; the boot tests also use OVMF (UEFI firmware)
and `python3`, and the unit tests need GoogleTest. The `Dockerfile` lists the
Debian packages, and when a tool is missing, `make` builds in that Docker image
instead.

```sh
make                  # build/kernel.iso (Multiboot 2) and build/kernel-mb1.iso
make run SELFTEST=0   # boot into the shell in QEMU (Ctrl-A X quits)
make run              # boot with the self-tests, then a stress test loop
make test             # unit tests, then QEMU boot tests
```

CI checks that the C++ sources follow `.clang-format` with clang-format 21.1.8,
since other versions format some code differently; to format with it, run
`pipx run --spec clang-format==21.1.8 clang-format -i <files>`.

## Boot filesystem

The directories and files in `rootfs/`, together with the standalone user
programs in `/bin/` (`ROOTFS_PROGRAMS` in `Makefile`, such as `/bin/hello`),
are what the filesystem holds at boot: the build packs `rootfs/` from the
working tree as it is (untracked files such as editor swap files included; git
itself doesn't track empty directories) into a POSIX ustar archive,
`/boot/rootfs.tar` on the ISO images, which GRUB loads into memory next to the
kernel as a boot module, and `ramfs` creates them when it starts. The shell
runs ELF programs from `/bin/` by name (`hello`) or from any path containing `/`
(`/bin/hello`, `run /path/to/program`). To have a file at boot, add it to
`rootfs/` and rebuild. The build fails for what `ramfs` can't hold (anything
but directories and regular files, with hard links copied as regular files,
names of more than 63 bytes, and files larger than 32 MiB) and for names with
white space, which `sh` splits words on and couldn't name. Paths must also fit
the ustar format: any path of up to 190 bytes below `rootfs/` does, and a
longer one, of up to 254 bytes, only if tar can split `./<path>` (with a `/`
after a directory's name) at a `/` into at most 155 bytes before it and 100
after it (tar fails the build if it can't). The files and directories count
against the limits of `ramfs`: at most 255 of them besides the root, files of at
most 32 MiB, and 64 MiB of memory for everything that it stores, so two 32 MiB
files, for example, can't both be there. Like everything else in `ramfs`,
changes made at run time are lost when the machine restarts.

## Security

Each user program runs in its own virtual address space with isolated CPU and
FPU register state, and CPU exceptions in user mode terminate only the faulting
process. There are no user accounts or file permissions: user processes share
the `ramfs` namespace and the unprivileged IPC ports. They also share finite
system-wide pools of physical memory, kernel tasks, and `ramfs` open handles,
fnodes (file nodes), and blocks. A process has limits of its own (on open
files, unwaited child processes, and mapped memory, for example), but they
don't divide up these pools: a process, or a tree of processes, can use up a
pool for all of them. `ramfs` keeps the memory of the blocks that it frees,
such as those of deleted files, for its own reuse, so the memory that it holds
only grows, up to 64 MiB, until the machine restarts.

Two hardware/privilege boundaries are also not isolated: there is no KPTI or
other mitigation for speculative-execution attacks such as Meltdown, and
user-space drivers (`kUserCapDriver`) program DMA without an IOMMU, control PCI
Bus Master and configuration space, and issue kernel-proxied ISA port
`0x60`/`0x64` I/O, so a driver process is effectively as privileged as the
kernel. That includes the filesystem server `ramfs`, which needs
`kUserCapDriver` to claim the filesystem's IPC port and parses the requests
that the kernel forwards from every process.

## License

MIT; see [LICENSE](LICENSE).
