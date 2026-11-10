Welcome to protos.

The files under / come from the rootfs/ directory of the source tree, and
the programs in /bin from user/ (added by the build): the build packs them
into the archive /boot/rootfs.tar on the boot image, GRUB loads the archive
into memory next to the kernel, and the filesystem server ramfs creates its
directories and files when it starts. Like everything in ramfs, they live in
memory only: changes are lost when the machine restarts.

Things to try:
  ls /
  hello
  hello world from protos
  disk
  disk write 0 hello persistent disk
  disk read 0
  disk zero 0
  disk -d 0 info
(disk reads and writes NVMe storage, which persists across SELFTEST=0
restarts. write and zero overwrite the disk's blocks, whatever they hold:
unless they get -f, they refuse a disk whose serial does not start with
"protos", as the disk image of `make run` does, so that they don't destroy
data on a real disk when protos runs on a real machine)
  cat /doc/shell.txt
  stat /README.txt
  write /hello.txt hello world
  cat /hello.txt
