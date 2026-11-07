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
  cat /doc/shell.txt
  stat /README.txt
  write /hello.txt hello world
  cat /hello.txt
