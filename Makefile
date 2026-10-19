AS := as
CXX := g++
LD := ld
BUILD_DIR := build

# Treat compiler warnings as errors (the default, and what CI enforces). Pass
# WERROR=0 if a newer compiler than the ones CI uses reports new warnings.
WERROR ?= 1
WERROR_FLAG := $(if $(filter 1,$(WERROR)),-Werror)

# Link the self-tests in selftest/ into the kernel and run them from
# kernel_main (RunSelfTests() in selftest/selftest.h): the boot self-tests,
# which end with "[TEST] ALL BOOT SELF-TESTS PASSED", then the multi-CPU stress
# test in a loop. With SELFTEST=0 the kernel initializes and then idles.
# `make run-boot-test`/`make test` require SELFTEST=1.
SELFTEST ?= 1

# With KVM=1, `make run` and the boot tests (test_boot.sh) run QEMU with KVM
# acceleration (-enable-kvm -cpu host), which needs read/write access to
# /dev/kvm. With USE_DOCKER=1 the container gets the host's /dev/kvm.
KVM ?= 0

ASFLAGS := --64 -I $(BUILD_DIR)
# Sources include project headers by their path from the repository root (for
# example "selftest/stress.h"), also from files in subdirectories.
PROJECT_INCLUDE_FLAGS := -iquote .
# The kernel is freestanding but uses a few header-only parts of libstdc++
# (<atomic>, <memory>, <new>, <type_traits>, <utility>). Their freestanding
# subset is too small, so $(FREESTANDING_CXX_CONFIG) below wraps the host
# <bits/c++config.h> and forces _GLIBCXX_HOSTED=1. This needs a native x86-64
# GCC with libstdc++ headers (or the Docker image); bare x86_64-elf cross
# compilers and clang/libc++ are not supported. The runtime support those
# headers need is provided by heap.cpp (operator new/delete) and
# cxx_runtime.cpp (mem* functions, __cxa_* hooks).
# -fstack-clash-protection makes a function whose stack frame is larger than a
# page touch the frame's pages in order, so that it can't step over the
# unmapped guard page below a stack without faulting.
# -mgeneral-regs-only keeps the kernel off the x87/MMX/SSE/AVX registers, which
# hold the values of the interrupted user program until the scheduler saves
# them (fpu.h).
CXXFLAGS := -m64 -masm=intel -std=c++20 -O2 -Wall -Wextra $(WERROR_FLAG) \
            -ffreestanding -nostdlib -fno-builtin \
            -fno-exceptions -fno-rtti -mno-red-zone \
            -mgeneral-regs-only -fno-stack-protector -fno-pie \
            -fstack-clash-protection \
            -DPROTOS_SELFTEST=$(if $(filter 1,$(SELFTEST)),1,0) \
            $(PROJECT_INCLUDE_FLAGS) -I$(BUILD_DIR)/include

INJECT_KILL_DELAY ?=
INJECT_KILL_DELAY_CHECKS := spinner mid_syscall teardown concurrent_killers \
                            busy_killer runaway shutdown
ifneq ($(strip $(INJECT_KILL_DELAY)),)
INJECT_KILL_DELAY_ID := $(strip $(foreach i,1 2 3 4 5 6 7,$(if $(filter \
    $(strip $(INJECT_KILL_DELAY)),$(word $(i),$(INJECT_KILL_DELAY_CHECKS))),$(i))))
ifeq ($(INJECT_KILL_DELAY_ID),)
$(error INJECT_KILL_DELAY must be one of: $(INJECT_KILL_DELAY_CHECKS))
endif
CXXFLAGS += -DPROTOS_INJECT_KILL_DELAY=$(INJECT_KILL_DELAY_ID)
endif

TEST_COMMON_CXXFLAGS := -std=c++20 -Wall -Wextra $(WERROR_FLAG) $(PROJECT_INCLUDE_FLAGS) -pthread -MMD -MP
TEST_COMMON_LDFLAGS := -lgmock_main -lgmock -lgtest -pthread

# User programs (user/): freestanding static executables for user space, which
# the kernel image embeds (user_programs.S). user/user.ld links them in the top
# 2 GiB of the address space, which the kernel code model addresses. Unlike the
# kernel, they may use the x87/MMX/SSE registers, and the AVX registers in
# inline assembly, which the kernel keeps for each process (user_mode.h), so
# GCC may use SSE in them. GCC must not turn their loops into calls of the
# memory functions that user/crt.cpp defines with such loops.
USER_PROGRAMS := init usertest
USER_CXXFLAGS := -m64 -masm=intel -std=c++20 -O2 -Wall -Wextra $(WERROR_FLAG) \
                 -ffreestanding -nostdlib -fno-builtin \
                 -fno-exceptions -fno-rtti -mno-red-zone \
                 -fno-stack-protector -fno-pie \
                 -mcmodel=kernel -fno-asynchronous-unwind-tables \
                 -fno-tree-loop-distribute-patterns \
                 $(PROJECT_INCLUDE_FLAGS) -I$(BUILD_DIR)/include
USER_LDFLAGS := -m elf_x86_64 -z max-page-size=0x1000 -z noexecstack \
                -static -nostdlib -T user/user.ld
USER_HDRS := syscall_abi.h user/syscall.h user/usertest.h
USER_ELFS := $(patsubst %,$(BUILD_DIR)/user/%.elf,$(USER_PROGRAMS))

TEST_BUILD_MODES := debug opt
TEST_BUILD_FLAGS_debug := -O0 -g3 -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS
TEST_BUILD_FLAGS_opt := -O2 -g -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS

TEST_SAN_MODES := nosan asan ubsan lsan tsan
TEST_SAN_FLAGS_nosan :=
TEST_SAN_FLAGS_asan := -fsanitize=address,pointer-compare,pointer-subtract \
                       -fno-sanitize-recover=all
TEST_SAN_FLAGS_ubsan := -fsanitize=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow \
                        -fno-sanitize-recover=all
TEST_SAN_FLAGS_lsan := -fsanitize=leak -fno-sanitize-recover=all
TEST_SAN_FLAGS_tsan := -fsanitize=thread -fno-sanitize-recover=all

TEST_SAN_ENV_nosan :=
TEST_SAN_ENV_asan := ASAN_OPTIONS=strict_string_checks=1:detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:detect_invalid_pointer_pairs=2:detect_leaks=1
TEST_SAN_ENV_ubsan := UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1
TEST_SAN_ENV_lsan := LSAN_OPTIONS=print_suppressions=0
TEST_SAN_ENV_tsan := TSAN_OPTIONS=halt_on_error=1

# Unit tests: every *_test.cpp file in the root and in selftest/. A test is
# named after its file without the directory (selftest/stress_test.cpp is
# stress_test), so test file names must be unique.
HOST_TEST_SRCS := $(sort $(wildcard *_test.cpp selftest/*_test.cpp))
HOST_TESTS := $(notdir $(basename $(HOST_TEST_SRCS)))
ifneq ($(words $(HOST_TESTS)),$(words $(sort $(HOST_TESTS))))
$(error Unit test file names must be unique: $(HOST_TEST_SRCS))
endif
$(foreach src,$(HOST_TEST_SRCS),$(eval HOST_TEST_SRC_$(notdir $(basename $(src))) := $(src)))
TASK_LAYER_SRCS := \
    core_runqueue_scheduler.cpp \
    work_stealing_scheduler.cpp \
    join_lifecycle_scheduler.cpp \
    preemptive_scheduler.cpp

TASK_LAYER_HDRS := \
    forwarding_task_scheduler.h \
    task_scheduler_internals.h \
    core_runqueue_scheduler.h \
    work_stealing_scheduler.h \
    join_lifecycle_scheduler.h \
    preemptive_scheduler.h

# Host tests that link idt.cpp also need the stand-ins for its kernel-only
# symbols (boot.S's ISR stub table, the CR2 register), and gdt.cpp, which sets
# the kernel stack for interrupts from user mode.
IDT_TEST_SRCS := idt.cpp idt_host_fakes.cpp gdt.cpp

TEST_EXTRA_SRCS_console_test := console.cpp
TEST_EXTRA_SRCS_cpu_test := cpu.cpp
TEST_EXTRA_SRCS_cxx_runtime_test := cxx_runtime.cpp
TEST_EXTRA_SRCS_elf_test := elf.cpp
TEST_EXTRA_SRCS_fpu_test := fpu.cpp
TEST_EXTRA_SRCS_gdt_test := gdt.cpp
TEST_EXTRA_SRCS_heap_test := heap.cpp
TEST_EXTRA_SRCS_idt_test := $(IDT_TEST_SRCS)
TEST_EXTRA_SRCS_multiboot_test := multiboot.cpp console.cpp
TEST_EXTRA_SRCS_pmm_test := pmm.cpp console.cpp
TEST_EXTRA_SRCS_random_test := random.cpp
TEST_EXTRA_SRCS_selftest_test := selftest/selftest.cpp
# ApKernelEntry() (smp.cpp) calls CpuInitAp(), whose host build counts calls.
TEST_EXTRA_SRCS_smp_test := smp.cpp smp_hardware_fake.cpp multiboot.cpp console.cpp cpu.cpp $(IDT_TEST_SRCS)
TEST_EXTRA_SRCS_task_stack_test := task_stack.cpp
# The user-copy helpers read the SMAP flag that cpu.cpp defines.
TEST_EXTRA_SRCS_user_access_test := user_access.cpp cpu.cpp
TEST_EXTRA_SRCS_vga_test := vga.cpp
# Host fakes for the scheduler tests (see scheduler_test_support.h). Tests that
# link the real idt.cpp and task.cpp must not link the IDT/task fakes. Tests
# that link core_runqueue_scheduler.cpp also link its context switch
# (context_switch.S).
SCHED_TEST_FAKES := scheduler_test_fakes.cpp
SCHED_TEST_IDT_TASK_FAKES := scheduler_test_idt_task_fakes.cpp
TEST_EXTRA_SRCS_core_runqueue_scheduler_test := core_runqueue_scheduler.cpp context_switch.S $(SCHED_TEST_FAKES) $(SCHED_TEST_IDT_TASK_FAKES)
TEST_EXTRA_SRCS_work_stealing_scheduler_test := work_stealing_scheduler.cpp core_runqueue_scheduler.cpp context_switch.S $(SCHED_TEST_FAKES) $(SCHED_TEST_IDT_TASK_FAKES)
TEST_EXTRA_SRCS_join_lifecycle_scheduler_test := join_lifecycle_scheduler.cpp $(SCHED_TEST_FAKES) $(SCHED_TEST_IDT_TASK_FAKES)
TEST_EXTRA_SRCS_preemptive_scheduler_test := preemptive_scheduler.cpp $(SCHED_TEST_FAKES) $(SCHED_TEST_IDT_TASK_FAKES)
TEST_EXTRA_SRCS_task_test := $(TASK_LAYER_SRCS) context_switch.S task.cpp $(IDT_TEST_SRCS) $(SCHED_TEST_FAKES)
TEST_EXTRA_SRCS_stress_test := selftest/stress.cpp random.cpp $(TASK_LAYER_SRCS) context_switch.S task.cpp $(IDT_TEST_SRCS) $(SCHED_TEST_FAKES)

SELFTEST_SRCS := selftest/selftest.cpp selftest/boot_selftest.cpp selftest/stress.cpp

CXX_SRCS := \
    uart.cpp \
    vga.cpp \
    console.cpp \
    idt.cpp \
    gdt.cpp \
    cpu.cpp \
    fpu.cpp \
    multiboot.cpp \
    paging.cpp \
    pmm.cpp \
    heap.cpp \
    cxx_runtime.cpp \
    smp.cpp \
    smp_hardware.cpp \
    $(TASK_LAYER_SRCS) \
    task_stack.cpp \
    address_space.cpp \
    task.cpp \
    elf.cpp \
    user_mode.cpp \
    user_access.cpp \
    random.cpp \
    $(if $(filter 1,$(SELFTEST)),$(SELFTEST_SRCS)) \
    kernel.cpp

CXX_HDRS := \
    check.h \
    format.h \
    interrupts.h \
    spinlock.h \
    uart.h \
    vga.h \
    console.h \
    idt.h \
    gdt.h \
    cpu.h \
    fpu.h \
    multiboot.h \
    paging.h \
    pmm.h \
    heap.h \
    cxx_runtime.h \
    smp.h \
    smp_hardware.h \
    steal_time.h \
    kill_delay_injection.h \
    cpu_clock.h \
    task.h \
    task_stack.h \
    address_space.h \
    $(TASK_LAYER_HDRS) \
    rbtree.h \
    random.h \
    elf.h \
    syscall_abi.h \
    user_mode.h \
    user_access.h \
    user_programs.h \
    user/usertest.h \
    selftest/selftest.h \
    selftest/boot_selftest.h \
    selftest/fpu_lifecycle_layout.h \
    selftest/line_buffer.h \
    selftest/stress.h

ASM_SRCS := boot.S context_switch.S user_mode_entry.S user_programs.S fault_probe.S
ASM_OBJS := $(patsubst %.S,$(BUILD_DIR)/%.o,$(ASM_SRCS))
CXX_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CXX_SRCS))
OBJS := $(ASM_OBJS) $(CXX_OBJS)
FREESTANDING_CXX_CONFIG := $(BUILD_DIR)/include/bits/c++config.h
# Flake injection for checking the deflaking tool; never set by `make test` or
# `make run-boot-test`. INJECT_FLAKY_TEST=<n> makes the boot self-test
# rbtree_edge_cases fail in about one boot in n, decided by a hardware entropy
# draw that the kernel logs. Build such a kernel in a BUILD_DIR of its own:
#   make USE_DOCKER=0 BUILD_DIR=build/inject-flaky INJECT_FLAKY_TEST=20 \
#       build/inject-flaky/kernel.iso
# (Before KERNEL_CONFIG, which must record the define.)
INJECT_FLAKY_TEST ?= 0
ifneq ($(filter-out 0,$(INJECT_FLAKY_TEST)),)
CXXFLAGS += -DPROTOS_INJECT_FLAKY_TEST=$(INJECT_FLAKY_TEST)
endif
# Records the kernel build configuration so that changing flags (e.g. WERROR or
# SELFTEST) rebuilds the kernel.
KERNEL_CONFIG_STAMP := $(BUILD_DIR)/.kernel-config
KERNEL_CONFIG := $(CXX) $(CXXFLAGS) | $(AS) $(ASFLAGS) | $(OBJS) | $(USER_CXXFLAGS) | $(USER_LDFLAGS)

KERNEL_ISO := $(BUILD_DIR)/kernel.iso
KERNEL_MB1_ISO := $(BUILD_DIR)/kernel-mb1.iso

DOCKER ?= docker
DOCKER_IMAGE ?= x86-64-kernel-build
DOCKER_PLATFORM ?= linux/amd64
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
DOCKER_STAMP := $(BUILD_DIR)/.docker.stamp

NPROC := $(shell nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
# --output-sync (-O) needs GNU make 4.0 or newer.
OUTPUT_SYNC := $(if $(filter-out 0.% 1.% 2.% 3.%,$(MAKE_VERSION)),-Otarget)

# Decide whether to build on the host or in the Docker image. Only the tools the
# requested goals need are checked: unit tests need GoogleTest but no GRUB,
# kernel builds need GRUB/xorriso/mtools but no GoogleTest, and boot tests also
# need QEMU. Set USE_DOCKER=0/1 to override.
IN_DOCKER := $(wildcard /.dockerenv /run/.containerenv)
HOST_GRUB_MKRESCUE := $(firstword $(foreach bin,grub-mkrescue grub2-mkrescue,$(if $(shell command -v $(bin) 2>/dev/null),$(bin))))
missing_bins = $(strip $(foreach bin,$(1),$(if $(shell command -v $(bin) 2>/dev/null),,$(bin))))
HAVE_HOST_GTEST := $(and $(wildcard /usr/include/gtest/gtest.h),$(wildcard /usr/include/gmock/gmock.h))

GOALS := $(or $(MAKECMDGOALS),all)
UNIT_TEST_ONLY_GOALS := unit-test test-host test-% $(BUILD_DIR)/obj/% \
    $(foreach b,$(TEST_BUILD_MODES),$(foreach s,$(TEST_SAN_MODES),run-%-$(b)-$(s) $(BUILD_DIR)/%_$(b)_$(s)))
NEED_GTEST := $(filter test $(UNIT_TEST_ONLY_GOALS),$(GOALS))
NEED_QEMU := $(filter test run run-boot-test,$(GOALS))
NEED_KERNEL_TOOLS := $(filter-out clean docker-build docker-test $(UNIT_TEST_ONLY_GOALS),$(GOALS))

ifneq ($(IN_DOCKER),)
USE_DOCKER := 0
else
MISSING_HOST_DEPS := $(call missing_bins,$(CXX) \
    $(if $(NEED_KERNEL_TOOLS),$(AS) $(LD) xorriso mformat) \
    $(if $(NEED_QEMU),qemu-system-x86_64)) \
    $(if $(NEED_KERNEL_TOOLS),$(if $(HOST_GRUB_MKRESCUE),,grub-mkrescue)) \
    $(if $(NEED_GTEST),$(if $(HAVE_HOST_GTEST),,gtest/gmock-headers))
ifeq ($(strip $(MISSING_HOST_DEPS)),)
USE_DOCKER ?= 0
else
ifeq ($(origin USE_DOCKER),undefined)
$(info Building in Docker; missing on the host: $(strip $(MISSING_HOST_DEPS)). Set USE_DOCKER=0 to force a host build.)
endif
USE_DOCKER ?= 1
endif
endif

ifeq ($(USE_DOCKER),1)
GRUB_MKRESCUE := grub-mkrescue
# The Docker image ships binutils 2.40.
LD_NO_WARN_RWX := --no-warn-rwx-segments
else
GRUB_MKRESCUE := $(or $(HOST_GRUB_MKRESCUE),grub-mkrescue)
# --no-warn-rwx-segments exists since binutils 2.39.
LD_NO_WARN_RWX := $(if $(shell $(LD) --help 2>/dev/null | grep -e --no-warn-rwx-segments),--no-warn-rwx-segments)
endif
LDFLAGS := -m elf_x86_64 -z max-page-size=0x1000 $(LD_NO_WARN_RWX) -T linker.ld

# Settings forwarded to the make that runs inside the Docker container
# (including -k, so that `make -k ... USE_DOCKER=1` keeps going there too).
INNER_MAKE := make --no-print-directory WERROR=$(WERROR) SELFTEST=$(SELFTEST) \
              $(if $(findstring k,$(firstword -$(MAKEFLAGS))),-k)
ifneq ($(strip $(INJECT_KILL_DELAY)),)
INNER_MAKE += INJECT_KILL_DELAY=$(strip $(INJECT_KILL_DELAY))
endif
# Boot-test settings forwarded into the container (see test_boot.sh).
BOOT_TEST_ENV_VARS := BIOS_TIMEOUT_SECS UEFI_TIMEOUT_SECS MIN_STRESS_PASSES OVMF OVMF_VARS REQUIRE_UEFI BOOT_TEST_JOBS

ifeq ($(USE_DOCKER),1)
# With KVM=1 the container gets the host's /dev/kvm, membership in the group
# that owns the device (so that the container user may open it), and KVM=1 for
# test_boot.sh.
DOCKER_KVM_ARGS := $(if $(filter 1,$(KVM)),--device /dev/kvm -e KVM=1 \
                   $(if $(wildcard /dev/kvm),--group-add $(shell stat -c %g /dev/kvm)))
DOCKER_RUN_ARGS := --rm --platform=$(DOCKER_PLATFORM) \
                   --user $(HOST_UID):$(HOST_GID) \
                   -e HOME=/tmp $(foreach var,$(BOOT_TEST_ENV_VARS),-e $(var)) \
                   $(DOCKER_KVM_ARGS) \
                   -v "$(CURDIR):/workspace" \
                   -w /workspace
DOCKER_RUN := $(DOCKER) run $(DOCKER_RUN_ARGS) $(DOCKER_IMAGE)
DOCKER_RUN_INTERACTIVE := $(DOCKER) run -it $(DOCKER_RUN_ARGS) $(DOCKER_IMAGE)
DOCKER_DEPS := $(DOCKER_STAMP)
else
DOCKER_RUN :=
DOCKER_RUN_INTERACTIVE :=
DOCKER_DEPS :=
# Host builds run in parallel by default, unless the command line has a -j.
# Sub-makes use the job slots of the top-level make: GNU make before 4.4
# doesn't show -j in MAKEFLAGS while it reads the makefile, so the filter alone
# would make every sub-make start its own -j$(NPROC) jobserver.
ifeq ($(MAKELEVEL),0)
ifeq ($(filter -j% --jobserver%,$(MAKEFLAGS)),)
MAKEFLAGS += -j$(NPROC) $(OUTPUT_SYNC)
endif
endif
endif

# The make in the Docker container builds the same flake-injected kernel.
ifneq ($(filter-out 0,$(INJECT_FLAKY_TEST)),)
INNER_MAKE += INJECT_FLAKY_TEST=$(INJECT_FLAKY_TEST)
endif

QEMU_RUN_FLAGS ?= -smp 4 -m 256M -serial mon:stdio
QEMU_KVM_FLAGS := $(if $(filter 1,$(KVM)),-enable-kvm -cpu host)

.PHONY: all clean test unit-test test-host docker-build docker-test run run-boot-test FORCE test-rbtree test-spinlock test-heap test-idt test-gdt test-pmm test-smp test-core-runqueue-scheduler test-work-stealing-scheduler test-join-lifecycle-scheduler test-preemptive-scheduler test-task test-task-stack test-stress

ifeq ($(USE_DOCKER),1)
all: $(DOCKER_STAMP)
	$(DOCKER_RUN) sh -c '$(INNER_MAKE) -j$$(nproc) -Otarget $(KERNEL_ISO) $(KERNEL_MB1_ISO)'
else
all: $(KERNEL_ISO) $(KERNEL_MB1_ISO)
endif

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(DOCKER_STAMP): Dockerfile | $(BUILD_DIR)
	$(DOCKER) build --platform=$(DOCKER_PLATFORM) -t $(DOCKER_IMAGE) .
	mkdir -p $(BUILD_DIR)/include
	$(DOCKER_RUN) cp -r /usr/include/gmock /usr/include/gtest $(BUILD_DIR)/include/
	touch $(DOCKER_STAMP)

$(FREESTANDING_CXX_CONFIG): Makefile | $(BUILD_DIR)
	mkdir -p $(dir $@)
	printf '#ifndef PROTOS_BITS_CXXCONFIG_H_\n#define PROTOS_BITS_CXXCONFIG_H_\n#pragma GCC system_header\n#include_next <bits/c++config.h>\n#undef _GLIBCXX_HOSTED\n#define _GLIBCXX_HOSTED 1\n#endif\n' > $@

FORCE:

$(KERNEL_CONFIG_STAMP): FORCE | $(BUILD_DIR)
	@echo '$(KERNEL_CONFIG)' | cmp -s - $@ || echo '$(KERNEL_CONFIG)' > $@

$(BUILD_DIR)/%.o: %.S $(KERNEL_CONFIG_STAMP) $(DOCKER_DEPS) | $(BUILD_DIR)
	$(DOCKER_RUN) $(AS) $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp $(CXX_HDRS) $(FREESTANDING_CXX_CONFIG) $(KERNEL_CONFIG_STAMP) $(DOCKER_DEPS) | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(DOCKER_RUN) $(CXX) $(CXXFLAGS) -c $< -o $@

# User programs. These rules have shorter stems than the kernel's, so make
# prefers them for $(BUILD_DIR)/user/.
$(BUILD_DIR)/user/%.o: user/%.cpp $(USER_HDRS) $(FREESTANDING_CXX_CONFIG) $(KERNEL_CONFIG_STAMP) $(DOCKER_DEPS) | $(BUILD_DIR)
	@mkdir -p $(@D)
	$(DOCKER_RUN) $(CXX) $(USER_CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/user/%.elf: $(BUILD_DIR)/user/%.o $(BUILD_DIR)/user/crt.o user/user.ld $(KERNEL_CONFIG_STAMP) $(DOCKER_DEPS)
	$(DOCKER_RUN) $(LD) $(USER_LDFLAGS) $(BUILD_DIR)/user/crt.o $< -o $@

# Keep the user program objects, which make would otherwise delete as
# intermediate files.
.SECONDARY: $(patsubst %,$(BUILD_DIR)/user/%.o,$(USER_PROGRAMS))

$(BUILD_DIR)/user_programs.o: $(USER_ELFS)

$(BUILD_DIR)/kernel.bin: $(OBJS) linker.ld $(KERNEL_CONFIG_STAMP) $(DOCKER_DEPS) | $(BUILD_DIR)
	$(DOCKER_RUN) $(LD) $(LDFLAGS) $(OBJS) -o $@

# $(call MAKE_ISO,iso_path,multiboot_command)
# Builds a GRUB rescue ISO (bootable via legacy BIOS and UEFI) that loads
# kernel.bin with the given GRUB multiboot command (multiboot2 or multiboot).
define MAKE_ISO
	rm -rf $(1).dir
	mkdir -p $(1).dir/boot/grub
	cp $(BUILD_DIR)/kernel.bin $(1).dir/boot/kernel.bin
	printf 'set timeout=0\nset default=0\n\nmenuentry "x86-64 Kernel" {\n    if [ "$${grub_platform}" = "efi" ]; then\n        insmod all_video\n    fi\n    $(2) /boot/kernel.bin\n    if [ "$${grub_platform}" = "pc" ]; then\n        set gfxpayload=text\n    fi\n    boot\n}\n' > $(1).dir/boot/grub/grub.cfg
	$(DOCKER_RUN) $(GRUB_MKRESCUE) -o $(1) $(1).dir
	rm -rf $(1).dir
endef

$(KERNEL_ISO): $(BUILD_DIR)/kernel.bin Makefile $(DOCKER_DEPS) | $(BUILD_DIR)
	$(call MAKE_ISO,$@,multiboot2)

$(KERNEL_MB1_ISO): $(BUILD_DIR)/kernel.bin Makefile $(DOCKER_DEPS) | $(BUILD_DIR)
	$(call MAKE_ISO,$@,multiboot)

# Unit-test objects. Each source of a unit test is compiled on its own into
# $(BUILD_DIR)/obj/<mode>_<san>/<source path without extension>.o, with a .d
# file next to it that lists the headers the source includes. The test
# binaries of one variant (build mode and sanitizer mode) share the objects of
# their common sources. The .config stamp of a variant records its compile and
# link commands, so that changing them (e.g. WERROR) rebuilds the variant.
HOST_TEST_OBJ_DIR := $(BUILD_DIR)/obj

# $(call host_test_objs,test_name,variant)
host_test_objs = $(addprefix $(HOST_TEST_OBJ_DIR)/$(2)/,$(addsuffix .o,$(basename $(HOST_TEST_SRC_$(1)) $(TEST_EXTRA_SRCS_$(1)))))

# $(call DEFINE_HOST_TEST_OBJECTS,build_mode,san_mode)
define DEFINE_HOST_TEST_OBJECTS
TEST_CXXFLAGS_$(1)_$(2) := $$(TEST_COMMON_CXXFLAGS) $$(TEST_BUILD_FLAGS_$(1)) $$(TEST_SAN_FLAGS_$(2))
TEST_LDFLAGS_$(1)_$(2) := $$(TEST_SAN_FLAGS_$(2)) $$(TEST_COMMON_LDFLAGS)
TEST_CONFIG_$(1)_$(2) := $$(CXX) $$(TEST_CXXFLAGS_$(1)_$(2)) | $$(TEST_LDFLAGS_$(1)_$(2))

$(HOST_TEST_OBJ_DIR)/$(1)_$(2)/.config: FORCE
	@mkdir -p $$(@D)
	@echo '$$(TEST_CONFIG_$(1)_$(2))' | cmp -s - $$@ || echo '$$(TEST_CONFIG_$(1)_$(2))' > $$@

$(HOST_TEST_OBJ_DIR)/$(1)_$(2)/%.o: %.cpp $(HOST_TEST_OBJ_DIR)/$(1)_$(2)/.config $$(DOCKER_DEPS)
	@mkdir -p $$(@D)
	$$(DOCKER_RUN) $$(CXX) $$(TEST_CXXFLAGS_$(1)_$(2)) -c $$< -o $$@

$(HOST_TEST_OBJ_DIR)/$(1)_$(2)/%.o: %.S $(HOST_TEST_OBJ_DIR)/$(1)_$(2)/.config $$(DOCKER_DEPS)
	@mkdir -p $$(@D)
	$$(DOCKER_RUN) $$(CXX) $$(TEST_CXXFLAGS_$(1)_$(2)) -c $$< -o $$@
endef

$(foreach b,$(TEST_BUILD_MODES),$(foreach s,$(TEST_SAN_MODES),$(eval $(call DEFINE_HOST_TEST_OBJECTS,$(b),$(s)))))

# $(call DEFINE_HOST_TEST_VARIANT,test_name,build_mode,san_mode)
define DEFINE_HOST_TEST_VARIANT
HOST_TEST_OBJS_$(1)_$(2)_$(3) := $$(call host_test_objs,$(1),$(2)_$(3))

$(BUILD_DIR)/$(1)_$(2)_$(3): $$(HOST_TEST_OBJS_$(1)_$(2)_$(3)) $(HOST_TEST_OBJ_DIR)/$(2)_$(3)/.config $$(DOCKER_DEPS) | $(BUILD_DIR)
	$$(DOCKER_RUN) $$(CXX) $$(HOST_TEST_OBJS_$(1)_$(2)_$(3)) $$(TEST_LDFLAGS_$(2)_$(3)) -o $$@

.PHONY: run-$(1)-$(2)-$(3)
run-$(1)-$(2)-$(3): $(BUILD_DIR)/$(1)_$(2)_$(3)
	$$(DOCKER_RUN) env $$(TEST_SAN_ENV_$(3)) ./$(BUILD_DIR)/$(1)_$(2)_$(3)

HOST_TEST_BINS_$(1) += $(BUILD_DIR)/$(1)_$(2)_$(3)
HOST_TEST_RUNS_$(1) += run-$(1)-$(2)-$(3)
HOST_TEST_ALL_BINS += $(BUILD_DIR)/$(1)_$(2)_$(3)
HOST_TEST_ALL_RUNS += run-$(1)-$(2)-$(3)
HOST_TEST_ALL_OBJS += $$(HOST_TEST_OBJS_$(1)_$(2)_$(3))
endef

define DEFINE_HOST_TEST_SUITE
$(foreach b,$(TEST_BUILD_MODES),$(foreach s,$(TEST_SAN_MODES),$(eval $(call DEFINE_HOST_TEST_VARIANT,$(1),$(b),$(s)))))

.PHONY: test-$(1)
ifeq ($(USE_DOCKER),1)
test-$(1): $(DOCKER_STAMP)
	$$(DOCKER_RUN) sh -c '$$(INNER_MAKE) -j$$$$(nproc) -Otarget $$(HOST_TEST_RUNS_$(1))'
else
test-$(1): $$(HOST_TEST_RUNS_$(1))
endif
endef

$(foreach t,$(HOST_TESTS),$(eval $(call DEFINE_HOST_TEST_SUITE,$(t))))

# The unit tests of the deflaking tool (deflake.py), a python3 script like the
# QEMU monitor helpers in test_boot.sh. They run with the C++ unit tests (make
# test, make unit-test, and in Docker with the image's python3); -B keeps
# python3 from writing __pycache__ into the source tree.
.PHONY: run-deflake_test
run-deflake_test: $(DOCKER_DEPS)
	$(DOCKER_RUN) python3 -B deflake_test.py
HOST_TEST_ALL_RUNS += run-deflake_test

test-rbtree: test-rbtree_test
test-spinlock: test-spinlock_test
test-heap: test-heap_test
test-idt: test-idt_test
test-gdt: test-gdt_test
test-pmm: test-pmm_test
test-smp: test-smp_test
test-core-runqueue-scheduler: test-core_runqueue_scheduler_test
test-work-stealing-scheduler: test-work_stealing_scheduler_test
test-join-lifecycle-scheduler: test-join_lifecycle_scheduler_test
test-preemptive-scheduler: test-preemptive_scheduler_test
test-task: test-task_test
test-task-stack: test-task_stack_test
test-stress: test-stress_test

ifeq ($(USE_DOCKER),1)
unit-test: $(DOCKER_STAMP)
	$(DOCKER_RUN) sh -c '$(INNER_MAKE) -j$$(nproc) -Otarget $(HOST_TEST_ALL_RUNS)'
else
unit-test: $(HOST_TEST_ALL_RUNS)
endif

test-host: unit-test

run-boot-test: $(KERNEL_ISO) $(KERNEL_MB1_ISO) $(DOCKER_DEPS)
	$(if $(filter 1,$(SELFTEST)),,$(error run-boot-test needs the self-tests; do not set SELFTEST=0))
	$(DOCKER_RUN) ./test_boot.sh $(KERNEL_ISO) $(KERNEL_MB1_ISO)

# Boots the kernel interactively; the serial console is attached to the
# terminal (press Ctrl-A X to quit QEMU). KVM=1 enables KVM acceleration.
run: $(KERNEL_ISO) $(DOCKER_DEPS)
	$(DOCKER_RUN_INTERACTIVE) qemu-system-x86_64 $(QEMU_RUN_FLAGS) $(QEMU_KVM_FLAGS) \
		$(if $(filter 1,$(USE_DOCKER)),-display none) -cdrom $(KERNEL_ISO)

# The QEMU boot test is timing-sensitive, so it runs only after the unit tests
# have finished instead of competing with them for CPUs.
ifeq ($(USE_DOCKER),1)
test: $(DOCKER_STAMP)
	$(DOCKER_RUN) sh -c '$(INNER_MAKE) -j$$(nproc) -Otarget $(HOST_TEST_ALL_RUNS) $(KERNEL_ISO) $(KERNEL_MB1_ISO) && $(INNER_MAKE) run-boot-test'
else
test:
	$(MAKE) --no-print-directory $(HOST_TEST_ALL_RUNS) $(KERNEL_ISO) $(KERNEL_MB1_ISO)
	$(MAKE) --no-print-directory run-boot-test
endif

docker-build:
	$(MAKE) all USE_DOCKER=1

docker-test:
	$(MAKE) test USE_DOCKER=1

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(sort $(HOST_TEST_ALL_OBJS:.o=.d)))
