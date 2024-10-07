const builtin = @import("builtin");
const std = @import("std");

pub fn build(b: *std.Build) void {
    // Target STM32F407VG
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .os_tag = .freestanding,
    });
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.

    const elf = b.addExecutable(.{
        .name = "firmware-zigg.elf",
        .target = target,
        .root_source_file = b.path("main.zig"),
        // .link_libc = true,
    });

    elf.setLinkerScript(b.path("link.ld"));

    // const flash_cmd = b.addSystemCommand(&[_][]const u8{
    //     "openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg", "-c",
    //     b.fmt(
    //         \\"program {s} verify reset exit"
    //     , .{elf.name}),
    // });

    b.installArtifact(elf);
    // flash_cmd.step.dependOn(&elf.step);
    // var flash_step = b.step("flash", "Flash and run the app on your STM32F4Discovery");
    // b.default_step.dependOn(flash_step);
    // flash_step.dependOn(&flash_cmd.step);
}
