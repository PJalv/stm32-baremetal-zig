const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb, // ARM instruction set
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 }, // cross compile is so easy!
        .os_tag = .freestanding,
    });

    const elf = b.addExecutable(.{
        .name = "firmware-zigg.elf",
        .target = target,
        .root_source_file = b.path("main.zig"),
    });

    elf.setLinkerScript(b.path("link.ld"));

    b.installArtifact(elf);
}
