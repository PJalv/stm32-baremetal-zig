const hal = @import("hal.zig");

// Main Function
pub fn main() void {
    hal.systick_init(hal.FREQ / 1000);
    hal.uart_init(hal.UART3, 115200);
    const period: u32 = 1000;
    var timer: u32 = 0;
    const led: u16 = hal.PIN('B', 7);
    const button: u16 = hal.PIN('C', 13);
    hal.gpio_set_mode(led, hal.GPIO_MODE.OUTPUT);
    hal.gpio_set_mode(button, hal.GPIO_MODE.INPUT);

    var on: bool = true;

    while (true) {
        if (hal.timer_expired(&timer, period, hal.get_counter())) {
            hal.gpio_write(led, on);
            on = !on;
            hal.uart_write_buf(hal.UART3, "LIGHTNING MCQUEEN\r\n");
        }
        hal.gpio_write(led, hal.gpio_read(button));
    }
}

export fn _start() noreturn {
    @call(.auto, main, .{});

    unreachable;
}
// Reset Function
extern fn _estack() void;

extern const _data_loadaddr: u32;
extern var _sdata: u32;
extern const _edata: u32;
extern var _sbss: u32;
extern const _ebss: u32;

fn _reset() callconv(.C) noreturn {
    // Copy data from flash to RAM
    const data_loadaddr: [*]const u8 = @ptrCast(&_data_loadaddr);
    const data = @as([*]u8, @ptrCast(&_sdata));
    const data_size = @intFromPtr(&_edata) - @intFromPtr(&_sdata);
    for (data_loadaddr[0..data_size], 0..) |d, i| data[i] = d;
    @memcpy(data[0..data_size], data_loadaddr[0..data_size]);

    // Clear the bss
    const bss: [*]u8 = @ptrCast(&_sbss);
    const bss_size = @intFromPtr(&_ebss) - @intFromPtr(&_sbss);
    @memset(bss[0..bss_size], 0);

    main();
    unreachable;
}
fn _zero() callconv(.C) void {}
// Vector Table
export const vector_table linksection(".vectors") = [_]*const fn () callconv(.C) void{ _estack, _reset, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, hal.SysTick_Handler };
