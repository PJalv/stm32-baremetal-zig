const std = @import("std");

// Constants and Struct Definitions
const rcc = extern struct {
    CR: u32,
    PLLCFGR: u32,
    CFGR: u32,
    CIR: u32,
    AHB1RSTR: u32,
    AHB2RSTR: u32,
    AHB3RSTR: u32,
    RESERVED0: u32,
    APB1RSTR: u32,
    APB2RSTR: u32,
    RESERVED1L: u32,
    RESERVED1H: u32,
    AHB1ENR: u32,
    AHB2ENR: u32,
    AHB3ENR: u32,
    RESERVED2: u32,
    APB1ENR: u32,
    APB2ENR: u32,
    RESERVED3L: u32,
    RESERVED3H: u32,
    AHB1LPENR: u32,
    AHB2LPENR: u32,
    AHB3LPENR: u32,
    RESERVED4: u32,
    APB1LPENR: u32,
    APB2LPENR: u32,
    RESERVED5L: u32,
    RESERVED5H: u32,
    BDCR: u32,
    CSR: u32,
    RESERVED6H: u32,
    RESERVEDK: u32,
    SSCGR: u32,
    PLLI2SCFGR: u32,
};

const gpio = extern struct {
    MODER: u32,
    OTYPER: u32,
    OSPEEDR: u32,
    PUPDR: u32,
    IDR: u32,
    ODR: u32,
    BSRR: u32,
    LCKR: u32,
    AFR: [2]u32,
};

const systick = extern struct {
    CTRL: u32,
    LOAD: u32,
    VAL: u32,
    CALIB: u32,
};

const uart = extern struct {
    SR: u32,
    DR: u32,
    BRR: u32,
    CR1: u32,
    CR2: u32,
    CR3: u32,
    GTPR: u32,
};

pub inline fn uart_init(self: *volatile uart, baud: u32) void {
    // https://www.st.com/resource/en/datasheet/stm32f429zi.pdf
    const af: u8 = 7; // Alternate function
    var rx: u16 = 0;
    var tx: u16 = 0; // pins

    if (self == UART1)
        RCC.APB2ENR |= BIT(4);
    if (self == UART2)
        RCC.APB1ENR |= BIT(17);
    if (self == UART3)
        RCC.APB1ENR |= BIT(18);

    if (self == UART1)
        tx = PIN('A', 9);
    rx = PIN('A', 10);
    if (self == UART2)
        tx = PIN('A', 2);
    rx = PIN('A', 3);
    if (self == UART3)
        tx = PIN('D', 8);
    rx = PIN('D', 9);

    gpio_set_mode(tx, GPIO_MODE.AF);
    gpio_set_af(tx, af);
    gpio_set_mode(rx, GPIO_MODE.AF);
    gpio_set_af(rx, af);
    self.CR1 = 0; // Disable this UART
    self.BRR = FREQ / baud; // FREQ is a UART bus frequency
    self.CR1 |= BIT(13) | BIT(2) | BIT(3); // Set UE, RE, TE
}
pub inline fn uart_read_ready(uart_: *volatile uart) u32 {
    return uart_.SR & BIT(5); // If RXNE bit is set, data is ready
}
pub inline fn uart_read_byte(self: *volatile uart) u8 {
    return @as(u8, self.DR & 255);
}

pub inline fn uart_write_byte(uart_: *volatile uart, byte: u8) void {
    uart_.DR = byte;
    while ((uart_.SR & BIT(7)) == 0)
        delay_loop(1);
}
pub inline fn uart_write_buf(uart_: *volatile uart, buf: []const u8) void {
    for (buf) |char| {
        uart_write_byte(uart_, char);
    }
}
const FREQ = 16000000;

const UART1: *volatile uart = @ptrFromInt(0x40011000);
const UART2: *volatile uart = @ptrFromInt(0x40004400);
const UART3: *volatile uart = @ptrFromInt(0x40004800);

const SYSTICK: *volatile systick = @ptrFromInt(0xe000e010);
const GPIOB: *volatile gpio = @ptrFromInt(0x40020400);
const RCC: *volatile rcc = @ptrFromInt(0x40023800);

const GPIO_MODE = enum(u3) { INPUT, OUTPUT, AF, ANALOG };

// Inline Functions
inline fn gpio_set_mode(pin: u16, mode: GPIO_MODE) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const n: u16 = PINNO(pin);
    RCC.AHB1ENR |= BIT(PINBANK(pin));
    const mask: u32 = @as(u32, 3) << @intCast((n * 2));
    const mode_value: u32 = @as(u32, (@intFromEnum(mode) & 3)) << @intCast((n * 2));
    gpio_.MODER &= ~mask;
    gpio_.MODER |= mode_value;
}

inline fn gpio_set_af(pin: u16, af_num: u8) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const n: u16 = PINNO(pin);
    RCC.AHB1ENR |= BIT(PINBANK(pin));
    gpio_.AFR[n >> 3] &= ~(@as(u32, 15) << @intCast(((n & 7) * 4)));
    gpio_.AFR[n >> 3] |= (@as(u32, af_num) << (@intCast((n & 7) * 4)));
}

inline fn gpio_write(pin: u16, val: bool) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const bsr_value: u32 = @as(u32, 1) << @as(u32, PINNO(pin)) << @as(u5, (if (val) 0 else 16));
    gpio_.BSRR = bsr_value;
}

fn delay_loop(iterations: usize) void {
    var count: usize = iterations;
    while (count != 0) : (count -= 1) {
        asm volatile ("nop");
    }
}

var counter: u32 = 0;

fn SysTick_Handler() callconv(.C) void {
    counter += 1;
}
fn timer_expired(t: *u32, prd: u32, now: u32) bool {
    if (now + prd < t.*)
        t.* = 0; // Time wrapped? Reset timer
    if (t.* == 0)
        t.* = now + prd; // First poll? Set expiration
    if (t.* > now)
        return false; // Not expired yet, return
    t.* = if ((now - t.*) > prd) now + prd else t.* + prd; // Next expiration time
    return true; // Expired, return true
}

inline fn systick_init(comptime ticks: u32) void {
    if ((ticks - 1) > 0xffffff)
        return; // Systick timer is 24 bit
    SYSTICK.LOAD = ticks - 1;
    SYSTICK.VAL = 0;
    SYSTICK.CTRL = BIT(0) | BIT(1) | BIT(2); // Enable systick
    RCC.APB2ENR |= BIT(14); // Enable SYSCFG
}

export fn _start() noreturn {
    @call(.auto, main, .{});

    unreachable;
}

inline fn GPIO(bank: u32) *gpio {
    return @ptrFromInt((0x40020000 + 0x400 * (bank)));
}

inline fn PINBANK(pin: u16) u32 {
    const bank = pin >> 8;
    return bank;
}

inline fn PINNO(pin: u16) u16 {
    const number = pin & 255;
    return number;
}

inline fn BIT(x: u32) u32 {
    const bit_value = @as(c_ulong, 1) << @intCast(x);
    return bit_value;
}

inline fn PIN(comptime bank: u32, comptime num: u32) u16 {
    const pin_value = ((bank - 'A') << 8) | num;
    return pin_value;
}

// Main Function
pub fn main() void {
    systick_init(16_000_000 / 1000);
    uart_init(UART3, 115200); // Initialise UART
    const period: u32 = 1000;
    var timer: u32 = 0;
    const led: u16 = PIN('B', 7);
    gpio_set_mode(led, GPIO_MODE.OUTPUT);
    var on: bool = true;

    while (true) {
        if (timer_expired(&timer, period, @as(*volatile u32, @ptrCast(&counter)).*)) {
            gpio_write(led, on);
            on = !on;
            const message: []const u8 = "HELLO FROM ZIG OMG\r\n";
            uart_write_buf(UART3, message);
        }
    }
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
export const vector_table linksection(".vectors") = [_]*const fn () callconv(.C) void{ _estack, _reset, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, SysTick_Handler };
