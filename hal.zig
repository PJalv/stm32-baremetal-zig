pub const FREQ = 16000000;

pub const rcc = extern struct {
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
    RESERVED1: [2]u32,
    AHB1ENR: u32,
    AHB2ENR: u32,
    AHB3ENR: u32,
    RESERVED2: u32,
    APB1ENR: u32,
    APB2ENR: u32,
    RESERVED3: [2]u32,
    AHB1LPENR: u32,
    AHB2LPENR: u32,
    AHB3LPENR: u32,
    RESERVED4: u32,
    APB1LPENR: u32,
    APB2LPENR: u32,
    RESERVED5: [2]u32,
    BDCR: u32,
    CSR: u32,
    RESERVED6H: u32,
    RESERVEDK: u32,
    SSCGR: u32,
    PLLI2SCFGR: u32,
};
pub const RCC: *volatile rcc = @ptrFromInt(0x40023800);

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
pub const GPIO_MODE = enum(u3) { INPUT, OUTPUT, AF, ANALOG };
const GPIOB: *volatile gpio = @ptrFromInt(0x40020400);

pub inline fn gpio_set_mode(pin: u16, comptime mode: GPIO_MODE) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const n: u16 = PINNO(pin);
    RCC.AHB1ENR |= BIT(PINBANK(pin));
    const mask: u32 = @as(u32, 3) << @intCast((n * 2));
    const mode_value: u32 = @as(u32, (@intFromEnum(mode) & 3)) << @intCast((n * 2));
    gpio_.MODER &= ~mask;
    gpio_.MODER |= mode_value;
}

pub inline fn gpio_set_af(pin: u16, comptime af_num: u8) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const n: u16 = PINNO(pin);
    RCC.AHB1ENR |= BIT(PINBANK(pin));
    gpio_.AFR[n >> 3] &= ~(@as(u32, 15) << @intCast(((n & 7) * 4)));
    gpio_.AFR[n >> 3] |= (af_num << (@intCast((n & 7) * 4)));
}

pub inline fn gpio_write(pin: u16, val: bool) void {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const bsr_value: u32 = @as(u32, 1) << @intCast(PINNO(pin)) << @intCast((if (val) 0 else 16));
    gpio_.BSRR = bsr_value;
}
pub inline fn gpio_read(pin: u16) bool {
    const gpio_: *gpio = GPIO(PINBANK(pin));
    const idr_value: u32 = gpio_.IDR; // Read the Input Data Register
    // Get the pin number and mask the corresponding bit
    return ((idr_value & @as(u32, 1) << @intCast(PINNO(pin))) != 0);
}

const systick = extern struct {
    CTRL: u32,
    LOAD: u32,
    VAL: u32,
    CALIB: u32,
};
const SYSTICK: *volatile systick = @ptrFromInt(0xe000e010);

pub fn sleep(iterations: usize) void {
    var i: usize = 0;
    const iptr: *volatile usize = &i;

    while (iptr.* < iterations) {
        iptr.* += 1;
    }
}

pub inline fn systick_init(comptime ticks: u32) void {
    if ((ticks - 1) > 0xffffff)
        return; // Systick timer is 24 bit
    SYSTICK.LOAD = ticks - 1;
    SYSTICK.VAL = 0;
    SYSTICK.CTRL = BIT(0) | BIT(1) | BIT(2); // Enable systick
    RCC.APB2ENR |= BIT(14); // Enable SYSCFG
}

pub fn SysTick_Handler() callconv(.C) void {
    counter += 1;
}

const uart = extern struct {
    SR: u32,
    DR: u32,
    BRR: u32,
    CR1: u32,
    CR2: u32,
    CR3: u32,
    GTPR: u32,
};
pub const UART1: *volatile uart = @ptrFromInt(0x40011000);
pub const UART2: *volatile uart = @ptrFromInt(0x40004400);
pub const UART3: *volatile uart = @ptrFromInt(0x40004800);

pub inline fn uart_init(uart_: *volatile uart, comptime baud: u32) void {
    const af: u8 = 7;
    var rx: u16 = 0;
    var tx: u16 = 0;

    if (uart_ == UART1)
        RCC.APB2ENR |= BIT(4);
    if (uart_ == UART2)
        RCC.APB1ENR |= BIT(17);
    if (uart_ == UART3)
        RCC.APB1ENR |= BIT(18);

    if (uart_ == UART1)
        tx = PIN('A', 9);
    rx = PIN('A', 10);
    if (uart_ == UART2)
        tx = PIN('A', 2);
    rx = PIN('A', 3);
    if (uart_ == UART3)
        tx = PIN('D', 8);
    rx = PIN('D', 9);

    gpio_set_mode(tx, GPIO_MODE.AF);
    gpio_set_af(tx, af);
    gpio_set_mode(rx, GPIO_MODE.AF);
    gpio_set_af(rx, af);
    uart_.CR1 = 0; // Disable this UART
    uart_.BRR = FREQ / baud; // FREQ is a UART bus frequency
    uart_.CR1 |= BIT(13) | BIT(2) | BIT(3); // Set UE, RE, TE
}

pub inline fn uart_read_ready(uart_: *volatile uart) u32 {
    return uart_.SR & BIT(5); // If RXNE bit is set, data is ready
}

pub inline fn uart_read_byte(uart_: *volatile uart) u8 {
    return @as(u8, uart_.DR & 255);
}

pub inline fn uart_write_byte(uart_: *volatile uart, byte: u8) void {
    uart_.DR = byte;
    while ((uart_.SR & BIT(7)) == 0)
        delay_loop(1);
}

pub inline fn uart_write_buf(uart_: *volatile uart, comptime buf: []const u8) void {
    for (buf) |char| {
        uart_write_byte(uart_, char);
    }
}

fn delay_loop(comptime iterations: usize) void {
    var count: usize = iterations;
    while (count != 0) : (count -= 1) {
        asm volatile ("nop");
    }
}

pub var counter: u32 = 0;

pub fn get_counter() u32 {
    return @as(*volatile u32, @ptrCast(&counter)).*;
}
pub fn timer_expired(t: *u32, comptime prd: u32, now: u32) bool {
    if (now + prd < t.*)
        t.* = 0; // Time wrapped? Reset timer
    if (t.* == 0)
        t.* = now + prd; // First poll? Set expiration
    if (t.* > now)
        return false; // Not expired yet, return
    t.* = if ((now - t.*) > prd) now + prd else t.* + prd; // Next expiration time
    return true; // Expired, return true
}

pub inline fn GPIO(bank: u32) *gpio {
    return @ptrFromInt((0x40020000 + 0x400 * (bank)));
}

pub inline fn PINBANK(pin: u16) u32 {
    const bank = pin >> 8;
    return bank;
}

pub inline fn PINNO(pin: u16) u16 {
    const number = pin & 255;
    return number;
}

pub inline fn BIT(x: u32) u32 {
    const bit_value: u32 = @as(u32, 1) << @intCast(x);
    return bit_value;
}

pub inline fn PIN(comptime bank: u32, comptime num: u32) u16 {
    const pin_value = ((bank - 'A') << 8) | num;
    return pin_value;
}
