#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

mod ringbuffer;
mod serial;
mod support;

use cortex_m_rt::entry;

use stm32f401_pac as pac;
use crate::ringbuffer::RingBuffer;
use crate::serial::{Config, Serial};

fn systemclock_config(dp: &pac::Peripherals) {
    // SystemClock_Config
    // From CubeMX
    // HSI 16Mhz
    // HSE 8Mhz
    // PllSource = HSI M=16 N=336 P=4 Q=7
    // SysclockSrc = PLL
    // AHB Pre 1 -> HCLK 84Mhz
    // APB1 /2
    // APB2 /1

    // __HAL_RCC_PWR_CLK_ENABLE
    dp.RCC.apb1enr().write(|w| w.pwren().set_bit());
    let _pwren = dp.RCC.apb1enr().read().pwren();

    // __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2)
    // dp.PWR.cr().write(|w| unsafe { w.vos().bits(0b10) });
    // while !dp.PWR.csr().read().vosrdy().bit() {}

    dp.RCC.cr().write(|w| w.hsion().set_bit());
    while !dp.RCC.cr().read().hsirdy().bit() {}

    dp.RCC.cr().modify(|_, m| unsafe { m.hsitrim().bits(0b10) });

    dp.RCC.cr().write(|w| w.pllon().clear_bit());
    while dp.RCC.cr().read().pllrdy().bit() {}

    dp.RCC.pllcfgr().write(|w| {
        w.pllsrc().clear_bit();

        // PLLM = 16
        w.pllm0().clear_bit();
        w.pllm1().clear_bit();
        w.pllm2().clear_bit();
        w.pllm3().clear_bit();
        w.pllm4().set_bit();
        w.pllm5().clear_bit();

        // PLLN = 336
        w.plln0().clear_bit();
        w.plln1().clear_bit();
        w.plln2().clear_bit();
        w.plln3().clear_bit();
        w.plln4().set_bit();
        w.plln5().clear_bit();
        w.plln6().set_bit();
        w.plln7().clear_bit();
        w.plln8().set_bit();

        // PLLP = 4
        w.pllp0().set_bit();
        w.pllp1().clear_bit();

        // PLLQ = x7
        w.pllq0()
            .set_bit()
            .pllq1()
            .set_bit()
            .pllq2()
            .set_bit()
            .pllq3()
            .clear_bit()
    });

    dp.RCC.cr().write(|w| w.pllon().set_bit());
    while !dp.RCC.cr().read().pllrdy().bit() {}

    // FLASH_LATENCY_2
    dp.FLASH.acr().write(|w| unsafe { w.latency().bits(0b10) });

    dp.RCC
        .cfgr()
        .modify(|_, w| unsafe { w.ppre1().bits(0b111) });

    dp.RCC
        .cfgr()
        .modify(|_, w| unsafe { w.ppre2().bits(0b111) });

    dp.RCC
        .cfgr()
        .modify(|_, w| unsafe { w.hpre().bits(0b0000) });

    // SYSCLK source is PLLCLK
    dp.RCC.cfgr().modify(|_, w| {
        w.sw0().clear_bit();
        w.sw1().set_bit()
    });

    dp.RCC
        .cfgr()
        .modify(|_, w| unsafe { w.ppre1().bits(0b100) });

    dp.RCC
        .cfgr()
        .modify(|_, w| unsafe { w.ppre2().bits(0b000) });
}

fn config_gpio(dp: &pac::Peripherals) {
    // Enable Clock to GPIOA & GPIOC
    dp.RCC
        .ahb1enr()
        .write(|w| w.gpioaen().set_bit().gpiocen().set_bit());
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    // Setup handler for device peripherals
    let dp = pac::Peripherals::take().unwrap();

    systemclock_config(&dp);

    config_gpio(&dp);

    config_usart2(&dp);

    let mut cp = cortex_m::Peripherals::take().unwrap();
    unsafe {
        cp.SYST.rvr.write(8_400_000);
        cp.SYST.cvr.write(8_400_000);
        cp.SYST.csr.write(0x0105)
    };

    dp.GPIOA
        .moder()
        .modify(|_, w| unsafe { w.moder5().bits(0b01) });

    let usart2_config = crate::serial::Config {
        baudrate: 115_200,
    };

    let usart2 = Serial::new(dp.USART2, usart2_config);

    let mut buffer = RingBuffer::new();
    loop {
        if cp.SYST.has_wrapped() {
            dp.GPIOA.odr().modify(|r, w| w.odr5().bit(!r.odr5().bit()));

            dp.USART2.cr1().modify(|_, w| w.re().clear_bit().te().set_bit());
            while let Ok(v) = buffer.read() {
                dp.USART2.dr().write(|w| unsafe { w.bits(v as u32) });
                while dp.USART2.sr().read().txe().bit_is_clear() {}
            }
            while dp.USART2.sr().read().tc().bit_is_clear() {}
            dp.USART2.cr1().modify(|_, w| w.re().set_bit().te().clear_bit());
        }

        if dp.USART2.sr().read().rxne().bit_is_set() {
            let _ = buffer.write(dp.USART2.dr().read().bits() as u8);
        }
    }
}

