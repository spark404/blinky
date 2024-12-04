#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

mod ringbuffer;
mod support;
mod serial;

use cortex_m_rt::entry;
use stm32f401_pac as pac;
use crate::ringbuffer::RingBuffer;
use crate::serial::{Serial};

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

    // Configure pa2 and pa3 for USART2
    dp.GPIOA
        .moder()
        .write(|w| unsafe { w.moder2().bits(0b10).moder3().bits(0b10) });
    
    dp.GPIOA
        .otyper()
        .write(|w| w.ot2().clear_bit().ot3().clear_bit());
    
    dp.GPIOA
        .ospeedr()
        .write(|w| unsafe { w.ospeedr2().bits(0b10).ospeedr3().bits(0b10) });
    
    dp.GPIOA
        .pupdr()
        .write(|w| unsafe { w.pupdr2().bits(0b00).pupdr3().bits(0b00) });
    
    dp.GPIOA
        .afrl()
        .write(|w| unsafe { w.afrl2().bits(0b0111).afrl3().bits(0b0111) });
    
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
    
    // dbg!("Running..");

    let mut cp = cortex_m::Peripherals::take().unwrap();
    unsafe {
        cp.SYST.rvr.write(8_400_000);
        cp.SYST.cvr.write(8_400_000);
        cp.SYST.csr.write(0x0105)
    };

    dp.GPIOA
        .moder()
        .modify(|_, w| unsafe { w.moder5().bits(0b01) });

    let usart2_config = serial::Config {
        baudrate: 115_200,
    };

    let mut usart2 = Serial::new(dp.USART2, usart2_config);
    let mut buffer = [0; 128];
    buffer[0] = 0x48;
    loop {
        if cp.SYST.has_wrapped() {
            dp.GPIOA.odr().modify(|r, w| w.odr5().bit(!r.odr5().bit()));
            let _ = usart2.write(buffer, 1);
        }
        // 
        // if dp.USART2.sr().read().rxne().bit_is_set() {
        //     let _ = buffer.write(dp.USART2.dr().read().bits() as u8);
        // }
    }
}

