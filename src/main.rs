#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

use stm32f401_pac as pac;

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
        w.pllq0().set_bit()
        .pllq1().set_bit()
        .pllq2().set_bit()
        .pllq3().clear_bit()
    });

    dp.RCC.cr().write(|w| w.pllon().set_bit());
    while ! dp.RCC.cr().read().pllrdy().bit() {}

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
        .write(|w| w
            .gpioaen().set_bit()
            .gpiocen().set_bit()
        );
}

fn config_usart2(dp: &pac::Peripherals) {
    // Init the clock
    dp.RCC
        .apb1enr()
        .write(|w| w
            .usart2en().set_bit()
        );
    
    dp.GPIOA
        .moder()
        .write(|w| unsafe { w
            .moder2().bits(0b10)
            .moder3().bits(0b10)
        });

    dp.GPIOA
        .otyper()
        .write(|w|  w
            .ot2().clear_bit()
            .ot3().clear_bit()
        );

    dp.GPIOA
        .ospeedr()
        .write(|w| unsafe { w
            .ospeedr2().bits(0b10)
            .ospeedr3().bits(0b10)
        });

    dp.GPIOA
        .pupdr()
        .write(|w| unsafe { w
            .pupdr2().bits(0b00)
            .pupdr3().bits(0b00)
        });

    dp.GPIOA
        .afrl()
        .write(|w| unsafe { w
            .afrl2().bits(0b0111)
            .afrl3().bits(0b0111)
        });

    // Disable USART2
    dp.USART2
        .cr1()
        .write(|w| w.ue().clear_bit());

    // 8N1 / oversampling 16 / transmit enabled
    // Config CR2 Stopbits
    dp.USART2
        .cr2()
        .modify(|_,w| unsafe {
            w.stop().bits(0b00)
        });

    // Config CR1 Wordlength, Parity, Mode, Oversampling
    dp.USART2
        .cr1()
        .modify(|_, w|
            w.m().clear_bit()
            .pce().clear_bit()
            .over8().clear_bit()
            .te().set_bit()
        );

    // Config CR3 disable CTSE, RTSE (hw flowcontrol)
    dp.USART2
        .cr3()
        .modify(|_, w|
            w.rtse().clear_bit()
            .ctse().clear_bit()
        );

    // Determine BRR from PCLK1 frq (84Mhz DIV/2) and baudrate
    let brr = baud_to_brr(84_000_000/2, 115_200);
    dp.USART2
        .brr()
        .write(|w| unsafe { w.bits(brr) });

    // Clear CR2 LINEN, CLKEN
    dp.USART2
        .cr2()
        .modify(|_,w|
            w.linen().clear_bit()
            .clken().clear_bit()
        );

    // Clear CR3 SCEN, HDSEL, IREN
    dp.USART2
        .cr3()
        .modify(|_,w| 
            w.scen().clear_bit()
            .hdsel().clear_bit()
            .iren().clear_bit()
        );

    // Enable USART2
    dp.USART2
        .cr1()
        .modify(|_, w| w.ue().set_bit());
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

    dp.GPIOA.moder().modify(|_, w| unsafe { w.moder5().bits(0b01) });

    let mut ctr = 0;
    let mut idx = 0;
    let message = "Hello World!\r\n";
    loop {
        if cp.SYST.has_wrapped() {
            dp.GPIOA.odr().modify(|r, w| w.odr5().bit(!r.odr5().bit()));
        }

        if ctr == 0 {
            let val = message.as_bytes()[idx];
            idx = (idx + 1) % message.as_bytes().len();
            dp.USART2
                .dr().write(|w| unsafe { w.dr().bits(val as u16)});
        }
        ctr = (ctr + 1) % 100000;

        // Read PC13 Input Value
        // if !dp.GPIOC.idr().read().idr13().bit() {
        //     // If high then set PA5 output to High (Turn on LED)
        //     dp.GPIOA.odr().write(|w| w.odr5().set_bit());
        // } else {
        //     // If low then set PA5 output to Low (Turn off LED)
        //     dp.GPIOA.odr().write(|w| w.odr5().clear_bit());
        // }
    }
}

fn baud_to_brr(fclk :u64, baudrate :u64) -> u32 {
    let div16 = (fclk * 25) / (4 * baudrate);
    let divmant =  div16 / 100;
    let divfraq = (((div16 - (divmant * 100)) * 16) + 50) / 100;
     ((divmant << 4) + (divfraq & 0xF0) + (divfraq & 0x0F)) as u32
}