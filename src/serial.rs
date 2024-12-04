use crate::support::{Ptr, RegisterBlockImpl, RegisterBlockUsart};

pub struct Config {
    pub baudrate: u32,
}
pub struct Serial {
    uart: RegisterBlockUsart
}

use stm32f401_pac as pac;

impl RegisterBlockImpl for RegisterBlockUsart {}

pub trait Instance:
    crate::support::Ptr<RB: RegisterBlockImpl>
    + core::ops::Deref<Target = Self::RB> {}

impl Serial {
    pub fn new<UART: Instance + crate::support::Ptr<RB = Self>>(uart: UART, dp: &pac::Peripherals, config: Config) -> Self {
        // Init the clock
        unsafe {
            let rcc = &*pac::Rcc::ptr();
            rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());
        }

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
        
        // Disable USART2
        uart.cr1().write(|w| w.ue().clear_bit());

        // 8N1 / oversampling 16 / transmit enabled
        // Config CR2 Stopbits
        uart
            .cr2()
            .modify(|_, w| unsafe { w.stop().bits(0b00) });

        // Config CR1 Wordlength, Parity, Mode, Oversampling
        uart
            .cr1()
            .modify(|_, w| w
                .m().clear_bit()
                .pce().clear_bit()
                .over8().clear_bit()
                .re().set_bit());

        // Config CR3 disable CTSE, RTSE (hw flowcontrol)
        uart
            .cr3()
            .modify(|_, w| w
                .rtse().clear_bit()
                .ctse().clear_bit());

        // Determine BRR from PCLK1 frq (84Mhz DIV/2) and baudrate
        let brr = baud_to_brr(84_000_000 / 2, config.baudrate);
        uart
            .brr()
            .write(|w| unsafe { w.bits(brr) });

        // Clear CR2 LINEN, CLKEN
        uart
            .cr2()
            .modify(|_, w| w
                .linen().clear_bit()
                .clken().clear_bit());

        // Clear CR3 SCEN, HDSEL, IREN
        uart
            .cr3()
            .modify(|_, w| w
                .scen().clear_bit()
                .hdsel().clear_bit()
                .iren().clear_bit());

        // Enable USART2
        uart
            .cr1()
            .modify(|_, w| w.ue().set_bit());

        Serial{
            uart
        }
    }

}

fn baud_to_brr(fclk: u64, baudrate: u64) -> u32 {
    let div16 = (fclk * 25) / (4 * baudrate);
    let divmant = div16 / 100;
    let divfraq = (((div16 - (divmant * 100)) * 16) + 50) / 100;
    ((divmant << 4) + (divfraq & 0xF0) + (divfraq & 0x0F)) as u32
}
