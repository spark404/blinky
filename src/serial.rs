use crate::support::{Ptr, RegisterBlockUsart};
use stm32f401_pac as pac;

pub struct Config {
    pub baudrate: u32,
}
pub struct Serial<USART> {
    uart: USART,
}

pub trait RegisterBlockImpl {
    fn new<USART: Instance + crate::support::Ptr<RB = Self>>(
        uart: USART,
        config: Config,
    ) -> Serial<USART>;
    fn write(&self, byte: u8);
    fn set_transmit_enable(&self, enable: bool);
    fn set_receive_enable(&self, enable: bool);
    fn transmit_complete(&self) -> bool;
}

pub trait Instance:
    crate::support::Ptr<RB: RegisterBlockImpl> + core::ops::Deref<Target = Self::RB>
{
}

impl RegisterBlockImpl for RegisterBlockUsart {
    fn new<USART: Instance + crate::support::Ptr<RB = Self>>(
        uart: USART,
        config: Config,
    ) -> Serial<USART> {
        // Init the clock
        unsafe {
            let rcc = &*pac::Rcc::ptr();
            rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());
        }
        
        // Disable USART2
        uart.cr1().write(|w| w.ue().clear_bit());

        // 8N1 / oversampling 16 / transmit enabled
        // Config CR2 Stopbits
        uart.cr2().modify(|_, w| unsafe { w.stop().bits(0b00) });

        // Config CR1 Wordlength, Parity, Mode, Oversampling
        uart.cr1().modify(|_, w| {
            w.m()
                .clear_bit()
                .pce()
                .clear_bit()
                .over8()
                .clear_bit()
                .re()
                .set_bit()
        });

        // Config CR3 disable CTSE, RTSE (hw flowcontrol)
        uart.cr3()
            .modify(|_, w| w.rtse().clear_bit().ctse().clear_bit());

        // Determine BRR from PCLK1 frq (84Mhz DIV/2) and baudrate
        let brr = baud_to_brr(84_000_000 / 2, config.baudrate);
        uart.brr().write(|w| unsafe { w.bits(brr) });

        // Clear CR2 LINEN, CLKEN
        uart.cr2()
            .modify(|_, w| w.linen().clear_bit().clken().clear_bit());

        // Clear CR3 SCEN, HDSEL, IREN
        uart.cr3()
            .modify(|_, w| w.scen().clear_bit().hdsel().clear_bit().iren().clear_bit());

        // Enable USART2
        uart.cr1().modify(|_, w| w.ue().set_bit());

        Serial { uart }
    }
    
    fn write(&self, byte: u8) {
        self.dr().write(|w| unsafe { w.bits(byte as u32) });
        while self.sr().read().txe().bit_is_clear() {}
    }
    
    fn set_transmit_enable(&self, enable: bool) {
        self.cr1().modify(|_, w|  w.te().bit(enable));
    }
    
    fn set_receive_enable(&self, enable: bool) {
        self.cr1().modify(|_, w| w.re().bit(enable));
    }

    fn transmit_complete(&self) -> bool {
        self.sr().read().tc().bit()
    }
}

impl<USART: Instance> Serial<USART> {
    pub fn new(usart: USART, config: Config) -> Self {
        <USART as Ptr>::RB::new(usart, config)
    }
    
    pub fn write(&mut self, buffer: [u8; 128], len: usize) -> Result<(), &str>{
        self.uart.set_receive_enable(false);
        self.uart.set_transmit_enable(true);

        for idx in 0..len {
            let _ = self.uart.write(buffer[idx]);
        }

        while !self.uart.transmit_complete() {}

        self.uart.set_transmit_enable(false);
        self.uart.set_receive_enable(true);
        
        Ok(())
    }
}

fn baud_to_brr(fclk: u64, baudrate: u32) -> u32 {
    let div16 = (fclk * 25) / (4 * baudrate) as u64;
    let divmant = div16 / 100;
    let divfraq = (((div16 - (divmant * 100)) * 16) + 50) / 100;
    ((divmant << 4) + (divfraq & 0xF0) + (divfraq & 0x0F)) as u32
}
