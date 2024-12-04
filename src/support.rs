use stm32f401_pac::Usart1;
use stm32f401_pac::Usart2;
use stm32f401_pac::Usart6;

pub(crate) use crate::pac::usart1::RegisterBlock as RegisterBlockUsart;
use crate::serial::Instance;

pub trait Ptr {
    /// RegisterBlock structure
    type RB;
    /// Return the pointer to the register block
    fn ptr() -> *const Self::RB;
}

impl Ptr for Usart1 {
    type RB = RegisterBlockUsart;
    fn ptr() -> *const Self::RB {
        Self::ptr()
    }
}
impl Ptr for Usart2 {
    type RB = RegisterBlockUsart;
    fn ptr() -> *const Self::RB {
        Self::ptr()
    }
}
impl Ptr for Usart6 {
    type RB = RegisterBlockUsart;
    fn ptr() -> *const Self::RB {
        Self::ptr()
    }
}

impl Instance for Usart1 {}
impl Instance for Usart2 {}
impl Instance for Usart6 {}