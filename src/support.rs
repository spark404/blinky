pub trait Ptr {
    /// RegisterBlock structure
    type RB;
    /// Return the pointer to the register block
    fn ptr() -> *const Self::RB;
}