pub struct RingBuffer {
    data :[u8; 128],
    read_idx : usize,
    write_idx: usize,
    full: bool
}

impl RingBuffer {
    pub fn new() -> RingBuffer {
        Self {
            data: [0; 128],
            read_idx: 0,
            write_idx: 0,
            full: false
        }
    }

    pub fn write(&mut self, data:u8) -> Result<(), &'static str> {
        if self.full {
            return Err("RingBuffer full");
        }

        self.data[self.write_idx] = data;
        self.write_idx = (self.write_idx + 1) % self.data.len();

        self.full = self.write_idx == self.read_idx;

        Ok(())
    }

    pub fn read(&mut self) -> Result<u8, &'static str> {
        if !self.full && self.write_idx == self.read_idx {
            return Err("RingBuffer empty");
        }

        let result = self.data[self.read_idx];
        self.read_idx = (self.read_idx + 1) % self.data.len();
        Ok(result)
    }

    
}