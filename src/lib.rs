use std::io::Read;
use std::time::Duration;

mod port_buffer;
use port_buffer::PortBuffer;

pub mod detect;

pub const DIR_ROUND: u16 = 36000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Point {
    pub len: u16,
    pub dir: u16,
}

pub struct LD19 {
    port: Box<dyn serialport::SerialPort>,
    buffer: PortBuffer,
}

impl LD19 {
    pub fn open(path: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(path, 230400)
            .timeout(Duration::from_millis(200))
            .open()?;
        Ok(Self {
            port,
            buffer: Default::default(),
        })
    }

    pub fn min_confidence_mut(&mut self) -> &mut u8 {
        &mut self.buffer.min_confidence
    }

    /// Read from the serial port and parse any available points.
    /// Returns an iterator over newly decoded points.
    pub fn poll(&mut self) -> impl Iterator<Item = Point> + '_ {
        let buf = self.buffer.as_buf();
        match self.port.read(buf) {
            Ok(n) if n > 0 => self.buffer.notify_received(n),
            _ => {}
        }
        &mut self.buffer
    }
}
