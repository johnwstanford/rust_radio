
extern crate byteorder;

use std::io::Read;
use std::fs::File;

use self::byteorder::{LittleEndian, ReadBytesExt};

pub const BUFFER_SIZE:usize = 2048;

pub struct BufferedFileSource<T: Default + Copy + Sized> {
	f: File, 
	idx:usize, 
	buffer: [T; BUFFER_SIZE],
	buffer_idx: usize,
	buffer_valid_len: usize,
}

impl<T: Default + Copy + Sized> BufferedFileSource<T> {

	pub fn new(filename:&str) -> Result<Self, &'static str> {
		let f   = File::open(filename).map_err(|_| "Unable to open file")?;
		let idx = 0;

		let buffer:[T; BUFFER_SIZE] = [T::default(); BUFFER_SIZE];
		let buffer_idx = 0;
		let buffer_valid_len = 0;

		Ok(Self { f, idx, buffer, buffer_idx, buffer_valid_len })
	}

	unsafe fn buffer_samples(&mut self) -> Result<(), &'static str> {
		let ptr:*mut T = &mut self.buffer[0];
		let ptr_u8:*mut u8 = ptr as *mut _;

		let slice_u8:&mut [u8] = std::slice::from_raw_parts_mut(ptr_u8, std::mem::size_of::<[T; BUFFER_SIZE]>());
		let bytes_read:usize = self.f.read(slice_u8).map_err(|_| "Unable to read from file")?;

		// The number of samples read is the number of bytes read divded by bytes per sample
		self.buffer_valid_len = bytes_read / std::mem::size_of::<T>();
		self.buffer_idx = 0;

		Ok(())
	}

}

impl<T: Default + Copy + Sized> Iterator for BufferedFileSource<T> {
	type Item = (T, usize);

	fn next(&mut self) -> Option<(T, usize)> {
		if self.buffer_idx >= self.buffer_valid_len {
			// If we've run out of buffer, then buffer new samples
			match unsafe { self.buffer_samples() } {
				Ok(()) => {
					if self.buffer_idx >= self.buffer_valid_len {
						// The buffering operation might succeed, but still read zero new samples; if so, return None
						None
					} else {
						// The buffering operation succeeded and returned new samples to read, so we've got something to return
						let ans = (self.buffer[self.buffer_idx], self.idx);
						self.idx += 1;
						self.buffer_idx += 1;
						Some(ans)
					}
				},
				Err(_) => None
			}
		} else {
			// There's no need to buffer new samples; just read the next one and return it
			let ans = (self.buffer[self.buffer_idx], self.idx);
			self.idx += 1;
			self.buffer_idx += 1;
			Some(ans)
		}
	}
}

pub struct FileSourceLEi16Complex {
	f:File,
	idx:usize,
}

pub fn file_source_i16_complex(filename:&str) -> FileSourceLEi16Complex {

	let f_result = File::open(filename);

	FileSourceLEi16Complex{ f: f_result.expect("Unable to open source file"), idx: 0 }
}

impl FileSourceLEi16Complex {

	pub fn drop(&mut self, n:usize) -> () {
		for _ in 0..n {
			self.next();
		}
	}
}

impl Iterator for FileSourceLEi16Complex {
	type Item = ((i16, i16), usize);

	fn next(&mut self) -> Option<((i16, i16), usize)> {
		match (self.f.read_i16::<LittleEndian>(), self.f.read_i16::<LittleEndian>()) {
			(Ok(re_i16), Ok(im_i16)) => {
				let i = self.idx;
				self.idx += 1;
				Some(((re_i16, im_i16), i))
			},
			(_, _) => None,
		}
	}
}