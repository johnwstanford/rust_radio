
extern crate byteorder;

use std::fs::File;

use self::byteorder::{LittleEndian, ReadBytesExt};

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