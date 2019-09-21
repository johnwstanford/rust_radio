extern crate byteorder;

pub mod filters;
pub mod fourier_analysis;
pub mod io;
pub mod gnss;
pub mod types;

pub mod utils {

    use std::io::Cursor;
    use byteorder::{BigEndian, ReadBytesExt};

    pub fn wrap_floor(x:f64, lower_limit:usize, upper_limit:usize) -> usize {
        let upper_f64:f64 = upper_limit as f64;
        let lower_f64:f64 = lower_limit as f64;
        let wrap_range:f64 = upper_f64 - lower_f64;
        
        let mut x_ans:f64 = x;
        while x_ans.floor() < lower_f64 { x_ans += wrap_range; }
        while x_ans.floor() > upper_f64 { x_ans -= wrap_range; }
        x_ans.floor() as usize
    }

    pub fn bool_slice_to_byte_vec(x:&[bool], pad_left:bool) -> Vec<u8> {
        let remainder = x.len() % 8;
        let mut padded_or_cut:Vec<bool> = if pad_left {
            if remainder == 0 { x.iter().map(|b| *b).collect() } 
            else { (0..(8-remainder)).map(|_| false).chain(x.iter().map(|b| *b)).collect() }
        } else { x.iter().skip(remainder).map(|b| *b).collect() };

        let mut ans:Vec<u8> = vec![];
        padded_or_cut.reverse();
        while padded_or_cut.len() > 0 {
            let mut next_byte:[bool; 8] = [false; 8];
            for i in 0..8 { 
                next_byte[i] = padded_or_cut.pop().expect("Ended up with a padded_or_cut:Vec<bool> with a length that's not a multiple of 8"); 
            }
            ans.push(bool_array_to_u8(&next_byte));
        }

        ans
    }

    pub fn bool_slice_to_u32(x:&[bool]) -> u32 {
    	bool_slice_to_type::<u32>(x, 4, |crs| crs.read_u32::<BigEndian>().unwrap())
    }
    pub fn bool_slice_to_i32(x:&[bool]) -> i32 {
    	bool_slice_to_type::<i32>(x, 4, |crs| crs.read_i32::<BigEndian>().unwrap())
    }
    pub fn bool_slice_to_i24(x:&[bool]) -> i32 {
    	assert!(x.len() == 24);
    	let mut abs_value_array:[bool; 24] = [false; 24];
    	for i in 1..24 { abs_value_array[i] = x[i] ^ x[0]; }

    	let abs_value:i32 = bool_slice_to_i32(&abs_value_array);
    	if x[0] { -abs_value } else { abs_value }
    }
    pub fn bool_slice_to_u16(x:&[bool]) -> u16 {
    	bool_slice_to_type::<u16>(x, 2, |crs| crs.read_u16::<BigEndian>().unwrap())
    }
    pub fn bool_slice_to_i16(x:&[bool]) -> i16 { 
    	bool_slice_to_type::<i16>(x, 2, |crs| crs.read_i16::<BigEndian>().unwrap())
    }
    pub fn bool_slice_to_i14(x:&[bool]) -> i16 {
    	assert!(x.len() == 14);
    	let mut abs_value_array:[bool; 14] = [false; 14];
    	for i in 1..14 { abs_value_array[i] = x[i] ^ x[0]; }

    	let abs_value:i16 = bool_slice_to_i16(&abs_value_array);
    	if x[0] { -abs_value } else { abs_value }
    }
    fn bool_slice_to_type<T>(x:&[bool], size:usize, f:fn(&mut Cursor<Vec<u8>>) -> T) -> T {
        let mut as_bytes:Vec<u8> = bool_slice_to_byte_vec(x, true);
        while as_bytes.len() < size { as_bytes.insert(0, 0); }

        let mut crs = Cursor::new(as_bytes);
        f(&mut crs)
    }

    pub fn bool_slice_to_u8(x:&[bool]) -> u8 {
        let mut as_bytes:Vec<u8> = bool_slice_to_byte_vec(x, true);
        as_bytes.pop().unwrap()
    }
    pub fn bool_array_to_u8(x:&[bool; 8]) -> u8 {
        let mut ans:u8 = 0;
        if x[7] { ans += 1;   }
        if x[6] { ans += 2;   }
        if x[5] { ans += 4;   }
        if x[4] { ans += 8;   }
        if x[3] { ans += 16;  }
        if x[2] { ans += 32;  }
        if x[1] { ans += 64;  }
        if x[0] { ans += 128; }
        ans
    }

}

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub enum DigSigProcErr {
    NoSourceData,
    LossOfLock,
    NoAcquisition,
    InvalidTelemetryData,
}
