
use crate::DigSigProcErr;

pub fn to_byte_vec(x:&[bool]) -> Result<Vec<u8>, DigSigProcErr> {
    if x.len()%8 == 0 {
    	// TODO: consider adding options for padding later
	    let mut ans:Vec<u8> = vec![];
	    let mut next_byte:[bool; 8] = [false; 8];
	    for (bit_idx, bit) in x.iter().enumerate() {
	        next_byte[bit_idx % 8] = *bit;
	        if bit_idx % 8 == 7 {
	            ans.push(to_u8(&next_byte)?);
	        }
	    }
	    Ok(ans)
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len()%8 != 0 in bools_to_int::to_byte_vec"))
    }
}

pub fn to_u8(bools:&[bool]) -> Result<u8, DigSigProcErr> {
    let n = bools.len();
    if n <= 8 {
	    Ok((0..n).filter(|i| bools[*i]).map(|i| 2u8.pow((n-i-1) as u32)).fold(0u8, |acc, x| acc+x))
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 8 in bools_to_int::to_u8"))
    }
}

pub fn to_u16(bools:&[bool]) -> Result<u16, DigSigProcErr> {
    let n = bools.len();
    if n <= 16 {
    	Ok((0..n).filter(|i| bools[*i]).map(|i| 2u16.pow((n-i-1) as u32)).fold(0u16, |acc, x| acc+x))	
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 16 in bools_to_int::to_u16"))
    }
}

pub fn to_u32(bools:&[bool]) -> Result<u32, DigSigProcErr> {
    let n = bools.len();
    if n <= 32 {
    	Ok((0..n).filter(|i| bools[*i]).map(|i| 2u32.pow((n-i-1) as u32)).fold(0u32, |acc, x| acc+x))	
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 32 in bools_to_int::to_u32"))
    }
}

pub fn to_u64(bools:&[bool]) -> Result<u64, DigSigProcErr> {
    let n = bools.len();
    if n <= 64 {
    	Ok((0..n).filter(|i| bools[*i]).map(|i| 2u64.pow((n-i-1) as u32)).fold(0u64, |acc, x| acc+x))	
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 64 in bools_to_int::to_u64"))
    }
}

pub fn to_i8(bools:&[bool]) -> Result<i8, DigSigProcErr> {
    let n = bools.len();
    if n <= 8 {
	    if bools[0] { Ok((1..n).filter(|i| !bools[*i]).map(|i| 2i8.pow((n-i-1) as u32)).fold(0i8, |acc, x| acc+x) * -1i8) }
	    else        { Ok((1..n).filter(|i|  bools[*i]).map(|i| 2i8.pow((n-i-1) as u32)).fold(0i8, |acc, x| acc+x))        }
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 8 in bools_to_int::to_i8"))
    }
}

pub fn to_i16(bools:&[bool]) -> Result<i16, DigSigProcErr> {
    let n = bools.len();
    if n <= 16 {
	    if bools[0] { Ok((1..n).filter(|i| !bools[*i]).map(|i| 2i16.pow((n-i-1) as u32)).fold(0i16, |acc, x| acc+x) * -1i16) }
	    else        { Ok((1..n).filter(|i|  bools[*i]).map(|i| 2i16.pow((n-i-1) as u32)).fold(0i16, |acc, x| acc+x))        }
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 16 in bools_to_int::to_i16"))
    }
}

pub fn to_i32(bools:&[bool]) -> Result<i32, DigSigProcErr> {
    let n = bools.len();
    if n <= 32 {
	    if bools[0] { Ok((1..n).filter(|i| !bools[*i]).map(|i| 2i32.pow((n-i-1) as u32)).fold(0i32, |acc, x| acc+x) * -1i32) }
	    else        { Ok((1..n).filter(|i|  bools[*i]).map(|i| 2i32.pow((n-i-1) as u32)).fold(0i32, |acc, x| acc+x))        }
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 32 in bools_to_int::to_i32"))
    }
}

pub fn to_i64(bools:&[bool]) -> Result<i64, DigSigProcErr> {
    let n = bools.len();
    if n <= 64 {
	    if bools[0] { Ok((1..n).filter(|i| !bools[*i]).map(|i| 2i64.pow((n-i-1) as u32)).fold(0i64, |acc, x| acc+x) * -1i64) }
	    else        { Ok((1..n).filter(|i|  bools[*i]).map(|i| 2i64.pow((n-i-1) as u32)).fold(0i64, |acc, x| acc+x))        }
    } else {
    	Err(DigSigProcErr::InvalidTelemetryData("x.len() > 64 in bools_to_int::to_i64"))
    }
}

