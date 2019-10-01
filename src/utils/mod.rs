
pub mod kinematics;

pub fn wrap_floor(x:f64, lower_limit:usize, upper_limit:usize) -> usize {
    let upper_f64:f64 = upper_limit as f64;
    let lower_f64:f64 = lower_limit as f64;
    let wrap_range:f64 = upper_f64 - lower_f64;
    
    let mut x_ans:f64 = x;
    while x_ans.floor() < lower_f64 { x_ans += wrap_range; }
    while x_ans.floor() > upper_f64 { x_ans -= wrap_range; }
    x_ans.floor() as usize
}

pub fn bool_slice_to_byte_vec(x:&[bool]) -> Vec<u8> {
    assert!(x.len()%8 == 0);    // TODO: consider adding options for padding later
    let mut ans:Vec<u8> = vec![];
    let mut next_byte:[bool; 8] = [false; 8];
    for (bit_idx, bit) in x.iter().enumerate() {
        next_byte[bit_idx % 8] = *bit;
        if bit_idx % 8 == 7 {
            ans.push(bool_slice_to_u8(&next_byte));
        }
    }
    ans
}

pub fn bool_slice_to_u8(bools:&[bool]) -> u8 {
    let n = bools.len();
    assert!(n <= 8);
    (0..n).filter(|i| bools[*i]).map(|i| 2u8.pow((n-i-1) as u32)).fold(0u8, |acc, x| acc+x)
}

pub fn bool_slice_to_u16(bools:&[bool]) -> u16 {
    let n = bools.len();
    assert!(n <= 16);
    (0..n).filter(|i| bools[*i]).map(|i| 2u16.pow((n-i-1) as u32)).fold(0u16, |acc, x| acc+x)
}

pub fn bool_slice_to_u32(bools:&[bool]) -> u32 {
    let n = bools.len();
    assert!(n <= 32);
    (0..n).filter(|i| bools[*i]).map(|i| 2u32.pow((n-i-1) as u32)).fold(0u32, |acc, x| acc+x)
}

pub fn bool_slice_to_i8(bools:&[bool]) -> i8 {
    let n = bools.len();
    assert!(n <= 8);
    if bools[0] { (1..n).filter(|i| !bools[*i]).map(|i| 2i8.pow((n-i-1) as u32)).fold(0i8, |acc, x| acc+x) * -1i8 }
    else        { (1..n).filter(|i|  bools[*i]).map(|i| 2i8.pow((n-i-1) as u32)).fold(0i8, |acc, x| acc+x)        }
}

pub fn bool_slice_to_i16(bools:&[bool]) -> i16 {
    let n = bools.len();
    assert!(n <= 16);
    if bools[0] { (1..n).filter(|i| !bools[*i]).map(|i| 2i16.pow((n-i-1) as u32)).fold(0i16, |acc, x| acc+x) * -1i16 }
    else        { (1..n).filter(|i|  bools[*i]).map(|i| 2i16.pow((n-i-1) as u32)).fold(0i16, |acc, x| acc+x)        }
}

pub fn bool_slice_to_i32(bools:&[bool]) -> i32 {
    let n = bools.len();
    assert!(n <= 32);
    if bools[0] { (1..n).filter(|i| !bools[*i]).map(|i| 2i32.pow((n-i-1) as u32)).fold(0i32, |acc, x| acc+x) * -1i32 }
    else        { (1..n).filter(|i|  bools[*i]).map(|i| 2i32.pow((n-i-1) as u32)).fold(0i32, |acc, x| acc+x)        }
}

