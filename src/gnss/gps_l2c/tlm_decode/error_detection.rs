
pub const CRC_24Q_POLYNOMIAL:[bool; 25] = [true, true, false, false, false, false, true, true, false, false, true, false,
	false, true, true, false, false, true, true, true, true, true, false, true, true];

pub fn is_subframe_crc_ok(message_w_crc:&[bool]) -> bool {

	if message_w_crc.len() != 300 { return false; }

	// Make a clone of the slice because we'll need to mutate it
	let mut m:[bool; 300] = [true; 300];
	m.clone_from_slice(message_w_crc);

  	for i in 0..(m.len() - CRC_24Q_POLYNOMIAL.len() + 1) {
  		if m[i] {
  			for j in 0..CRC_24Q_POLYNOMIAL.len() {
  				m[i+j] = m[i+j] ^ CRC_24Q_POLYNOMIAL[j];
  			}
  		}
  	}

  	for b in m.iter() {
  		if *b { return false; }
  	}

  	true
}