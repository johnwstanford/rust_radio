
const CRC_24Q_POLYNOMIAL:[bool; 25] = [true, true, false, false, false, false, true, true, false, false, true, false,
	false, true, true, false, false, true, true, true, true, true, false, true, true];

fn main() {

	let mut message_w_crc:Vec<bool> = vec![true,  false,  false,  false,  true,  false,  true,  true,  false,  false,  true,  true,  true,  true,  false,  true,  true,  true,  true,  false,
  		true,  false,  true,  false,  false,  false,  true,  true,  false,  true,  true,  false,  false,  true,  true,  true,  false,  false,  true,  false,  true,  true,  true,
  		true,  false,  false,  true,  false,  true,  true,  true,  false,  true,  true,  false,  false,  true,  true,  true,  true,  true,  true,  false,  true,  false,  false,
  		false,  true,  false,  true,  false,  true,  true,  true,  false,  false,  false,  false,  true,  false,  true,  false,  false,  true,  true,  true,  true,  false,  false,
  		true,  true,  false,  true,  false,  true,  true,  true,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  true,  false,  true,  true,  false,
  		true,  true,  true,  false,  true,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  true,  true,  true,  true,  false,  true,  false,  false,
  		false,  false,  false,  false,  false,  true,  true,  true,  true,  true,  true,  true,  true,  true,  false,  false,  true,  false,  false,  false,  false,  false,  false,
  		true,  true,  false,  true,  false,  true,  true,  true,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,
  		false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  false,  true,  false,  false,  true,  false,  false,
  		false,  false,  false,  false,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  false,  false,  false,
  		true,  false,  true,  false,  true,  false,  false,  false,  false,  false,  false,  true,  false,  false,  true,  true,  true,  true,  true,  true,  true,  false,  true,
  		true,  true,  true,  true,  false,  false,  true,  false,  false,  true,  true,  false,  true,  true,  true,  false,  true,  true,  true,  true,  true,  true,  true,
  		false,  true,  true,  true,  true,  false,  true,  true,  true,  false,  false,  true,  false,  true,  true,  true,  false,  true,  false,  false,  true,  false,  true,  
  		true,  false,  false,  true,  false];

  	for i in 0..(message_w_crc.len() - CRC_24Q_POLYNOMIAL.len() + 1) {
  		if message_w_crc[i] {
  			for j in 0..CRC_24Q_POLYNOMIAL.len() {
  				message_w_crc[i+j] = message_w_crc[i+j] ^ CRC_24Q_POLYNOMIAL[j];
  			}
  		}
  	}

  	for b in message_w_crc {
  		assert!(!b);
  	}

  	println!("CRC Ok");
}