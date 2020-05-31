
use rust_radio::gnss::gps_l2c::tlm_decode::error_correction;

fn main() {
	
	let symbols:Vec<bool> = vec![true, true, false, true, true, true, false, false, true, true, false, true, false, true, true, true, true, false, true, true, false, false, false, false, false, false, false, true, false, true, true, true, false, true, false, false, false, true, true, false, true, false, false, false, true, false, true, false, false, true, false, true, true, true, true, true, false, false, true, false, false, true, true, true, false, false, true, false, true, false, false, false, false, false, false, true, true, true, true, true, false, true, true, false, false, true, false, true, false, false, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, true, false, false, true, false, true, false, false, true, true, false, true, false, false, false, false, false, false, true, false, true, true, false, true, true, true, true, true, true, false, true, true, false, false, false, true, true, true, true, false, false, true, false, false, true, true, false, false, false, true, false, false, false, false, true, true, true, false, true, true, true, false, true, false, false, false, false, true, true, true, false, true, false, false, false, false, true, false, true, true, false, false, false, false, true, true];

	if let Some(result) = error_correction::decode(symbols) {
	
		// Expect 1010000000110111010000011100001000010010000001111111111111111100010101000000100111111101111100100110

		for s in result {
			print!("{}", if s {1} else {0});
		}
		
		print!("\n");

	}




}