
extern crate rust_radio;

use rust_radio::gnss::gps_l2c::tlm_decode::error_detection;

fn main() {

	let message_w_crc:[bool; 300] = [true,  false,  false,  false,  true,  false,  true,  true,  false,  false,  true,  true,  true,  true,  false,  true,  true,  true,  true,  false,
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

  	println!("{}", error_detection::is_subframe_crc_ok(&message_w_crc));

}