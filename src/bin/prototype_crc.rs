
const CRC_24Q_POLYNOMIAL:[bool; 25] = [true, true, false, false, false, false, true, true, false, false, true, false,
	false, true, true, false, false, true, true, true, true, true, false, true, true];

fn main() {

	let subframe:Vec<bool> = vec![true,  false,  false,  false,  true,  false,  true,  true,  false,  false,  true,  true,  true,  true,  false,  true,  true,  true,  true,  false,
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

  	println!("polynomial length {:?}", CRC_24Q_POLYNOMIAL.len());
  	println!("subframe length: {:?}", subframe.len());

}