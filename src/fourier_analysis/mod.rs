
extern crate num_complex;
extern crate test;

use self::num_complex::Complex;
use ::types::even_odd_slice::{self, EvenOddSlice};

pub fn fft(x: &Vec<Complex<f64>>) -> Vec<Complex<f64>> { fft_k(even_odd_slice::new(&x), -1.0) }
pub fn ifft(x: &Vec<Complex<f64>>) -> Vec<Complex<f64>> { fft_k(even_odd_slice::new(&x), 1.0).iter().map(|c| c / (x.len() as f64)).collect() }

fn fft_k(mut x:EvenOddSlice<Complex<f64>>, sign:f64) -> Vec<Complex<f64>> {
    let n = x.len();
    if n%2 == 0 {
	    let ek = fft_k(x.even(), sign);
	    let ok = fft_k(x.odd(),  sign);
	    let pairs:Vec<(&Complex<f64>, Complex<f64>)> = (0..(n / 2)).map(|k| {
		    let ex = Complex{re: 0.0, im: (sign * 2.0 * std::f64::consts::PI * (k as f64)) / (n as f64)}.exp();
		    (&ek[k], ex * ok[k])
	    }).collect();

	    let first_half  = pairs.iter().map(|(ek, eok)| *ek + eok);
	    let second_half = pairs.iter().map(|(ek, eok)| *ek - eok);
	    first_half.chain(second_half).collect()
    }
    else if n == 1 { vec![x.next().unwrap().clone()]              }
	else           { panic!("Can only run FFT/IFFT with powers of 2"); }    
}

#[test]
fn test_fft_and_ifft() {
	let x_time_usize:Vec<usize> = (0..8).collect();
	let x_time:Vec<Complex<f64>> = x_time_usize.iter().map(|x| Complex{re: *x as f64, im: 0.0}).collect();
	let x_freq:Vec<Complex<f64>> = fft(&x_time);

	let x_time_usize_p:Vec<usize> = ifft(&x_freq).iter().map(|c| c.re.round() as usize ).collect();

	for (a,b) in x_time_usize.iter().zip(x_time_usize_p.iter()) {
		assert_eq!(a, b);
	}
}

#[bench]
fn bench_fft(b: &mut self::test::Bencher) {

	b.iter(|| {
		let x_time:Vec<Complex<f64>> = (0..131072).map(|x| Complex{re: x as f64, im: 0.0}).collect();	
		fft(&x_time);
	})
	

}