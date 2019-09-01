
extern crate byteorder;
extern crate num_complex;
extern crate rust_radio;

use std::env;
use std::io::{Read, Write};
use std::fs::File;
use byteorder::{ByteOrder, LittleEndian};
use num_complex::Complex;

#[derive(Debug, PartialEq)]
enum SampleType {
	ComplexI16,
	ComplexF32,
}

#[derive(Debug, PartialEq)]
enum DownsampleMethod {
	Average,
	Drop,
}

fn main() {

	let args:Vec<String> = env::args().collect();
	let mut arg_iter = args.iter();
	arg_iter.next();	// throw away the first element, which is just the filename of the binary

	let mut n_downsample:usize = 2;
	let mut source_filename:Option<String> = None;
	let mut target_filename:Option<String> = None;
	let mut input_type:SampleType = SampleType::ComplexI16;
	let mut output_type:SampleType = SampleType::ComplexI16;
	let mut downsample_method:DownsampleMethod = DownsampleMethod::Average;

	while let Some(ref arg) = arg_iter.next() {
		if *arg == "-h" || *arg == "--help" || !arg.starts_with("-") {
			println!("Available command line arguments for dsp_downsample");
			println!("-n or --number      Number of samples to combine into one sample (default=2)");
			println!("-s or --source      Input filename; if not specified, input will come from stdin");
			println!("-o or --output      Output filename; if not specified, output will go to stdout");
			println!("-a or --input_type  Type of input; options are [complex_i16 (default)]");
			println!("-b or --output_type Type of output; options are [complex_i16 (default), complex_f32]");
			println!("-m or --method      Method of downsampling; options are [avg (default), drop]");
			return;
		}
		
		let next_arg = match arg_iter.next() {
			Some(x) => x,
			None => panic!("All arguments except -h or --help must come in pairs"),
		};

		match arg.as_ref() {
			"-n" | "--number"      => match next_arg.parse::<usize>() {
				Ok(n) => n_downsample = n,
				Err(e) => panic!("Unable to parse {:?} as usize ({:?})", next_arg, e),
			},
			"-s" | "--source"      => source_filename = Some(next_arg.to_string()),
			"-o" | "--output"      => target_filename = Some(next_arg.to_string()),
			"-a" | "--input_type"  => input_type = match next_arg.as_ref() {
				"complex_i16" => SampleType::ComplexI16,
				x => panic!("{} isn't a valid sample type", x),
			},
			"-b" | "--output_type" => output_type = match next_arg.as_ref() {
				"complex_i16" => SampleType::ComplexI16,
				"complex_f32" => SampleType::ComplexF32,
				x => panic!("{} isn't a valid sample type", x),
			},
			"-m" | "--method"      => downsample_method = match next_arg.as_ref() {
				"avg" => DownsampleMethod::Average,
				"drop" => DownsampleMethod::Drop,
				x => panic!("{} isn't a valid downsample method", x),
			},
			_                      => panic!("Unrecognized arguments: {:?} {:?}", arg, next_arg),
		}
		
	}

	eprintln!("Downsample {:?}:1 using method {:?}", &n_downsample, &downsample_method);
	eprintln!("{:?} as {:?} -> {:?} as {:?}", &source_filename,  &input_type, &target_filename, &output_type);

	// Done parsing command line arguments.  Now do the actual work
	if !(source_filename.is_some() && target_filename.is_some()) {
		panic!("Only input and output to files are supported right now");
	}
	if input_type != SampleType::ComplexI16 || output_type != SampleType::ComplexI16 {
		panic!("Only complex_i16 input and output are supported right now");
	}
	if downsample_method != DownsampleMethod::Average {
		panic!("Only the averaging downsample method is supported right now");
	}

	let mut f_in = File::open(source_filename.unwrap()).expect("Unable to open source file");
	let mut f_out = File::create(target_filename.unwrap()).expect("Unable to create output file");
	let mut buffer = [0; 4];

	let mut downsample_buffer:Vec<Complex<f32>> = Vec::new();

	let mut bytes_read:usize = 0;
	while let Ok(_) = f_in.read_exact(&mut buffer) {

		bytes_read = bytes_read + 4;

        let re_i16 = LittleEndian::read_i16(&buffer[0..2]);
        let im_i16 = LittleEndian::read_i16(&buffer[2..4]);
        let this_sample = Complex{re: re_i16 as f32, im: im_i16 as f32};
        
        downsample_buffer.push(this_sample);
        if downsample_buffer.len() >= n_downsample {
			let downsample_sum:Complex<f32> = downsample_buffer.iter().fold(Complex{ re: 0.0, im: 0.0 }, |a,b| a+b );
        	let downsample_avg:Complex<f32> = downsample_sum * Complex{ re: (1.0 / (downsample_buffer.len() as f32)), im: 0.0};
        	let re:i16 = downsample_avg.re as i16;
        	let im:i16 = downsample_avg.im as i16;
        	f_out.write_all(&re.to_le_bytes()).expect("Unable to write data");
        	f_out.write_all(&im.to_le_bytes()).expect("Unable to write data");
			downsample_buffer.clear();
        }

	}

	eprintln!("Downsampling complete");
}