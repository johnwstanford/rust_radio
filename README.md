# rust_radio

RustRadio is a digital signal processing library for the Rust programming language.  It's inspired by the GNU Radio and GNSS-SDR open source projects and a desire to bring similar functionality to Rust.  Right now everything's in the rapid prototyping phase and there are not yet any guarantees on stability or backwards compatability, although I hope to get to that point before long.

The GNSS functionality is the best-developed portion of the project so far but I plan to expand the scope to a broader range of digital signal processing functionality.  The library also currently includes my own FFT module made (almost) from scratch.  It only works with power-of-two window sizes and the performance may not be as good as better-developed libraries, but it's good for learning.  When I need an FFT in the GNSS code, I use the rustfft crate instead of my own FFT.

# GNSS Quick Start

The library includes several binaries that demo some of the key features.  One of them is called gps_l1_ca_subframe_decode and it takes raw IQ samples from a software-defined radio unit, acquires SVs, tracks them, and decodes navigation messages.  However, it doesn't yet translate the navigation messages into a fix.  The binary outputs the navigation messages in JSON format.  Most navigation messages are implemented so far, but not all.  To try it, clone the repository.

```git clone https://github.com/johnwstanford/rust_radio.git```

Then build the release version of the crate.

```cargo build --release```

Navigate to the folder with the compiled binaries.

```cd target/release```

Run gps_l1_ca_subframe_decode and redirect stdout to a json file.

```./gps_l1_ca_subframe_decode --filename ../../data/1562782171_1575.42Mhz_2e6sps.dat --sample_rate_sps 2e6 > 1562782171_1575.42Mhz_2e6sps.json``` 

The binary expects needs to know the sample rate of the file (2.0 Msps in this case) and expects raw IQ samples packed as 16-bit signed integers (4 bytes per sample).  There's some sample data included in the repo that was collected using an Ettus Research USRP B205mini-i.  As the binary runs, you'll see lines scrolling with information.  Acquisition is shown in green text and failed acquisition and loss of lock is shown in red.  When a navigation subframe is successfully decoded, it's shown in blue text and the hexadecimal bytes that make it up are shown in white text.  However, none of this shows up to the final JSON file because it's all output to stderr.  The only thing output the stdout is a final JSON array consisting of all the subframes decoded.