# rust_radio

RustRadio is a digital signal processing library for the Rust programming language.  It's inspired by the GNU Radio and GNSS-SDR open source projects and a desire to bring similar functionality to Rust.  Right now everything's in the rapid prototyping phase and there are not yet any guarantees on stability or backwards compatibility, although I hope to get to that point before long.  There will probably be at least one major reorganization when I add more signals to the GNSS code.  Right now it only decodes the GPS L1 C/A signal and when I add more, I'll want to separate the functionality that's common to all signals and constellations from the functionality that's specific to one.  However, it's hard to do that before I've implemented multiple signals.

The GNSS functionality is the best-developed portion of the project so far but I plan to expand the scope to a broader range of digital signal processing functionality.  The library also currently includes my own FFT module made (almost) from scratch.  It only works with power-of-two window sizes and the performance may not be as good as better-developed libraries, but it's good for learning.  When I need an FFT in the GNSS code, I use the `rustfft` crate instead of my own FFT.

This crate has been most recently built and tested with stable Rust 1.37.0.

# GNSS Quick Start

The library includes several binaries that demo some of the key features.  One of them is called `gps_l1_ca_subframe_decode` and it takes raw IQ samples from a software-defined radio receiver, acquires SVs, tracks them, and decodes navigation messages.  However, it doesn't yet translate the navigation messages into a fix.  It outputs the navigation messages in JSON format.  Not all navigation messages are implemented so far.  To try it, clone the repository.

```git clone https://github.com/johnwstanford/rust_radio.git```

Then build the release version of the crate.

```cargo build --release```

Navigate to the folder with the compiled binaries.

```cd target/release```

Run `gps_l1_ca_subframe_decode` and redirect stdout to a JSON file.

```./gps_l1_ca_subframe_decode --filename ../../data/1562782171_1575.42Mhz_2e6sps.dat --sample_rate_sps 2e6 > 1562782171_1575.42Mhz_2e6sps.json``` 

The binary needs to know the sample rate of the file (2.0 Msps in this case) and expects raw IQ samples packed as 16-bit signed integers (4 bytes per sample).  There's some sample data included in the repo that was collected using an Ettus Research USRP B205mini-i.  As the binary runs, you'll see lines scrolling with information.  Acquisition is shown in green text and red indicates failed acquisition or loss of lock.  When a navigation subframe is successfully decoded, it's shown in blue text and the hexadecimal bytes that make it up are shown in white text.  However, none of this shows up to the final JSON file because it's all output to stderr.  The only thing output the stdout is a final JSON array consisting of all the subframes decoded.