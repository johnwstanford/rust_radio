# Global Navigation Satellite System

Although RustRadio is intended to include broader signal-processing functionality in the future, it currently consists almost exclusively of GNSS-related code.  This, in turn, is intended to eventually cover multiple GNSS systems, but currently only covers the American GPS system.  The most completely-supported signal on the GPS system is currently the L1 C/A signal, but acquisition currently also works for the L2 signal.

This chapter describes the GNSS algorithms from a first-principles perspective as much as possible.  It's not meant to be documentation for the library and should be mostly agnostic to the fact that the algorithms are implemented in Rust, though this won't be strictly adhered to and implementation detailed may be mentioned where they make sense.  

The GNSS code is broadly organized into acquisition, tracking, telemetry decoding, and PVT (position, velocity, and time).  The acquisition algorithms search for signals and produce a preliminary lock consisting of a doppler frequency shift and code phase.  This preliminary lock is passed to the tracking algorithm where it is refined to the point where it can be used to decode the actual navigation signals.  The data from the tracking algorithm is passed to the telemetry decoding algorithm where the raw bits are interpreted.  This data, along with information on when exactly it arrived, is passed to PVT, which generates an actual position and time fix.