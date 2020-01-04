
// Given in IS-GPS-200K, Table 3-IIa
const CM_INITIAL_STATE:[[bool; 27]; 32] = [
	[true,  true,  true,  true,  false, false, false, true,  false, true,  false, false, false, false, true,  true,  true,  true,  true,  true,  false, true,  true,  false, true,  false, false ],	// PRN 01
	[true,  true,  true,  true,  false, true,  true,  true,  false, false, false, false, false, false, true,  true,  false, false, false, false, false, false, true,  true,  true,  false, true  ],	// PRN 02
	[false, false, false, false, false, false, false, true,  false, true,  true,  true,  true,  false, false, true,  true,  true,  false, false, true,  true,  false, false, true,  false, false ],	// PRN 03
	[false, false, false, true,  true,  false, true,  true,  false, false, true,  false, true,  true,  false, true,  false, true,  true,  true,  true,  false, true,  false, true,  false, false ],	// PRN 04
	[true,  true,  false, false, false, false, false, false, true,  true,  false, false, false, false, false, false, true,  true,  true,  false, false, true,  true,  true,  false, false, true  ],	// PRN 05
	[true,  true,  true,  false, false, false, false, true,  true,  false, true,  false, false, true,  true,  false, true,  false, true,  true,  true,  false, true,  true,  false, true,  true  ],	// PRN 06
	[false, false, true,  false, true,  false, true,  false, false, true,  false, true,  false, false, true,  false, false, false, false, false, false, true,  true,  true,  false, false, false ],	// PRN 07
	[true,  true,  false, false, false, true,  true,  true,  true,  false, true,  true,  false, false, true,  true,  true,  false, false, true,  true,  true,  true,  false, false, false, true  ],	// PRN 08
	[false, false, false, true,  false, false, true,  true,  true,  true,  false, true,  true,  false, false, false, false, true,  true,  true,  false, false, true,  false, false, false, true  ],	// PRN 09
	[true,  true,  true,  false, true,  true,  false, true,  true,  false, false, false, false, true,  true,  false, false, true,  false, false, false, true,  false, false, true,  true,  false ],	// PRN 10
	[true,  true,  true,  false, false, true,  false, true,  true,  true,  false, true,  false, false, true,  false, true,  false, false, false, true,  true,  false, false, true,  false, true  ],	// PRN 11
	[false, false, false, false, true,  false, true,  false, false, true,  false, false, false, true,  true,  true,  true,  true,  true,  true,  false, false, false, false, true,  true,  false ],	// PRN 12
	[false, false, false, false, true,  false, false, false, true,  false, true,  false, true,  true,  false, true,  false, false, false, false, false, false, false, false, false, true,  true  ],	// PRN 13
	[false, true,  false, false, true,  true,  false, false, false, true,  true,  false, true,  false, true,  true,  false, true,  false, true,  true,  true,  false, true,  false, false, true  ],	// PRN 14
	[false, false, false, false, false, false, false, false, true,  false, true,  true,  false, false, true,  true,  false, false, true,  false, false, false, false, false, false, false, false ],	// PRN 15
	[false, true,  false, false, true,  false, false, true,  false, false, false, false, false, true,  false, false, false, true,  true,  false, true,  false, false, false, true,  true,  false ],	// PRN 16
	[true,  false, true,  true,  false, false, false, false, false, false, true,  false, true,  true,  false, true,  false, false, false, false, false, false, true,  false, true,  true,  false ],	// PRN 17
	[false, true,  false, false, false, false, true,  false, true,  true,  false, true,  false, true,  false, false, false, true,  true,  true,  true,  false, false, false, true,  false, true  ],	// PRN 18
	[false, false, false, true,  true,  false, true,  false, false, false, false, false, false, true,  false, false, true,  false, false, false, true,  true,  false, false, true,  false, false ],	// PRN 19
	[false, false, true,  false, true,  false, false, false, false, false, false, true,  true,  true,  false, false, false, true,  false, true,  false, true,  true,  true,  true,  false, false ],	// PRN 20
	[false, false, false, true,  false, false, true,  false, false, false, false, false, false, true,  false, false, true,  true,  true,  false, true,  false, true,  true,  false, true,  true  ],	// PRN 21
	[true,  true,  true,  false, true,  false, true,  false, false, true,  true,  true,  true,  false, false, true,  false, false, false, true,  true,  false, true,  false, true,  true,  true  ],	// PRN 22
	[false, false, false, true,  false, false, true,  false, true,  true,  true,  true,  true,  false, false, false, true,  true,  true,  false, true,  true,  true,  true,  true,  true,  true  ],	// PRN 23
	[true,  true,  true,  true,  false, false, false, false, true,  false, true,  false, false, false, false, false, false, true,  true,  true,  false, true,  true,  false, false, false, false ],	// PRN 24
	[true,  true,  true,  false, false, false, false, false, false, false, true,  false, true,  true,  true,  true,  false, false, false, false, true,  false, true,  true,  true,  false, false ],	// PRN 25
	[false, false, false, false, false, true,  false, false, false, false, true,  false, true,  false, false, true,  true,  true,  false, true,  false, true,  true,  false, false, false, true  ],	// PRN 26
	[true,  true,  true,  false, false, true,  false, true,  true,  true,  false, false, false, true,  true,  false, true,  true,  true,  false, false, true,  false, false, true,  false, true  ],	// PRN 27
	[true,  true,  true,  false, true,  true,  true,  true,  true,  false, true,  true,  false, true,  false, true,  false, false, false, false, true,  true,  true,  false, false, true,  false ],	// PRN 28
	[false, true,  true,  false, false, true,  false, false, true,  true,  true,  false, false, true,  false, true,  true,  true,  true,  false, false, false, true,  true,  true,  false, false ],	// PRN 29
	[true,  true,  true,  false, false, true,  false, false, false, true,  false, false, true,  false, true,  false, true,  false, false, false, false, false, false, false, true,  true,  true  ],	// PRN 30
	[true,  true,  true,  false, true,  false, false, true,  false, true,  false, false, true,  true,  false, false, true,  false, false, false, true,  false, true,  true,  false, true,  true  ],	// PRN 31
	[false, false, false, true,  false, true,  false, false, false, false, false, true,  true,  true,  true,  false, true,  false, false, true,  false, false, false, true,  false, true,  true  ]	// PRN 32
	];

const CL_INITIAL_STATE:[[bool; 27]; 32] = [
	[ true,  true, false, false,  true, false,  true, false, false, false, false,  true,  true, false, false,  true, false,  true,  true,  true,  true,  true,  true,  true, false,  true, false],	// PRN 01
	[ true, false,  true, false, false, false,  true,  true, false,  true,  true, false, false, false,  true, false, false, false, false,  true,  true,  true,  true, false, false,  true, false],	// PRN 02
	[false,  true, false, false,  true, false, false, false, false, false,  true,  true,  true,  true, false, false, false, false, false, false, false, false, false,  true,  true,  true, false],	// PRN 03
	[ true,  true,  true, false, false,  true, false, false, false,  true, false, false, false, false, false,  true,  true, false, false, false,  true, false, false, false,  true, false, false],	// PRN 04
	[false, false, false, false, false, false, false, false,  true, false, false,  true,  true, false, false, false,  true,  true, false,  true,  true,  true, false, false,  true, false,  true],	// PRN 05
	[false, false, false,  true, false,  true, false,  true,  true, false, false, false, false,  true, false, false,  true,  true, false,  true,  true, false,  true, false,  true,  true, false],	// PRN 06
	[ true,  true, false,  true, false,  true, false,  true, false,  true, false,  true, false,  true, false, false, false,  true, false,  true, false,  true,  true,  true,  true,  true, false],	// PRN 07
	[false,  true, false, false, false, false,  true,  true, false, false, false,  true, false,  true, false,  true, false, false,  true,  true,  true,  true,  true,  true,  true,  true,  true],	// PRN 08
	[false, false, false, false, false,  true,  true, false,  true,  true, false,  true,  true,  true, false, false,  true,  true, false,  true,  true,  true,  true,  true,  true, false, false],	// PRN 09
	[ true, false,  true,  true,  true, false, false, false,  true,  true, false,  true, false,  true, false, false,  true, false, false, false, false,  true,  true,  true,  true,  true, false],	// PRN 10
	[false, false, false, false,  true, false, false,  true,  true, false, false,  true,  true,  true, false, false,  true,  true,  true, false,  true, false,  true, false,  true, false,  true],	// PRN 11
	[false, false,  true, false, false,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true, false,  true, false, false,  true, false,  true, false, false, false],	// PRN 12
	[ true,  true, false, false, false, false,  true,  true, false,  true, false,  true, false, false,  true,  true,  true, false, false,  true,  true,  true, false,  true,  true, false,  true],	// PRN 13
	[false, false, false, false, false, false, false,  true,  true, false, false, false, false,  true,  true,  true,  true,  true, false,  true,  true,  true, false, false, false,  true,  true],	// PRN 14
	[false, false, false,  true, false, false,  true,  true, false,  true, false,  true, false, false,  true,  true, false,  true,  true, false,  true,  true,  true, false,  true, false,  true],	// PRN 15
	[ true,  true, false,  true,  true,  true, false, false,  true,  true, false,  true, false, false,  true, false, false,  true,  true,  true, false, false,  true, false, false, false,  true],	// PRN 16
	[ true,  true, false, false, false, false,  true, false,  true,  true, false, false, false, false, false, false,  true, false, false,  true, false, false,  true, false, false, false, false],	// PRN 17
	[false, false, false, false, false, false, false,  true, false,  true, false,  true,  true,  true,  true,  true,  true, false, false,  true, false, false, false, false,  true,  true,  true],	// PRN 18
	[ true, false,  true, false,  true, false,  true, false,  true, false, false,  true,  true,  true, false, false,  true,  true,  true, false, false,  true, false,  true, false, false,  true],	// PRN 19
	[false,  true, false,  true,  true, false,  true,  true, false,  true, false,  true, false,  true, false,  true,  true,  true,  true,  true,  true,  true,  true, false,  true, false,  true],	// PRN 20
	[false, false, false, false, false, false,  true,  true, false,  true,  true,  true,  true,  true, false, false, false, false,  true,  true,  true, false, false, false, false,  true,  true],	// PRN 21
	[ true, false,  true, false, false, false, false, false,  true,  true, false, false,  true,  true,  true,  true, false, false,  true, false,  true,  true, false,  true,  true,  true, false],	// PRN 22
	[ true,  true,  true,  true, false, false, false,  true,  true,  true,  true,  true,  true, false, false,  true,  true,  true,  true, false, false,  true, false, false, false,  true,  true],	// PRN 23
	[ true,  true, false, false, false,  true,  true, false,  true,  true, false,  true, false,  true,  true,  true, false, false,  true,  true,  true, false,  true, false,  true,  true, false],	// PRN 24
	[ true,  true,  true,  true,  true, false, false,  true,  true,  true,  true, false, false,  true, false, false, false,  true,  true, false, false, false,  true, false, false, false, false],	// PRN 25
	[ true,  true,  true, false,  true, false, false, false, false,  true,  true,  true, false,  true, false,  true,  true,  true,  true, false, false,  true,  true,  true,  true, false, false],	// PRN 26
	[ true,  true,  true, false, false, false, false, false, false,  true, false,  true, false,  true, false, false, false,  true, false, false, false,  true, false, false, false,  true,  true],	// PRN 27
	[false,  true, false, false,  true, false, false,  true, false,  true, false,  true,  true,  true, false,  true,  true,  true, false,  true, false,  true,  true, false, false,  true,  true],	// PRN 28
	[false, false,  true, false,  true,  true, false,  true, false,  true,  true,  true,  true,  true, false,  true, false,  true, false,  true,  true, false, false, false,  true, false, false],	// PRN 29
	[ true,  true,  true,  true, false, false,  true,  true, false, false,  true,  true, false,  true,  true, false,  true, false, false,  true, false,  true, false, false,  true, false,  true],	// PRN 30
	[false, false,  true, false, false, false, false,  true, false, false,  true,  true, false, false, false, false, false, false,  true, false, false,  true,  true, false,  true,  true, false],	// PRN 31
	[false,  true, false,  true, false,  true,  true, false,  true, false,  true, false, false,  true,  true, false, false,  true,  true,  true,  true, false, false,  true,  true,  true, false]	// PRN 32
	];

// TODO: move the final state constant to a unit test; it's only really needed for testing
const CM_FINAL_STATE:[[bool; 27]; 32] = [
	[true, false, true, true, false, true, false, true, false, true, false, true, true, true, false, true, true, false, false, false, false,  false, false, false,  false, true, false, ],		// PRN 01
	[false, false, false,  false, true, true, true, false, false, true, false, false, true, false, false, true, false, true, false, false, false,  false, true, true, true, false, false, ],		// PRN 02
	[true, true, true, false, true, false, false, true, true, true, false, false, true, false, false, false, true, true, true, true, true, false, false, true, false, false, true, ],		// PRN 03
	[true, false, true, false, false, true, false, false, true, false, true, false, false, true, false, false, true, false, false, false, false,  false, false, true, false, true, true, ],		// PRN 04
	[true, false, false, true, true, false, false, true, true, false, false, false,  true, false, true, true, false, true, false, true, false, false, false, true, false, true, true, ],		// PRN 05
	[true, true, false, true, true, false, true, true, true, false, false, false,  true, false, false, true, false, false, true, false, true, false, true, false, true, false, false, ],		// PRN 06
	[true, true, false, true, false, true, false, true, false, false, true, true, false, true, false, false, true, false, true, true, false, true, false, true, false, true, true, ],		// PRN 07
	[true, false, true, false, false, false,  true, false, true, true, true, true, false, false, false,  false, true, true, false, true, true, true, false, false, true, false, false, ],		// PRN 08
	[true, false, true, false, true, false, false, false, false,  false, true, true, false, false, false,  false, true, false, true, true, true, true, true, true, true, false, true, ],		// PRN 09
	[false, true, false, true, false, false, true, false, false, false, true, false, false, false, false,  true, false, true, true, false, true, false, false, false,  true, true, false, ],		// PRN 10
	[false, true, false, false, true, true, true, true, false, false, false, true, true, true, true, true, false, false, false, false, false,  false, false, false,  false, true, false, ],		// PRN 11
	[true, true, false, true, false, true, true, false, false, false, true, true, false, false, false,  true, false, true, true, false, true, false, true, true, false, false, true, ],		// PRN 12
	[true, false, false, false, true, true, true, false, true, false, false, false,  true, true, true, false, false, false,  true, false, true, true, true, true, false, false, true, ],		// PRN 13
	[true, true, false, false, true, true, false, false, false,  true, false, false, false, true, true, false, false, true, false, true, false, true, false, true, false, false, true, ],		// PRN 14
	[false, true, false, false, true, true, true, false, false, false, false, false,  true, false, false, false, true, true, true, false, false, false, false, true, true, true, true, ],		// PRN 15
	[true, false, true, false, true, true, true, false, true, true, false, true, true, false, false, false, false, false,  true, true, true, true, false, false, true, false, true, ],		// PRN 16
	[false, false, false,  true, false, false, false, true, true, false, false, false,  true, false, true, true, true, false, true, true, true, false, true, true, true, false, false, ],		// PRN 17
	[true, true, true, false, true, true, false, false, true, false, true, true, false, false, false,  true, false, false, false, false, true, false, false, false,  false, true, true, ],		// PRN 18
	[true, false, false, false, false, true, false, true, false, false, false, true, false, true, false, false, false, false,  false, false, true, false, false, false,  true, false, true, ],		// PRN 19
	[false, true, true, true, true, false, true, false, true, true, true, false, false, true, true, true, true, false, false, false, true, false, false, true, false, false, true, ],		// PRN 20
	[false, false, true, true, false, false, false, true, true, false, true, true, false, true, false, true, false, false, true, true, false, true, false, true, true, true, true, ],		// PRN 21
	[false, false, true, false, false, true, false, false, false,  true, true, true, true, true, false, true, true, false, true, false, false, true, true, false, false, true, false, ],		// PRN 22
	[true, true, false, false, false, false,  false, true, false, true, false, false, false, false, false,  true, false, true, false, true, false, false, false, false,  false, true, true, ],		// PRN 23
	[false, false, true, true, true, true, true, true, true, true, true, true, false, true, true, true, false, true, true, true, false, true, false, true, false, false, false,  ],		// PRN 24
	[true, true, false, false, true, true, false, false, false,  false, false, true, true, true, true, true, true, true, true, false, true, true, true, false, false, false, false,  ],		// PRN 25
	[true, true, false, true, false, true, false, true, true, true, false, false, true, true, false, true, true, true, false, false, true, false, false, false,  true, true, true, ],		// PRN 26
	[true, false, false, false, false, false,  true, true, false, true, false, true, true, true, true, true, true, false, true, true, false, false, true, true, false, false, false,  ],		// PRN 27
	[false, true, false, false, true, false, false, false, true, true, true, true, true, true, true, true, true, true, false, false, true, false, false, false,  false, false, false,  ],		// PRN 28
	[true, true, true, true, true, true, false, true, true, false, true, false, true, true, false, true, true, false, true, true, false, true, true, true, false, true, true, ],		// PRN 29
	[false, false, true, false, false, false,  false, false, false,  false, false, false,  false, false, true, false, false, false,  true, true, true, false, false, true, false, false, false,  ],		// PRN 30
	[true, false, false, false, true, true, false, false, true, false, false, false,  false, true, true, true, true, true, false, false, true, false, true, true, false, true, false, ],		// PRN 31
	[true, true, false, false, true, false, true, false, false, false, false, true, false, true, false, true, true, true, true, false, false, true, true, true, true, false, true, ]			// PRN 32
];

const CL_FINAL_STATE:[[bool; 27]; 32] = [
	[false,  true, false,  true,  true, false,  true,  true,  true,  true,  true,  true, false,  true, false,  true, false, false, false,  true, false, false,  true,  true,  true,  true, false],	// PRN 01
	[false, false,  true,  true,  true, false,  true,  true,  true,  true, false,  true, false, false,  true,  true,  true, false, false, false, false,  true,  true, false,  true,  true, false],	// PRN 02
	[ true,  true,  true,  true,  true,  true, false, false,  true,  true,  true,  true,  true, false,  true,  true,  true, false,  true, false, false, false, false, false,  true, false,  true],	// PRN 03
	[false, false, false,  true, false, false,  true,  true,  true, false,  true, false, false, false, false, false,  true, false,  true,  true, false, false,  true, false,  true, false, false],	// PRN 04
	[false, false, false,  true, false,  true, false,  true, false,  true,  true,  true,  true,  true,  true, false, false, false,  true, false, false, false,  true,  true, false,  true,  true],	// PRN 05
	[ true,  true,  true,  true,  true, false, false, false,  true,  true,  true,  true,  true, false, false, false,  true,  true,  true,  true, false,  true,  true, false,  true, false,  true],	// PRN 06
	[false, false,  true, false,  true,  true, false,  true,  true, false, false, false, false, false,  true,  true, false,  true,  true,  true,  true, false,  true, false,  true,  true, false],	// PRN 07
	[ true,  true, false, false, false,  true, false, false, false,  true,  true, false, false, false,  true, false, false,  true,  true, false,  true, false, false,  true, false, false,  true],	// PRN 08
	[false,  true,  true,  true, false,  true, false,  true, false, false, false,  true,  true, false,  true, false, false, false, false,  true,  true, false,  true, false, false,  true,  true],	// PRN 09
	[false, false, false,  true, false,  true, false, false,  true, false,  true, false,  true,  true, false,  true,  true, false, false, false, false,  true, false, false,  true,  true, false],	// PRN 10
	[false,  true,  true, false, false, false,  true, false,  true,  true,  true, false, false, false,  true, false, false,  true, false,  true,  true,  true,  true,  true, false,  true,  true],	// PRN 11
	[ true, false,  true, false, false, false,  true, false, false,  true,  true, false,  true,  true,  true,  true,  true, false,  true,  true,  true,  true,  true,  true, false,  true,  true],	// PRN 12
	[false,  true, false,  true,  true,  true, false,  true, false,  true, false,  true,  true,  true,  true, false,  true, false,  true,  true, false, false,  true,  true,  true, false, false],	// PRN 13
	[ true,  true,  true, false,  true,  true, false, false,  true, false,  true,  true, false,  true, false, false, false, false,  true,  true,  true,  true,  true,  true, false, false,  true],	// PRN 14
	[ true,  true, false, false,  true,  true, false, false,  true, false,  true,  true, false,  true, false,  true,  true, false,  true, false,  true,  true,  true, false, false,  true,  true],	// PRN 15
	[false,  true, false, false,  true,  true, false, false,  true,  true, false,  true, false, false,  true,  true,  true, false, false,  true,  true,  true,  true, false, false, false, false],	// PRN 16
	[false, false, false, false,  true,  true, false, false, false, false,  true,  true,  true,  true, false,  true,  true,  true, false,  true,  true,  true,  true, false,  true,  true, false],	// PRN 17
	[ true,  true,  true, false, false,  true, false,  true,  true,  true, false,  true,  true, false, false, false,  true,  true,  true,  true, false, false, false,  true, false,  true,  true],	// PRN 18
	[false,  true, false, false,  true,  true, false,  true, false,  true,  true, false,  true,  true,  true,  true, false, false,  true,  true, false,  true, false,  true,  true, false, false],	// PRN 19
	[ true,  true, false,  true, false, false, false, false,  true,  true,  true,  true, false,  true,  true, false,  true,  true, false, false,  true,  true, false,  true,  true, false,  true],	// PRN 20
	[ true,  true,  true, false,  true,  true, false, false, false, false, false,  true, false,  true, false,  true, false,  true, false,  true,  true,  true, false, false,  true, false,  true],	// PRN 21
	[false, false, false, false, false, false, false, false, false, false,  true,  true, false, false,  true,  true,  true, false, false, false, false,  true,  true,  true,  true, false, false],	// PRN 22
	[false, false,  true,  true,  true,  true, false, false,  true, false,  true,  true, false, false,  true, false,  true,  true,  true,  true, false, false, false,  true,  true, false, false],	// PRN 23
	[false, false, false, false, false, false, false, false,  true,  true, false,  true, false,  true, false, false,  true,  true,  true,  true, false,  true,  true, false, false,  true, false],	// PRN 24
	[false, false, false, false,  true, false, false,  true,  true,  true, false, false,  true, false,  true,  true,  true,  true, false,  true, false,  true, false,  true, false, false, false],	// PRN 25
	[false,  true,  true, false,  true,  true, false, false, false,  true,  true,  true, false,  true,  true, false,  true,  true, false,  true, false,  true, false,  true,  true, false, false],	// PRN 26
	[ true,  true, false, false,  true, false,  true, false,  true, false, false, false,  true, false,  true,  true, false,  true,  true,  true,  true, false,  true, false,  true,  true, false],	// PRN 27
	[ true, false, false,  true,  true,  true,  true,  true, false,  true, false,  true, false,  true, false,  true, false, false, false, false, false,  true,  true, false, false, false,  true],	// PRN 28
	[ true,  true, false, false, false, false, false,  true, false, false, false, false,  true,  true, false,  true,  true, false, false, false, false, false,  true,  true, false, false,  true],	// PRN 29
	[false, false, false, false, false,  true, false,  true, false,  true, false, false, false, false,  true, false,  true, false,  true, false,  true, false,  true, false,  true,  true, false],	// PRN 30
	[ true,  true,  true, false, false, false,  true, false,  true, false, false,  true,  true, false, false,  true, false, false,  true, false,  true, false, false, false, false, false,  true],	// PRN 31
	[ true,  true, false, false, false,  true,  true, false,  true, false,  true,  true,  true,  true,  true, false,  true,  true, false, false,  true,  true,  true,  true, false, false,  true]	// PRN 32
	];

pub struct ModularShiftRegister {
	pub state: [bool; 27],
}

impl ModularShiftRegister {
	
	pub fn shift(&mut self) -> bool {
		let current_output:bool = self.state[26];

		self.state[26] = self.state[25];
		self.state[25] = self.state[24];
		self.state[24] = self.state[23] ^ current_output;
		self.state[23] = self.state[22] ^ current_output;
		self.state[22] = self.state[21] ^ current_output;
		self.state[21] = self.state[20] ^ current_output;
		self.state[20] = self.state[19];
		self.state[19] = self.state[18];
		self.state[18] = self.state[17] ^ current_output;
		self.state[17] = self.state[16];
		self.state[16] = self.state[15] ^ current_output;
		self.state[15] = self.state[14];
		self.state[14] = self.state[13] ^ current_output;
		self.state[13] = self.state[12];
		self.state[12] = self.state[11];
		self.state[11] = self.state[10] ^ current_output;
		self.state[10] = self.state[ 9];
		self.state[ 9] = self.state[ 8];
		self.state[ 8] = self.state[ 7] ^ current_output;
		self.state[ 7] = self.state[ 6];
		self.state[ 6] = self.state[ 5] ^ current_output;
		self.state[ 5] = self.state[ 4];
		self.state[ 4] = self.state[ 3];
		self.state[ 3] = self.state[ 2] ^ current_output;
		self.state[ 2] = self.state[ 1];
		self.state[ 1] = self.state[ 0];
		self.state[ 0] = current_output;

		current_output
	}

}

pub fn cm_code(prn:usize) -> [bool; 10230] {
	if prn >= 1 && prn <= 32 {
		let mut ans:[bool; 10230] = [false; 10230];
		let mut shift_reg = ModularShiftRegister{ state: CM_INITIAL_STATE[prn-1] };
		for idx in 0..10230 { 
			// TODO: move this check to a unit test
			if idx == 10229 {
				for idx in 0..27 { 
					assert!(shift_reg.state[idx] == CM_FINAL_STATE[prn-1][idx]); 
				}
			}
			ans[idx] = shift_reg.shift(); 
		}
		ans
	} else {
		panic!("Invalid PRN number for CM code generation");
	}
}

pub fn cl_code(prn:usize) -> [bool; 767250] {
	if prn >= 1 && prn <= 32 {
		let mut ans:[bool; 767250] = [false; 767250];
		let mut shift_reg = ModularShiftRegister{ state: CL_INITIAL_STATE[prn-1] };
		for idx in 0..767250 { 
			// TODO: move this check to a unit test
			if idx == 767249 {
				for idx in 0..27 { 
					assert!(shift_reg.state[idx] == CL_FINAL_STATE[prn-1][idx]); 
				}
			}
			ans[idx] = shift_reg.shift(); 
		}
		ans
	} else {
		panic!("Invalid PRN number for CL code generation");
	}
}