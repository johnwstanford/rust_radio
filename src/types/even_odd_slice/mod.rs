pub struct EvenOddSlice<'a, T:'a> {
	items: &'a Vec<T>,
	start: usize,
	step: usize,
	current: usize,
}

pub fn new<'a, T:'a>(x: &'a Vec<T>) -> EvenOddSlice<T> {
	EvenOddSlice{items: x, start: 0, step: 1, current: 0}
}

impl<'a, T:'a> Clone for EvenOddSlice<'a, T> {
	fn clone(&self) -> EvenOddSlice<'a, T> {
		EvenOddSlice{items: self.items, start: self.start, step: self.step, current:self.current}
	}
}

impl<'a, T:'a> Iterator for EvenOddSlice<'a, T> {
	type Item = &'a T;

	fn next(&mut self) -> Option<&'a T> {
		if self.current >= self.items.len() {
			None
		}
		else {
			let ans = &self.items[self.current];
			self.current += self.step;
			Some(ans)
		}
	}
}

impl<'a, T> EvenOddSlice<'a, T> {

	pub fn even(&self) -> EvenOddSlice<T> {
		EvenOddSlice{items: self.items, start: self.start, step: self.step*2, current: self.start}
	}

	pub fn odd(&self) -> EvenOddSlice<T> {
		EvenOddSlice{items: self.items, start: self.start+self.step, step: self.step*2, current: self.start+self.step}
	}

	pub fn len(&self) -> usize {
		let to_go = self.items.len() - self.start;
		let mut ans = to_go / self.step;
		if ans*self.step != to_go {
			ans = ans + 1;
		}
		ans

		/* Keep this dumb, but reliable way in comments in case I need to debug
		let mut idx = self.start;
		let mut count = 0;
		while idx < self.items.len() {
			idx += self.step;
			count += 1;
		}

		if ans != count {
			panic!("EvenOddSlice.len() failure")
		}
		count*/
	}
}

