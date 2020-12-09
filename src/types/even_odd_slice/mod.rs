pub struct EvenOddSlice<'a, T:'a> {
	items: &'a [T],
	start: usize,
	step: usize,
	current: usize,
}

pub fn new<'a, T:'a>(items: &'a [T]) -> EvenOddSlice<T> {
	EvenOddSlice{items, start: 0, step: 1, current: 0}
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

	pub fn even(&self) -> Self {
		EvenOddSlice{items: self.items, start: self.start, step: self.step*2, current: self.start}
	}

	pub fn odd(&self) -> Self {
		EvenOddSlice{items: self.items, start: self.start+self.step, step: self.step*2, current: self.start+self.step}
	}

	pub fn len(&self) -> usize {
		self.items.len() / self.step
	}
}

