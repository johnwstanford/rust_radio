pub struct EvenOddIter<'a, T:'a> {
	items: &'a [T],
	start: usize,
	step: usize,
	current: usize,
}

impl<'a, T:'a> Iterator for EvenOddIter<'a, T> {
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

impl<'a, T> EvenOddIter<'a, T> {

	pub fn from(items: &'a [T]) -> Self {
		Self{items, start: 0, step: 1, current: 0}
	}

	pub fn even(&self) -> Self {
		Self{items: self.items, start: self.start, step: self.step*2, current: self.start}
	}

	pub fn odd(&self) -> Self {
		Self{items: self.items, start: self.start+self.step, step: self.step*2, current: self.start+self.step}
	}

	pub fn len(&self) -> usize {
		self.items.len() / self.step
	}
}

