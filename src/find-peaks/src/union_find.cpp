/**
	Simple disjoint sets, aka union find implementation.
	Ported from Python: https://git.sthu.org/?p=persistence.git;hb=HEAD

    Copyright (C) 2022  University College London
	developed by: Balázs Dura-Kovács (b.dura-kovacs@ucl.ac.uk)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <findpeaks/union_find.hpp>
#include <stdexcept>
#include <limits>

namespace findpeaks {

const size_t union_find::NOPARENT = std::numeric_limits<size_t>::max();

union_find::union_find(size_t N) : parents(N, union_find::NOPARENT), weights(N) {

}

void union_find::add(size_t element, signed_size_t weight) {
	if (parents[element] == union_find::NOPARENT) {
		parents[element] = element;
		weights[element] = weight;
	}
}

bool union_find::contains(size_t element) {
	return parents[element] != union_find::NOPARENT;
}

size_t union_find::find_set(size_t element) {
	if (parents[element] == union_find::NOPARENT) {
		throw std::invalid_argument("Element is not in forest");
	}

	// find path of nodes leading to the root
	std::vector<size_t> path = {element};
	size_t root = parents[element];
	while (root != path.back()) {
		path.push_back(root);
		root = parents[root];
	}

	// compress the path
	for (const size_t &ancestor: path) {
		parents[ancestor] = root;
	}

	return root;
}

void union_find::join(size_t a, size_t b) {
	size_t root_a = find_set(a);
	size_t root_b = find_set(b);

	if (weights[root_a] > weights[root_b]) {
		parents[root_b] = root_a;
	} else {
		parents[root_a] = root_b;
	}
}

}