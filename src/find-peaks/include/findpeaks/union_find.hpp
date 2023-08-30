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

#ifndef FINDPEAK_UNION_FIND_H
#define FINDPEAK_UNION_FIND_H

#include <vector>

namespace findpeaks {

typedef std::make_signed_t<size_t> signed_size_t;

class union_find {

public:
	union_find(size_t N);

	void add(size_t element, signed_size_t weight);

	bool contains(size_t element);

	size_t find_set(size_t element);

	void join(size_t a, size_t b);

private:
	std::vector<size_t> parents;
	std::vector<signed_size_t> weights;

	static const size_t NOPARENT;
};

}


#endif //FINDPEAK_UNION_FIND_H
