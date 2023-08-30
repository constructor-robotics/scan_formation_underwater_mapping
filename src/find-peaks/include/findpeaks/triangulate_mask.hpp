/**
	Findpeaks: small library to find local extrema in sparse 2D datasets

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

#ifndef FINDPEAKS_TRIANGULATE_MASK_HPP
#define FINDPEAKS_TRIANGULATE_MASK_HPP

#include <findpeaks/findpeaks.hpp>
#include <findpeaks/mask.hpp>
#include <vector>
#include <delabella.h>


namespace findpeaks {

/**
 * First perform Delaunay triangulation on the list of input pixels
 * Then find local maxima/minima
 * A pixel is defined as local maximum if its value is >= than
 * all of its neighbours in the triangulated graph
 *
 * This method is intended for sparse data.
 *
 * Warning: pixel coordinates are converted to double if
 * they are not double already
 *
 * @tparam pixel_data_type
 * @tparam pixel_coordinate_type
 * @tparam extremum
 * @param pixels List of pixels
 * @return
 */
template <typename pixel_data_type, std::convertible_to<double> pixel_coordinate_type, extremum_t extremum = maximum>
std::vector<pixel_t2<pixel_data_type, pixel_coordinate_type>> triangulate_mask(const std::vector<pixel_t2<pixel_data_type, pixel_coordinate_type>> &input_pixels) {
	//convert pixel coordinates to double if necessary
	const std::vector<pixel_t2<pixel_data_type, double>> * pixels;
	if constexpr (std::is_same_v<pixel_coordinate_type, double>){
		pixels = &input_pixels;
	}else{
		pixels = new std::vector<pixel_t2<pixel_data_type, double>>(input_pixels.size());
		for(size_t i = 0; i<input_pixels.size(); i++){
			(*pixels)[i] = {
				input_pixels[i].value, {
					(double) input_pixels[i].position.x,
					(double) input_pixels[i].position.y
				}
			};
		}
	}

	// perform triangulation
	IDelaBella* idb = IDelaBella::Create();

	int verts = idb->Triangulate(pixels->size(), &(*pixels)[0].position.x, &(*pixels)[0].position.y, sizeof(pixel_t2<pixel_data_type, double>));
	if(verts <= 0){
		idb->Destroy();
		throw std::invalid_argument("Input pixel locations are collinear");
	}

	const int tris = verts / 3;
	const DelaBella_Triangle* dela = idb->GetFirstDelaunayTriangle();


	// comparing neighbour pixels
	// each comparison is done twice, as we iterate through all edges twice
	// but this way it's still more efficient than building a graph and iterating the edges
	std::vector<bool> is_extremum(pixels->size(), true);
	for (int i = 0; i<tris; i++){
		int ai, bi, ci;
		pixel_data_type a, b, c;

		ai = dela->v[0]->i;
		bi = dela->v[1]->i;
		ci = dela->v[2]->i;

		a = (*pixels)[ai].value;
		b = (*pixels)[bi].value;
		c = (*pixels)[ci].value;

		// this is not a runtime check
		if constexpr (extremum == maximum){
			is_extremum[ai] = is_extremum[ai] && a >= b && a >= c;
			is_extremum[bi] = is_extremum[bi] && b >= a && b >= c;
			is_extremum[ci] = is_extremum[ci] && c >= a && c >= b;
		}else{
			is_extremum[ai] = is_extremum[ai] && a <= b && a <= c;
			is_extremum[bi] = is_extremum[bi] && b <= a && b <= c;
			is_extremum[ci] = is_extremum[ci] && c <= a && c <= b;
		}

		dela = dela->next;
	}

	idb->Destroy();
	if constexpr (!std::is_same_v<pixel_coordinate_type, double>){
		delete pixels;
	}

	// return list of local extrema
	std::vector<pixel_t2<pixel_data_type, double>> peaks;
	for(size_t i = 0; i<is_extremum.size(); i++){
		if(is_extremum[i]){
			peaks.push_back(input_pixels[i]);
		}
	}

	std::sort(peaks.begin(), peaks.end(), comparer<pixel_data_type, pixel_coordinate_type, extremum>::compare_by_value);

	return peaks;

}


}
#endif //FINDPEAKS_TRIANGULATE_MASK_HPP
