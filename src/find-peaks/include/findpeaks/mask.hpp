/**
	Findpeaks: mask
	Find peaks by applying a local maximum filter

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

#ifndef FINDPEAKS_MASK_HPP
#define FINDPEAKS_MASK_HPP

#include <findpeaks/findpeaks.hpp>
#include <vector>

namespace findpeaks {



template <typename pixel_data_type, typename pixel_coordinate_type, extremum_t extremum>
struct comparer{
	static inline bool compare(const pixel_data_type &a, const pixel_data_type &b);
	static bool compare_by_value(const pixel_t2<pixel_data_type, pixel_coordinate_type> &a, const pixel_t2<pixel_data_type, pixel_coordinate_type> &b);
};

template <typename pixel_data_type, typename pixel_coordinate_type>
struct comparer<pixel_data_type, pixel_coordinate_type, maximum>{
	static inline bool compare(const pixel_data_type &a, const pixel_data_type &b){
		return a >= b;
	}
	static bool compare_by_value(const pixel_t2<pixel_data_type, pixel_coordinate_type> &a, const pixel_t2<pixel_data_type, pixel_coordinate_type> &b){
		return a.value > b.value;
	}
};

template <typename pixel_data_type, typename pixel_coordinate_type>
struct comparer<pixel_data_type, pixel_coordinate_type, minimum>{
	static inline bool compare(const pixel_data_type &a, const pixel_data_type &b){
		return a <= b;
	}
	static bool compare_by_value(const pixel_t2<pixel_data_type, pixel_coordinate_type> &a, const pixel_t2<pixel_data_type, pixel_coordinate_type> &b){
		return a.value < b.value;
	}
};


template <typename pixel_data_type, extremum_t extremum = maximum>
std::vector<pixel_t2<pixel_data_type>> mask(image_t<pixel_data_type> &image, bool process_edges = true){
	std::vector<pixel_t2<pixel_data_type>> peaks;

	// process edges separately, so that we don't have to check for bounds
	// in the middle of the image (improves efficiency)

	// 1D data
	if(image.width == 1 || image.height == 1){
		const size_t N = image.width * image.height;
		if(comparer<pixel_data_type, size_t, extremum>::compare(image.data[0], image.data[1])){
			peaks.push_back({image.data[0], {0,0}});
		}
		if(comparer<pixel_data_type, size_t, extremum>::compare(image.data[N-1], image.data[N-2])){
			peaks.push_back({image.data[N-1], index_1d_to_2d(N-1, image.width, image.height)});
		}

		for(size_t i=1; i<=N-2; i++){
			if(
				comparer<pixel_data_type, size_t, extremum>::compare(image.data[i], image.data[i-1]) &&
				comparer<pixel_data_type, size_t, extremum>::compare(image.data[i], image.data[i+1])
			){
				peaks.push_back({image.data[i], index_1d_to_2d(i, image.width, image.height)});
			}
		}

		return peaks;
	}

	// it's guaranteed that image here is minimum 2x2

	/*
		         x
		  ------->
		  |
		  |
		y v
	 */


	pixel_data_type pixel_val;
	if(!process_edges) {
		goto process_middle;
	}

	// top left corner
	pixel_val = image.get_pixel_value(0,0);
	if(
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1,0)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(0,1)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1,1))
	){
		peaks.push_back({pixel_val, {0,0}});
	}

	// top right corner
	pixel_val = image.get_pixel_value(image.width-1,0);
	if(
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2,0)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-1,1)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2,1))
	){
		peaks.push_back({pixel_val, {image.width-1,0}});
	}

	// bottom right corner
	pixel_val = image.get_pixel_value(image.width-1,image.height-1);
	if(
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2,image.height-1)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-1,image.height-2)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2,image.height-2))
	){
		peaks.push_back({pixel_val, {image.width-1,image.height-1}});
	}

	// bottom left corner
	pixel_val = image.get_pixel_value(0,image.height-1);
	if(
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1,image.height-1)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(0,image.height-2)) &&
		comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1,image.height-2))
	){
		peaks.push_back({pixel_val, {0,image.height-1}});
	}

	// top edge
	for(size_t x=1; x<=image.width-2; x++){
		pixel_val = image.get_pixel_value(x,0);
		if(
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1,0)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1,0)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1,1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x  ,1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1,1))
		){
			peaks.push_back({pixel_val, {x, 0}});
		}
	}

	// bottom edge
	for(size_t x=1; x<=image.width-2; x++){
		pixel_val = image.get_pixel_value(x,image.height-1);
		if(
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1,image.height-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1,image.height-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1,image.height-2)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x  ,image.height-2)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1,image.height-2))
		){
			peaks.push_back({pixel_val, {x, image.height-1}});
		}
	}

	// left edge
	for(size_t y=1; y<=image.height-2; y++){
		pixel_val = image.get_pixel_value(0,y);
		if(
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(0, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(0, y+1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1, y  )) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(1, y+1))
		){
			peaks.push_back({pixel_val, {0,y}});
		}
	}

	// right edge
	for(size_t y=1; y<=image.height-2; y++){
		pixel_val = image.get_pixel_value(image.width-1,y);
		if(
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-1, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-1, y+1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2, y  )) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(image.width-2, y+1))
		){
			peaks.push_back({pixel_val, {image.width-1,y}});
		}
	}

process_middle:

	for(size_t x=1; x<=image.width-2; x++){
	for(size_t y=1; y<=image.height-2; y++){
		// now it is guaranteed to have 8 neighbours
		pixel_val = image.get_pixel_value(x,y);

		if(
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1, y+1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1, y+1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1, y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x  , y-1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x  , y+1)) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x-1, y  )) &&
			comparer<pixel_data_type, size_t, extremum>::compare(pixel_val, image.get_pixel_value(x+1, y  ))
		){
			peaks.push_back({pixel_val, {x,y}});
		}
	}
	}

	std::sort(peaks.begin(), peaks.end(), comparer<pixel_data_type, size_t, extremum>::compare_by_value);

	return peaks;

}


}

#endif //FINDPEAKS_MASK_HPP
