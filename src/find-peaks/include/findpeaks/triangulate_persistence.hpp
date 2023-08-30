/**
	Findpeaks: small library to find local extrema in sparse 2D datasets
 	using the topology method

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

#ifndef FINDPEAKS_TRIANGULATE_PERSISTENCE_HPP
#define FINDPEAKS_TRIANGULATE_PERSISTENCE_HPP

#include <findpeaks/findpeaks.hpp>
#include <findpeaks/persistence.hpp>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <delabella.h>

namespace findpeaks {

class Graph {
public:
	void add_edge(linear_index_t a, linear_index_t b) {
		adjacency_list[a].insert(b);
		adjacency_list[b].insert(a);
	}

	const std::unordered_set<linear_index_t> &neighbours(linear_index_t a) {
		return adjacency_list[a];
	}

private:
	std::unordered_map<linear_index_t, std::unordered_set<linear_index_t>> adjacency_list;
};

/**
 * Warning: It sorts input_pixels by value in place!
 */
template<typename pixel_data_type, std::convertible_to<double> pixel_coordinate_type>
std::vector<peak_t<pixel_data_type, pixel_coordinate_type>>
triangulate_persistance(std::vector<pixel_t2<pixel_data_type, pixel_coordinate_type>> &input_pixels, extremum_t extremum = maximum) {
	switch(extremum){
		case maximum:
			std::sort(input_pixels.begin(), input_pixels.end(),
				[](pixel_t2<pixel_data_type, pixel_coordinate_type> &a, pixel_t2<pixel_data_type, pixel_coordinate_type> &b) -> bool {
					return a.value > b.value;
				});
			break;
		case minimum:
			std::sort(input_pixels.begin(), input_pixels.end(),
				[](pixel_t2<pixel_data_type, pixel_coordinate_type> &a, pixel_t2<pixel_data_type, pixel_coordinate_type> &b) -> bool {
					return a.value < b.value;
				});
			break;
	}

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

	// create adjacency graph
	// to easily find neighbour pixels
	Graph g;
	for (int i = 0; i<tris; i++){
		int ai, bi, ci;

		ai = dela->v[0]->i;
		bi = dela->v[1]->i;
		ci = dela->v[2]->i;

		g.add_edge(ai, bi);
		g.add_edge(ai, ci);
		g.add_edge(bi, ci);

		dela = dela->next;
	}

	idb->Destroy();

	union_find ds(pixels->size());

	std::map<linear_index_t, peak_t<pixel_data_type, pixel_coordinate_type>> peaks;

	// first iteration: init global maximum
	{
		pixel_t2<pixel_data_type, double> const &p = (*pixels)[0];
		const pixel_data_type &v = p.value;
		constexpr pixel_data_type inf = std::numeric_limits<pixel_data_type>::has_infinity ?
				std::numeric_limits<pixel_data_type>::infinity() : std::numeric_limits<pixel_data_type>::max();
		peaks[0] = {v, inf, input_pixels[0].position};
	}

	ordered_set_abstract<pixel_data_type> * nc;
	if(extremum == maximum){
		nc = new ordered_set<pixel_data_type, PixelCompByValueMaximum>;
	}else{
		nc = new ordered_set<pixel_data_type, PixelCompByValueMinimum>;
	}

	for(signed_size_t i = 0; i<pixels->size(); i++){
		pixel_t2<pixel_data_type, double> const &p = (*pixels)[i];
		const linear_index_t p_linear_index = i;
		pixel_data_type v = p.value; // water level

		std::unordered_set<linear_index_t> const &ni = g.neighbours(p_linear_index);


		for(const linear_index_t &q : ni){
			if(!ds.contains(q)){
				continue;
			}
			const linear_index_t set_index = ds.find_set(q);
			nc->insert({
				(*pixels)[set_index].value,
				set_index
			});
		}

		ds.add(p_linear_index, -i);

		if(!nc->empty()){
			auto nci = nc->cbegin();
			const linear_index_t oldp = nci->position;

			ds.join(oldp, p_linear_index);
			for(nci++; nci != nc->cend(); nci++){
				const linear_index_t setid = ds.find_set(nci->position);
				if(peaks.count(setid) == 0){
					peaks[setid] = {
						nci->value,
						extremum == maximum ? (nci->value-v) : (v-nci->value),
						input_pixels[setid].position,
						input_pixels[p_linear_index].position
					};
				}

				ds.join(oldp, nci->position);
			}
		}

		nc->clear();
	}

	delete nc;

	if constexpr (!std::is_same_v<pixel_coordinate_type, double>){
		delete pixels;
	}

	std::vector<peak_t<pixel_data_type, pixel_coordinate_type>> peaks_sorted(peaks.size());
	size_t i=0;
	for(auto const& [key, val] : peaks){
		peaks_sorted[i] = val;
		i++;
	}

	std::sort(peaks_sorted.begin(), peaks_sorted.end(), [](const peak_t<pixel_data_type, pixel_coordinate_type> &a, const peak_t<pixel_data_type, pixel_coordinate_type> &b){
		return a.persistence > b.persistence;
	});


	return peaks_sorted;

}

}

#endif //FINDPEAKS_TRIANGULATE_PERSISTENCE_HPP
