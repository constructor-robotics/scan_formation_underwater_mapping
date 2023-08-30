/**
	Findpeaks: small library to find local extrema in 1D or 2D datasets
 	based on the persistent homology method.
	Ported Stefan Huber's code from Python: https://git.sthu.org/?p=persistence.git;hb=HEAD

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
#ifndef FINDPEAKS_PERSISTENCE_HPP
#define FINDPEAKS_PERSISTENCE_HPP

#include <findpeaks/findpeaks.hpp>
#include <findpeaks/union_find.hpp>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

namespace findpeaks {

template <typename pixel_data_type>
std::vector<linear_index_t> neighbours(const pixel_index_t<> &p, const image_t<pixel_data_type> &image){
	std::vector<linear_index_t> ret;
	/*
	ret.reserve(8);

	for(signed_size_t i = signed_size_t(p.x)-1; i<=signed_size_t(p.x)+1; i++){
	for(signed_size_t j = signed_size_t(p.y)-1; j<=signed_size_t(p.y)+1; j++){
		if(i<0 || j<0 || i>=image.width || j>=image.height || (i==p.x && j==p.y)){
			continue;
		}
		ret.push_back(index_2d_to_1d(i,j,image.width, image.height));
	}
	}*/


	ret.reserve(4);
	if(p.x>0){
		ret.push_back(index_2d_to_1d(p.x-1, p.y, image.width, image.height));
	}
	if(p.x<image.width-1){
		ret.push_back(index_2d_to_1d(p.x+1, p.y, image.width, image.height));
	}
	if(p.y>0){
		ret.push_back(index_2d_to_1d(p.x, p.y-1, image.width, image.height));
	}
	if(p.y<image.height-1){
		ret.push_back(index_2d_to_1d(p.x, p.y+1, image.width, image.height));
	}

	return ret;
}

template <typename pixel_data_type>
struct PixelCompByValueAbstract {
    virtual bool operator()(const pixel_t<pixel_data_type> &a, const pixel_t<pixel_data_type> &b) const =0;
};

template <typename pixel_data_type>
struct PixelCompByValueMaximum : public PixelCompByValueAbstract<pixel_data_type> {
    bool operator()(const pixel_t<pixel_data_type> &a, const pixel_t<pixel_data_type> &b) const{
        return a.value > b.value;
    }
};

template <typename pixel_data_type>
struct PixelCompByValueMinimum : public PixelCompByValueAbstract<pixel_data_type> {
    bool operator()(const pixel_t<pixel_data_type> &a, const pixel_t<pixel_data_type> &b) const{
        return a.value < b.value;
    }
};


template <typename pixel_data_type>
class ordered_set_abstract{
public:
	inline virtual std::pair<typename std::set<pixel_t<pixel_data_type>>::iterator, bool> insert( const pixel_t<pixel_data_type>& value ) =0;
	inline virtual bool empty() const =0;
	inline virtual typename std::set<pixel_t<pixel_data_type>>::iterator cbegin() const noexcept =0;
	inline virtual typename std::set<pixel_t<pixel_data_type>>::iterator cend() const noexcept =0;
	inline virtual void clear() =0;

};

template <typename pixel_data_type, template<typename> typename sorter >
class ordered_set : public ordered_set_abstract<pixel_data_type>{
public:
	inline std::pair<typename std::set<pixel_t<pixel_data_type>>::iterator, bool> insert( const pixel_t<pixel_data_type>& value ){
		return s.insert(value);
	}

	inline bool empty() const {
		return s.empty();
	}

	inline typename std::set<pixel_t<pixel_data_type>>::iterator cbegin() const noexcept {
		return s.cbegin();
	}

	inline typename std::set<pixel_t<pixel_data_type>>::iterator cend() const noexcept {
		return s.cend();
	}

	inline void clear(){
		 s.clear();
	}

private:
	std::set<pixel_t<pixel_data_type>, sorter<pixel_data_type>> s;

};


template <typename pixel_data_type>
std::vector<peak_t<pixel_data_type>> persistance(image_t<pixel_data_type> &image, extremum_t extremum = maximum){
	std::vector<pixel_index_t<>> indices(image.width * image.height);

	{
		size_t ii = 0;
		for (size_t j = 0; j < image.height; j++) {
		for (size_t i = 0; i < image.width; i++) {
			indices[ii] = {i, j};
			ii++;
		}
		}
	}


	switch(extremum){
		case maximum:
			std::sort(indices.begin(), indices.end(), [&image](pixel_index_t<> &a, pixel_index_t<> &b) -> bool {
				return image.get_pixel_value(a) > image.get_pixel_value(b);
			});
			break;
		case minimum:
			std::sort(indices.begin(), indices.end(), [&image](pixel_index_t<> &a, pixel_index_t<> &b) -> bool {
				return image.get_pixel_value(a) < image.get_pixel_value(b);
			});
			break;
	}


	union_find ds(image.height*image.width);

	std::map<linear_index_t, peak_t<pixel_data_type>> peaks;

	// first iteration: init global maximum
	{
		pixel_index_t<> const &p = indices[0];
		const linear_index_t linear_index = index_2d_to_1d(p.x,p.y,image.width, image.height);
		const pixel_data_type v = image.get_pixel_value(linear_index);
		constexpr pixel_data_type inf = std::numeric_limits<pixel_data_type>::has_infinity ?
				std::numeric_limits<pixel_data_type>::infinity() : std::numeric_limits<pixel_data_type>::max();
		peaks[linear_index] = {v, inf, p};
	}

	ordered_set_abstract<pixel_data_type> * nc;
	if(extremum == maximum){
		nc = new ordered_set<pixel_data_type, PixelCompByValueMaximum>;
	}else{
		nc = new ordered_set<pixel_data_type, PixelCompByValueMinimum>;
	}

	for(signed_size_t i = 0; i<indices.size(); i++){
		pixel_index_t<> const &p = indices[i];
		linear_index_t p_linear_index = index_2d_to_1d(p.x,p.y,image.width, image.height);
		pixel_data_type v = image.get_pixel_value(p_linear_index); // water level

		std::vector<linear_index_t> ni = neighbours(p, image);


		for(const linear_index_t &q : ni){
			if(!ds.contains(q)){
				continue;
			}
			const linear_index_t set_index = ds.find_set(q);
			nc->insert({
				image.get_pixel_value(set_index),
				set_index
			});
		}

		ds.add(p_linear_index, -i);

		if(!nc->empty()){
			auto nci = nc->cbegin();
			const linear_index_t oldp = nci->position;
			//ds.union_set(oldp, p_linear_index);
			ds.join(oldp, p_linear_index);
			for(nci++; nci != nc->cend(); nci++){
				const linear_index_t setid = ds.find_set(nci->position);
				if(peaks.count(setid) == 0){
					peaks[setid] = {
						nci->value,
						extremum == maximum ? (nci->value-v) : (v-nci->value),
						index_1d_to_2d(setid, image.width, image.height),
						p
					};
				}

				ds.join(oldp, nci->position);
			}
		}

		nc->clear();

	}

	delete nc;

	std::vector<peak_t<pixel_data_type>> peaks_sorted(peaks.size());
	size_t i=0;
	for(auto const& [key, val] : peaks){
		peaks_sorted[i] = val;
		i++;
	}

	std::sort(peaks_sorted.begin(), peaks_sorted.end(), [](const peak_t<pixel_data_type> &a, const peak_t<pixel_data_type> &b){
		return a.persistence > b.persistence;
	});


	return peaks_sorted;
}

}

#endif //FINDPEAKS_PERSISTENCE_HPP
