/**
	Findpeaks: small library to find local extrema in 1D or 2D datasets
 	based on the persistent homology method.

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

#include <iostream>
#include <vector>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <highfive/H5Easy.hpp>
#include <findpeaks/persistence.hpp>
#include <findpeaks/mask.hpp>
#include <findpeaks/triangulate_mask.hpp>
#include <findpeaks/triangulate_persistence.hpp>


std::chrono::steady_clock::time_point tic(){
	return std::chrono::steady_clock::now();
}

std::chrono::duration<double, std::milli> toc(std::chrono::steady_clock::time_point begin, bool print = true){
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	std::chrono::duration<double, std::milli> diff = now - begin;

	if(print){
		printf("%.2f ms\n", diff);
	}

	return diff;
}


int main(int argc, char* argv[]) {
	if(argc < 2){
		std::cout << "Usage: " << argv[0] << " data.h5 [benchmark iterations = 1]\n";
		return 1;
	}

	HighFive::File input_file(argv[1], HighFive::File::ReadOnly);

	HighFive::DataSet dataset = input_file.getDataSet("/im");
	std::vector<size_t> dimensions = dataset.getDimensions();
	if(dimensions.size() == 1){
		dimensions[1] = 1;
	}
	if(dimensions.size() > 2){
		std::cerr << "Please supply 1D or 2D data\n";
		return 1;
	}
	std::vector<double> data(dimensions[0]*dimensions[1]);
	dataset.read<double>(data.data());


	findpeaks::image_t<double> image = {
			dimensions[0], dimensions[1],
			&data[0]
	};

	int iterations = 1;
	if(argc >= 3){
		iterations = std::atoi(argv[2]);
		if(iterations == 0){
			iterations = 1;
		}
	}

	std::cout << "Find peaks by topology method\n" <<
				 "=============================\n\n";

	std::vector<findpeaks::peak_t<double>> peaks;
	auto begin = tic();
	for(int i = 0; i<iterations; i++){
		peaks = findpeaks::persistance(image);
	}
	toc(begin);

	for(const auto &p: peaks){
		std::cout  << "(" << p.birth_position.x << ", " << p.birth_position.y << ")\t"
		<< p.birth_level << "  " << p.persistence
		<< "\t(" << p.death_position.x << ", " << p.death_position.y << ")\n";
	}

	std::cout << "\n\nFind peaks by mask method\n" <<
					 "=========================\n\n";

	std::vector<findpeaks::pixel_t2<double>> peaks_by_mask;
	begin = tic();
	for(int i = 0; i<iterations; i++){
		peaks_by_mask = findpeaks::mask<double, findpeaks::maximum>(image, false);
	}
	toc(begin);

	for(const auto &p: peaks_by_mask){
		std::cout  << "(" << p.position.x << ", " << p.position.y << ")\t"
		<< p.value << "\n";
	}

	std::cout << "\n\nFind peaks by triangulated mask method\n" <<
				     "======================================\n\n";

	// create list of input pixels
	std::vector<findpeaks::pixel_t2<double, double>> pixel_list(dimensions[0]*dimensions[1]);
	size_t j=0;
	for(size_t x=0; x<dimensions[0]; x++){
	for(size_t y=0; y<dimensions[1]; y++){
		pixel_list[j++] = {
			image.get_pixel_value(x, y), {
				(double) x, (double) y
			}
		};
	}
	}

	std::vector<findpeaks::pixel_t2<double, double>> peaks_by_triangulated_mask;
	begin = tic();
	for(int i = 0; i<iterations; i++){
		peaks_by_triangulated_mask = findpeaks::triangulate_mask<double, double, findpeaks::maximum>(pixel_list);
	}
	toc(begin);

	for(const auto &p: peaks_by_triangulated_mask){
		std::cout  << "(" << p.position.x << ", " << p.position.y << ")\t"
		<< p.value << "\n";
	}

	std::cout << "\n\nFind peaks by triangulated persistence method\n" <<
				     "=============================================\n\n";



	std::vector<findpeaks::peak_t<double, double>> peaks_by_triangulated_persistence;
	begin = tic();
	for(int i = 0; i<iterations; i++){
		peaks_by_triangulated_persistence = findpeaks::triangulate_persistance<double, double>(pixel_list, findpeaks::maximum);
	}
	toc(begin);

	for(const auto &p: peaks_by_triangulated_persistence){
		std::cout  << "(" << p.birth_position.x << ", " << p.birth_position.y << ")\t"
		<< p.birth_level << "  " << p.persistence
		<< "\t(" << p.death_position.x << ", " << p.death_position.y << ")\n";
	}

}
