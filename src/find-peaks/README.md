# Find peaks

This is a small C++ library to find local extrema in 1D or 2D datasets
based on the persistent homology method described in 
Herbert Edelsbrunner's Computational Topology, Chapter VII or based on a simple local
maximum filter.

## Algorithms

### 1. Topology

This is a C++ port of Stefan Huber's original implementation of the algorithm, available
[here](https://git.sthu.org/?p=persistence.git;hb=HEAD).

The same implementation is also available in the larger [findpeaks](https://github.com/erdogant/findpeaks/)
Python library.

Recommended reading on the algorithm:
* [Stefan Huber's blog post](https://www.sthu.org/code/codesnippets/imagepers.html)
* [Huber, S. (2021). Persistent Homology in Data Science. In: Haber, P., Lampoltshammer, T., Mayr, M., Plankensteiner, K. (eds) Data Science – Analytics and Applications. Springer Vieweg, Wiesbaden](https://doi.org/10.1007/978-3-658-32182-6_13)
* Edelsbrunner, H., & J Harer. (2010). Computational topology : an introduction. American Mathematical Society. (Chapter VII)

### 2. Mask

Each pixel is marked as a peak if it's the largest (or smallest) in its 8 pixel neighbourhood.
This is essentially a 3x3 sliding window filter.

When processing noisy data, this method requires pre-processing, eg. low-pass filter,
median filter, Gaussian smoothing, etc. Pre-processing methods are not implemented in this
library.

## Usage

This is a template library meaning it can deal with any kind of numerical data type,
so it doesn't matter if your data is represented as 1 byte unsigned chars, or as floats
for example. Here, we will use `double`.

First, you need to define a simple struct that contains the dimensions of the dataset and a
raw pointer to the data.

```c++
#include <findpeaks/findpeaks.hpp>

std::vector<double> v(20);
// load data to v
// ...
findpeaks::image_t<double> image = {
	1, 20,
	&v[0]
};
```

### 1. Using the topology method

You can call `persistance()` that will return a list of local maxima, sorted by
significance.

```c++
#include <findpeaks/persistence.hpp>

std::vector<findpeaks::peak_t<double>> peaks = findpeaks::persistance(image);

for(const auto &p: peaks){
    std::cout  << "(" << p.birth_position.x << ", " << p.birth_position.y << ")\t"
    << p.birth_level << "  " << p.persistence
    << "\t(" << p.death_position.x << ", " << p.death_position.y << ")\n";
}
```

To find local minima instead, call:

```c++
findpeaks::persistance(image, findpeaks::minimum)
```

### 2. Using the mask method

Calling `mask()` will return a list of detected peak locations sorted by their values.

```c++
#include <findpeaks/mask.hpp>
std::vector<findpeaks::pixel_t2<double>> peaks_by_mask = findpeaks::mask<double, findpeaks::maximum>(image, false);

for(const auto &p: peaks_by_mask){
    std::cout  << "(" << p.position.x << ", " << p.position.y << ")\t"
    << p.value << "\n";
}
```

Alternatively, `findpeaks::mask<double, findpeaks::minimum>(image, false)` would find local minima.
The last boolean parameter indicates whether the algorithms should look for peaks along
the edges of the image. If it's set to `true`, it can lead to artifacts along the edges, as
some parts of the 3x3 sliding window would be outside the image.

## Usage for 2D sparse datasets

If you have a dataset where some or most datapoints are missing or a list of (x, y, v) coordinates,
neighbour pixels are not defined, therefore it's difficult to find local extrema without applying
some heuristics. This library offers a solution though! It can perform Delaunay triangulation
on the set of datapoints using the [delabella](https://github.com/msokalski/delabella) library. After this, neighbour datapoints will be defined and it will become
perfectly feasible to run either the topology or the mask algorithm.

### 1. Using the topology method on sparse 2D dataset

The topology method can run on any triangulation, so we can make use of Delaunay triangulation.

First, you need to create a list of datapoints. In this example, we are filtering out `nan` values:

```c++
#include <cmath>
// here float is the type of the data value
// and double is the type of coordinates (pixel locations in 2D plane)
std::vector<findpeaks::pixel_t2<float, double>> pixel_list();
for(size_t x=0; x<dimensions[0]; x++){
for(size_t y=0; y<dimensions[1]; y++){
    float v = image.get_pixel_value(x, y);
    if(!std::isnan(v)){
        pixel_list.push_back({
            v, {
                (double) x, (double) y
            }
        });
    }
}
}
```

Then, you can find peaks using the topology method:

```c++
#include <findpeaks/triangulate_persistence.hpp>

// warning: pixel_list is passed by reference and will be sorted by value 
std::vector<findpeaks::peak_t<float, double>> peaks_by_triangulated_persistence 
    = findpeaks::triangulate_persistance<float, double>(pixel_list, findpeaks::maximum);


for(const auto &p: peaks_by_triangulated_persistence){
    std::cout  << "(" << p.birth_position.x << ", " << p.birth_position.y << ")\t"
    << p.birth_level << "  " << p.persistence
    << "\t(" << p.death_position.x << ", " << p.death_position.y << ")\n";
}
```

### 2. Using the mask method on sparse 2D dataset

A pixel is defined as local maximum, if its value is greater or equal to all its
neighbour pixels, where neighbours are found by Delaunay triangulation.

After defining a list of datapoints, exactly as in the previous section, you can call `triangulate_mask()`:

```c++
#include <findpeaks/triangulate_mask.hpp>

std::vector<findpeaks::pixel_t2<float, double>> peaks_by_triangulated_mask =
    findpeaks::triangulate_mask<float, double, findpeaks::maximum>(pixel_list);

for(const auto &p: peaks_by_triangulated_mask){
    std::cout  << "(" << p.position.x << ", " << p.position.y << ")\t"
    << p.value << "\n";
}
```

## Example

A full example can be found in `example/main.cpp` It can find peaks in datasets loaded from
HDF5 files like so:

```bash
./findpeak_example image.h5
```

The data must be in `/im` in the input file. Example dataset can be downloaded from [here](https://gitlab.com/balping/find-peaks/uploads/87bdbb38aa9415be5b689e86f81771c5/im.h5).

You can build the example code like this:

Install hdf5 (dev) library on your system, then:

```bash
mkdir build && cd build
cmake -DFINDPEAKS_COMPILE_EXAMPLE=ON -DCMAKE_BUILD_TYPE=release ..
make
```

## Benchmark

Comparing different implementations of different algorithms for a 201x201 image:

| Method                      | Time (ms) |
|-----------------------------|-----------|
| Python topology<sup>1</sup> | 525.11    |
| C++ topology                | 11.18     |
| Python mask<sup>2</sup>     | 1.434     |
| C++ mask                    | 0.1129    |
| C++ triangulated mask       | 21.86     |
| C++ triangulated topology   | 84.29     |

<sup>1</sup> [Implementation](https://git.sthu.org/?p=persistence.git;hb=HEAD)  
<sup>2</sup> [Implementation](https://github.com/erdogant/findpeaks/)

## License

The library is licensed under GPLv3. Please understand what this means, before reusing the code.

## Installation

You can easily re-use this library in your project using CMake's FetchContent feature, like so:

```cmake
include(FetchContent)
FetchContent_Declare(findpeaks
	GIT_REPOSITORY https://gitlab.com/balping/find-peaks.git
	GIT_TAG master
)

FetchContent_MakeAvailable(findpeaks)

target_include_directories(YOUR_TARGET PRIVATE ${findpeaks_SOURCE_DIR}/include)
target_link_libraries(YOUR_TARGET findpeaks)
```

## Demo

Peaks located in [example dataset](https://gitlab.com/balping/find-peaks/uploads/87bdbb38aa9415be5b689e86f81771c5/im.h5)
using the topology method. Brighter spots indicate more persistent peaks.

![peaks](https://gitlab.com/balping/find-peaks/uploads/3905e2deae38a06779354e37a8b35170/peaks.webp)