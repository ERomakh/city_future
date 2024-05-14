#ifndef classif
#define classif

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <functional>
#include <algorithm>
#include <unordered_map>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

#include "data_preparation.hpp"

// namespace fs = std::filesystem;

namespace cl {
    class Cl_bild {
        public:
            std::vector<OGRPoint> point_vector;
            v_doub population;
            v_doub workplaces;
            v_string category;
            v_doub levels;
            v_doub area;
            v_doub relative_density;
            
            void save_shapefile(std::string save_path, OGRSpatialReference srs_d);
            void logistic_factory(std::unique_ptr<preparation::Buildings>& prepared_bds, std::unique_ptr<preparation::Buildings>& without_factories,
                    std::unique_ptr<cl::Cl_bild>& final_data);
            void logistic_office(std::unique_ptr<preparation::Buildings>& without_factories, std::unique_ptr<preparation::Buildings>& only_house);
            void house_category(std::unique_ptr<preparation::Buildings>& only_house, std::unique_ptr<io::Genplan_data>& gendata);
            void fill_levels();
            void linear_regression(std::unique_ptr<cl::Cl_bild>& classificated, std::unique_ptr<cl::Cl_bild>& factory_office);
            void estimate_population(std::unique_ptr<io::Genplan_data>& gendata);
            void estimate_workforce(std::unique_ptr<io::Genplan_data>& gendata);
            void density_map(std::string fishnet_path, OGRSpatialReference srs_d, int sq_size_len);
    };

    std::vector<std::tuple<std::string, OGRPolygon>> extended_areas(std::string pathext, OGRSpatialReference osrs_d);
    void make_empty_buildings(std::unique_ptr<cl::Cl_bild>& classificated, std::unique_ptr<io::Genplan_data>& gendata,
                                std::string pathfrom, std::string pathto, OGRSpatialReference osrs_d, std::unique_ptr<io::Genplan_data>& gendata2);
}

#endif