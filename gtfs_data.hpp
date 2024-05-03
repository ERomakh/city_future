#ifndef gtfs_data_p
#define gtfs_data_p

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

#include "classifications.hpp"


namespace gtfs {

    class GTFS_points{
        public:
            std::vector<OGRPoint> stop_coord;
            v_int stop_id_vec;
            std::vector<v_int> routes_vec;

            void download_gtfs(std::string path_stops, OGRSpatialReference spartef);
            void save_prev_points(std::string savepath, OGRSpatialReference srs_d);
    };

    class GTFS_lines{
        public:
            v_int route_id_vec;
            v_int type_of_transport;
            v_doub time_head_vec;
            v_string trip_id_vec;
            v_string trip_vec;
            std::vector<v_int> stops_in_routes;
            std::vector<OGRLineString> route_geometry;

            void make_routes(std::unordered_map<std::string, std::string> gtfs_paths, OGRSpatialReference projected);
            void save_prev_lines(std::string savepath, OGRSpatialReference srs_d);

        private:
            std::unordered_map<int, int> plain_routes(std::string path_routes);
            std::vector<std::tuple<int, std::string, std::string>> trip_load(std::string path_routes);
            std::unordered_map<std::string, double> freq_load(std::string path_routes);
            std::unordered_map<std::string, OGRLineString> shape_locations(std::string path_routes, OGRSpatialReference projected);
            std::unordered_map<std::string, v_int> stop_connections(std::string path_routes);
    };

    v_string split_string(std::string str, std::string delim);
}

#endif