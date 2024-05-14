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
            std::map<int, OGRPoint> id_connect;
            std::vector<v_int> routes_vec;
            std::unordered_map<int, double> total_capacity; // stop id -> total possible passangers

            void download_gtfs(std::string path_stops, OGRSpatialReference spartef);
            void save_prev_points(std::string savepath, OGRSpatialReference srs_d);
            void get_metro_stations(std::string metro_path_st, OGRSpatialReference osrs);
            void make_connection();
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
            v_doub transport_capacity;

            void make_routes(std::unordered_map<std::string, std::string> gtfs_paths, OGRSpatialReference projected);
            void save_prev_lines(std::string savepath, OGRSpatialReference srs_d);
            void get_metro_lines(std::string metro_path_lines, OGRSpatialReference osrs);
            void connect_stations(std::unique_ptr<GTFS_points>& stations);
            void calculate_capacity();
            void stop_capacity(std::unique_ptr<GTFS_points>& stops);

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