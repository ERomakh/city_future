#ifndef io_bound
#define io_bound

#include <vector>
#include <algorithm>
#include <tuple>
#include <map>
#include <unordered_map>
#include <string>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

typedef std::vector<std::string> v_string;
typedef std::vector<int> v_int;
typedef std::vector<long long> v_longlong;
typedef std::vector<double> v_doub;
typedef std::vector<unsigned long> v_ulong;
typedef std::tuple<double, double> point;
typedef std::vector<point> linestring;
typedef std::vector<linestring> line_vector;

namespace io {
    class Osm_building {
        public:
            v_longlong osm_id_vec;
            v_string building_type_vec;
            v_int levels;
            std::vector<OGRPolygon> wkt_polygon;
            v_doub square_vec;
            std::vector<point> bgeom_array;
            OGREnvelope buildings_extent;
            OGRSpatialReference osrs_spatref;

            void download_data(std::string pathname);
            void make_array();

    };

    class Osm_roads {
        public:
            v_longlong osm_id_vec;
            v_string fclass_vec;
            v_doub roads_length;
            std::vector<v_int> spatial_index;
            std::vector<OGRLineString> wkt_linestring;
            line_vector linestr_vec;
            OGREnvelope roads_extent;
            OGRSpatialReference osrs_spatref;

            void download_data(std::string pathname);
            void spindex_grid();
            void make_array();
    };

    class Osm_amenities {
        public:
            std::vector<OGRPoint> amenity_point;
    };

    class Genplan_data{
        public:
            std::vector<OGRPolygon> polygon_data;
            v_string category_genplan;
            
            std::unordered_map<std::string, int> workforce_distrib;

            void download_poly(std::string path_to_poly);
            void download_txt(std::string path_to_txt);
    };

    long long time_seconds();

    std::map<std::tuple<double, double>, std::string> make_utm_crs();

    OGRSpatialReference estimate_utm_crs(OGREnvelope envel, std::map<std::tuple<double, double>, std::string> utm_data);

    std::string get_unix_time();

    OGRMultiPolygon pbf_find_city(std::string path_to_pbf, long long city_id);

    void open_pbf(std::string path_to_pbf, std::unique_ptr<Osm_building>& osm_bld, std::unique_ptr<Osm_roads>& osm_road,
        std::unique_ptr<Osm_amenities>& apoint, long long city_id);

    std::string find_pbf(std::string exe_path);
    std::string find_genplan_shp(std::string exe_path);
    std::string find_txt_path(std::string exe_path);
    std::unordered_map<std::string, std::string> gtfs_paths(std::string exe_path);
}

#endif