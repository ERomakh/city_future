#ifndef data_prep
#define data_prep

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <functional>
#include <algorithm>
#include <unordered_map>

#include <igraph.h>
#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

#include "io.hpp"

namespace preparation{

    class Roads {
        public:
            v_longlong osm_id_vec;
            v_string fclass_vec;
            v_doub roads_length;
            std::vector<OGRLineString> ogr_linestr;
            line_vector linestr_vec;

            void change_topolgy(std::unique_ptr<io::Osm_roads>& osm_r);
    };

    class Graph_view {
        public:
            igraph_t graph;
            v_longlong vertex_graph;
            std::unordered_map<unsigned long, OGRPoint> graph_geom_connect;
            std::vector<point> graph_array_vec;
            v_doub graph_weights;
            v_doub bc_nodes;
            v_doub cc_nodes;
            
            void create_igraph(std::unique_ptr<Roads>& topo_roads);
            void calculate_centrality();
            void make_array();
            
    };

    class Buildings {
        public:
            std::unordered_map<unsigned long, long long> connections;
            v_longlong osm_id_vec;
            v_string building_type_vec;
            v_string symplified_categ;
            v_int levels;
            std::vector<OGRPolygon> ogr_poly_vec;
            std::map<long long, OGRPolygon> poly_id_geom;
            v_doub square_vec;
            v_doub relative_density;
            v_doub amenities_density;
            v_doub distance_to_center;
            std::vector<point> bgeom_array;
            v_string genplan_categ;

            v_doub bc_buildings;
            v_doub cc_buildings;

            void create_buildings(std::unique_ptr<io::Osm_building>& shp_build);
            void transfer_centrality(std::unique_ptr<Graph_view>& graph_roads, int thread_number);
            void find_density(int thread_number);
            std::vector<std::tuple<int, int>> split_data(int thread_number);
            void find_to_center();
            void get_from_genplan(std::unique_ptr<io::Genplan_data>& genplan_d);
            void simplify_categ();
    };

    v_doub amenity_density(std::unique_ptr<io::Osm_amenities>& amenities, std::unique_ptr<io::Osm_building>& builds);
}


#endif