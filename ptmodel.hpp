#ifndef ptmodel_a
#define ptmodel_a

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
#include <igraph.h>

#include "gtfs_data.hpp"

namespace transmodel {

    class PT_model {
        public:
            std::map<std::tuple<int, int>, std::tuple<double, std::string>> edge_data;
            std::unordered_map<int, int> edge_stop_connection; // stop id -> node id
            std::unordered_map<int, v_int> node_closest_wp; // node id -> closest wp
            std::unordered_map<int, v_int> stop_workplaces_poss;
            std::unordered_map<int, igraph_integer_t> vert_id;
            std::unordered_map<int, v_int> all_avialable_wp;
            std::unordered_map<int, int> stop_wp_connection; // stop id
            std::unordered_map<int, int> node_wp_connection; // node id -> total workforce
            std::unordered_map<int, double> node_population; // node id
            std::unordered_map<std::string, int> route_wp;
            std::unordered_map<int, v_int> workplaces_near_stop; // id of workplaces near node id
            std::unordered_map<int, v_int> stop_workplace; // stop id -> workplaces near stopid
            std::unordered_map<int, int> total_wp_avaliable; // only stop id
            std::unordered_map<int, double> start_pressure;  // only stop id
            std::unordered_map<int, double> solved_model; // node id -> number od exits
            std::unordered_map<int, double> nodeid_pressure; // node id -> total number of passangers
            v_int routes_vec;
            v_doub route_total_wp;
            v_int capacity_vec;
            v_int load_points;
            v_int entrance_vec;
            v_int exit_vec;

            void make_pt_model(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void total_route_wp(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines);
            void total_avaliable(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void model_solve();

    };

    class Car_model {
         public:
            std::map<std::tuple<int, int>, std::tuple<std::tuple<int, int>, std::tuple<int, int>, double>> car_data;
            std::map<int, std::tuple<int, int>> car_node_data;
            std::unordered_map<int, int> settl_node; // settlement id -> car node
            std::unordered_map<int, int> working_node; // workplace id -> car node

            void create_model(std::map<std::tuple<int, int>, std::tuple<std::tuple<int, int>, std::tuple<int, int>, double>> data);


    };

    class Settlement_area{
        public:
            std::vector<OGRPoint> point_vector;
            v_int set_id_vec;
            std::vector<v_int> closest_stops;
            std::unordered_map<int, v_int> house_and_wp; // setid -> vector of workplace id
            std::unordered_map<int, v_int> road_house_wp;
            v_doub population;
            std::vector<std::tuple<int, int>> tuple_view;
            v_doub pt_users;
            std::unordered_map<int, double> pt_avaliable; // house id -> normalized workforce by public transport
            std::unordered_map<int, double> car_avaliable; // house id -> normalized workforce by car
            std::unordered_map<int, double> ped_avaliable; // house id -> normalized workforce by walk
            v_int car_places;
            v_int pt_places;
            v_int ped_places;
            std::unordered_map<int, OGRPoint> stop_connection;

            void create_settlement(std::unique_ptr<cl::Cl_bild>& cl_bd, std::unique_ptr<io::Genplan_data>& gdata);
            void make_transport_connections(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void stop_selection(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void car_settlement(std::unique_ptr<Car_model>& carmodel);
            void pt_users_prediction();
            
            
    };

    class Working_places{
        public:
            std::vector<OGRPoint> point_vector;
            std::vector<v_int> closest_stops; // workplace -> stop id
            std::unordered_map<int, v_int> closest_wp; // stop id -> workplace
            v_doub workforce;
            v_int wp_id_vec;
            std::unordered_map<int, double> workforce_id; // workplace id -> workforce
            std::unordered_map<int, OGRPoint> stop_connection;
            std::vector<std::tuple<int, int>> tuple_view;

            void create_wp(std::unique_ptr<cl::Cl_bild>& cl_bd);
            void pedestrian_choice(std::unique_ptr<Settlement_area>& settlemet, int thread_num);
            void make_transport_connections(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void make_possible_wp(std::unique_ptr<PT_model>& ptmodel);
            void possible_work_stop(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void workplace_settlement(std::unique_ptr<Car_model>& carmodel); 
    };

    class Graph_DJ{
        public:
            std::unordered_map<int, std::unordered_map<int, int>> unweighted_graph_data;
            std::unordered_map<int, std::unordered_map<int, double>> weighted_graph_data;

            void make_graph(std::vector<std::tuple<int, int>> existed_data);
            void make_weighted_graph(std::vector<std::tuple<int, int, double>> existed_weighted_data);

            std::unordered_map<int, double> djikstra_to_all(int start);

            template<typename cost_type>
            void djikstra_search(v_int& path, cost_type& total_cost, int start, int goal);
    };

    class Result_model{
        public:
            std::unordered_map<int, double> stopid_pressure; // stop id -> passengers
            std::unordered_map<int, double> stopid_load; // stop id -> load in percent
            std::unordered_map<int, double> exit_model; // stop id -> exits
            std::unordered_map<int, double> entrance_model; // stop id -> entrances
            std::vector<std::tuple<OGRPoint, double, double, double>> model_for_save; // geometry, passengers, exits, entrances

            void transfer_from_pt(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points);
            void save_model_data(std::string savepath, OGRSpatialReference srs_d);
    };

    void predict_pt_users(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<Settlement_area>& setarea, std::unique_ptr<Working_places>& wplaces,
            std::unique_ptr<gtfs::GTFS_points>& gtfs_points, int thread_num);
    void car_users(std::unique_ptr<Settlement_area>& setarea, std::unique_ptr<Car_model>& carmodel, std::unique_ptr<Working_places>& wplaces, int thread_num);

    void compare_model(std::unique_ptr<Result_model>& newmodel, std::unique_ptr<Result_model>& oldmodel, std::string spath, OGRSpatialReference srs_d);
}

#endif