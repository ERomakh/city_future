// public transport model
#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <functional>
#include <algorithm>
#include <mutex>
#include <memory>
#include <chrono>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <map>
#include <type_traits>
#include <queue>
#include <unordered_set>
#include <fstream>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>
#include <igraph.h>

#include "ptmodel.hpp"

// Graph model

void transmodel::Graph_DJ::make_graph(std::vector<std::tuple<int, int>> existed_data){

    v_int all_keys;
    v_int first_keys;
    v_int second_keys;

    for (std::tuple<int, int> ed:existed_data){
        all_keys.push_back(std::get<0>(ed));
        all_keys.push_back(std::get<1>(ed));

        first_keys.push_back(std::get<0>(ed));
        second_keys.push_back(std::get<0>(ed));
    }

    v_int unique_data = all_keys;
    std::vector<int>::iterator key_it2 = std::unique(unique_data.begin(), unique_data.end());
    unique_data.resize(std::distance(unique_data.begin(), key_it2));

    std::sort(unique_data.begin(), unique_data.end());

    for (int uident:unique_data){
        std::unordered_map<int, int> uid_data;
        for (std::tuple<int, int> sk:existed_data){
            if (std::get<0>(sk) == uident){
                if (!(uid_data.contains(std::get<1>(sk)))){
                    uid_data.emplace(std::get<1>(sk), 1);
                }
            }
        }
        if (uid_data.size() > 0){
            unweighted_graph_data.emplace(uident, uid_data);
        }
    }

}


void transmodel::Graph_DJ::make_weighted_graph(std::vector<std::tuple<int, int, double>> existed_weighted_data){

    v_int all_keys;
    v_int first_keys;
    v_int second_keys;
    v_doub weights;

    for (std::tuple<int, int, double> ewd:existed_weighted_data){
        all_keys.push_back(std::get<0>(ewd));
        all_keys.push_back(std::get<1>(ewd));
    }

    v_int unique_data = all_keys;
    std::vector<int>::iterator key_it2 = std::unique(unique_data.begin(), unique_data.end());
    unique_data.resize(std::distance(unique_data.begin(), key_it2));

    std::sort(unique_data.begin(), unique_data.end());

    for (int uident:unique_data){
        std::unordered_map<int, double> uid_data;
        for (std::tuple<int, int, double> ewd:existed_weighted_data){
            if (std::get<0>(ewd) == uident){
                if (!(uid_data.contains(std::get<1>(ewd)))){
                    uid_data.emplace(std::get<1>(ewd), std::get<2>(ewd));
                }
            }
        }
        if (uid_data.size() > 0){
            weighted_graph_data.emplace(uident, uid_data);
        }
    }

}


std::unordered_map<int, double> transmodel::Graph_DJ::djikstra_to_all(int start){

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<int, int>>> frontier;
    std::pair<double, int> start_pair = std::make_pair(static_cast<double>(0), start);
    frontier.push(start_pair);

    std::unordered_map<int, int> came_from;
    std::unordered_map<int, double> cost_of_travel;

    came_from.emplace(start, -1);
    cost_of_travel.emplace(start, static_cast<double>(0));


    if (!(weighted_graph_data.contains(start))){
        goto no_point;
    }

    while(!(frontier.empty())){
        std::pair<double, int> current_n = frontier.top();
        frontier.pop();

        if ((unweighted_graph_data.contains(current_n.second)) && (cost_of_travel.contains(current_n.second))){
            
            std::unordered_map<int, int> neighbors = unweighted_graph_data.at(current_n.second);
        
            for (auto &[k, v]:neighbors){
                double new_cost = cost_of_travel.at(current_n.second) + v;
                if (!(cost_of_travel.contains(k))){

                    cost_of_travel.emplace(k, new_cost);
                    std::pair<double, int> newpriority = std::make_pair(new_cost, k);
                    frontier.push(newpriority);
                    came_from.emplace(k, current_n.second);

                } else {
                    if (new_cost < cost_of_travel.at(k)){

                        cost_of_travel.at(k) = new_cost;
                        std::pair<double, int> newpriority = std::make_pair(new_cost, k);
                        frontier.push(newpriority);
                        came_from.at(k) = current_n.second;

                    }
                }
            }
        }
        
    }

    no_point: {}

    return cost_of_travel;
}


template<typename cost_type>
void transmodel::Graph_DJ::djikstra_search(v_int& path_between, cost_type& total_cost, int start, int goal){
    v_int internal_path;

    std::priority_queue<std::pair<cost_type, int>, std::vector<std::pair<cost_type, int>>, std::greater<std::pair<int, int>>> frontier;
    std::pair<cost_type, int> start_pair = std::make_pair(0, start);
    frontier.push(start_pair);

    std::unordered_map<int, int> came_from;
    std::unordered_map<int, int> cost_of_travel;

    came_from.emplace(start, -1);
    cost_of_travel.emplace(start, 0);

    int identifyer = 0;

    if (!(unweighted_graph_data.contains(start))){
        goto no_point;
    }

    if (!(unweighted_graph_data.contains(goal))){
        goto no_point;
    }

    while(!(frontier.empty())){
        std::pair<cost_type, int> current_n = frontier.top();
        frontier.pop();

        if (current_n.second == goal){
            identifyer = 1;
            break;
        }

        if ((unweighted_graph_data.contains(current_n.second)) && (cost_of_travel.contains(current_n.second))){
            
            std::unordered_map<int, int> neighbors = unweighted_graph_data.at(current_n.second);
        
            for (auto &[k, v]:neighbors){
                cost_type new_cost = cost_of_travel.at(current_n.second) + v;
                if (!(cost_of_travel.contains(k))){

                    cost_of_travel.emplace(k, new_cost);
                    std::pair<cost_type, int> newpriority = std::make_pair(new_cost, k);
                    frontier.push(newpriority);
                    came_from.emplace(k, current_n.second);

                } else {
                    if (new_cost < cost_of_travel.at(k)){

                        // std::cout << "Place of death" << std::endl;

                        cost_of_travel.at(k) = new_cost;
                        std::pair<cost_type, int> newpriority = std::make_pair(new_cost, k);
                        frontier.push(newpriority);
                        came_from.at(k) = current_n.second;

                    }
                }
            }
        }
        
    }

    no_point: {}

    if (identifyer == 0){
        total_cost = 0;
        return;
    } else {
        // ------------------------------------------------------- CHECK PROBLEM PLACE START
        int node = goal;
        int new_node;
        path_between.push_back(node);

        while (node != start){
            new_node = came_from.at(node);
            path_between.push_back(new_node);
            node = new_node;
        }

        path_between.push_back(start);
        total_cost = cost_of_travel.at(goal);
        std::reverse(path_between.begin(), path_between.end());
        // ------------------------------------------------------- CHECK PROBLEM PLACE END

        return;
    }
}


void transmodel::Settlement_area::create_settlement(std::unique_ptr<cl::Cl_bild>& cl_bd, std::unique_ptr<io::Genplan_data>& gdata){

    double divider = static_cast<double>(gdata->workforce_distrib.at("total_workers")) / static_cast<double>(gdata->workforce_distrib.at("population"));
    for (int i = 0; i < cl_bd->point_vector.size(); i++){
        if (cl_bd->population.at(i) > static_cast<double>(0)) {
            double pop = cl_bd->population.at(i) * divider;
            point_vector.push_back(cl_bd->point_vector.at(i));
            population.push_back(pop);
        }
    }

    for (int i = 0; i < point_vector.size(); i++){
        set_id_vec.push_back(i);
    }
}


void transmodel::Working_places::create_wp(std::unique_ptr<cl::Cl_bild>& cl_bd){

    for (int i = 0; i < cl_bd->point_vector.size(); i++){
        if (cl_bd->workplaces.at(i) > 0){
            point_vector.push_back(cl_bd->point_vector.at(i));
            workforce.push_back(cl_bd->workplaces.at(i));
        }
    }

    for (int i = 0; i < point_vector.size(); i++){
        wp_id_vec.push_back(i);
    }

}


void transmodel::Working_places::pedestrian_choice(std::unique_ptr<Settlement_area>& settlement, int thread_num){

    settlement->ped_places.assign(settlement->point_vector.size(), 0);

    std::vector<std::tuple<int, int>> splitted_i;

    ssize_t total_size = settlement->point_vector.size();

    int onechunk = total_size/thread_num;
    int prev_value = 0;
    int next_value = onechunk;
    if (thread_num == 1){
        std::tuple<int, int> splitter = std::make_tuple(prev_value, static_cast<int>(total_size));
        splitted_i.push_back(splitter);
    } else {
        for (int i=0; i<thread_num; i++){
            if (i != thread_num - 1){
                std::tuple<int, int> splitter = std::make_tuple(prev_value, next_value);
                splitted_i.push_back(splitter);
                prev_value += onechunk;
                next_value += onechunk;
            } else {
                std::tuple<int, int> splitter = std::make_tuple(prev_value, static_cast<int>(total_size));
                splitted_i.push_back(splitter);
            }
        }
    }

    auto pedchoise_multi = [this](std::unique_ptr<Settlement_area>& settlement, std::mutex& multi_mt, std::tuple<int, int>& delimvals){

        for (int i = std::get<0>(delimvals); i != std::get<1>(delimvals); i++){
        OGRGeometry* buffer_zone = NULL;
        buffer_zone = point_vector.at(i).Buffer(1000);

        for (int j = 0; j < point_vector.size(); j++){
            if (buffer_zone->Intersects(&point_vector.at(j))){
                multi_mt.lock();
                settlement->ped_places.at(i) += workforce.at(j);
                multi_mt.unlock();
            }
        }


        delete buffer_zone;
        }
    };

    std::mutex multi_mt;

    std::vector<std::thread> pch_thread(thread_num);

    for (int i = 0; i < thread_num; i++){
        pch_thread[i] = std::thread(pedchoise_multi, std::ref(settlement), std::ref(multi_mt), std::ref(splitted_i.at(i)));
    }

    for (auto& ptd:pch_thread){
        ptd.join();
    }

    for (int i = 0; i < settlement->ped_places.size(); i++){
        settlement->ped_avaliable.try_emplace(i, settlement->ped_places.at(i));
    }
    
}


void transmodel::PT_model::make_pt_model(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){

    // make graph index
    std::unordered_map<int, int> existed_keys;

    v_int allstops_allroutes;
    for (v_int stop_vec:gtfs_lines->stops_in_routes){
        for (int stop_i:stop_vec){
            allstops_allroutes.push_back(stop_i);
        }
    }

    std::unordered_set<int> unique_stops_uset;
    v_int unique_stops;
    for (int sww:allstops_allroutes){
        unique_stops_uset.insert(sww);
    }

    for (int usw:unique_stops_uset){
        unique_stops.push_back(usw);
    }

    // v_int unique_stops = allstops_allroutes;
    // std::vector<int>::iterator key_it = std::unique(unique_stops.begin(), unique_stops.end());
    // unique_stops.resize(std::distance(unique_stops.begin(), key_it));
    for (unsigned long i = 0; i < unique_stops.size(); i++){
        existed_keys.emplace(unique_stops.at(i), i);
    }

    // make graph structure
    std::vector<std::tuple<unsigned long, unsigned long>> existed_edges;
    v_ulong edge_vector;
    v_doub weight_time_vector;

    for (int j = 0; j < gtfs_lines->stops_in_routes.size(); j++){
        for (int i = 0; i < gtfs_lines->stops_in_routes.at(j).size() - 1; i++){
            unsigned long key1 = existed_keys.at(gtfs_lines->stops_in_routes.at(j).at(i));
            unsigned long key2 = existed_keys.at(gtfs_lines->stops_in_routes.at(j).at(i+1));
            std::tuple<unsigned long, unsigned long> current_edge = std::make_tuple(key1, key2);
            if (!(std::find(existed_edges.begin(), existed_edges.end(), current_edge) != existed_edges.end())){
                existed_edges.push_back(current_edge);
                edge_vector.push_back(key1);
                edge_vector.push_back(key2);

                OGRPoint point_a = gtfs_points->id_connect.at(gtfs_lines->stops_in_routes.at(j).at(i));
                OGRPoint point_b = gtfs_points->id_connect.at(gtfs_lines->stops_in_routes.at(j).at(i+1));
                double dist = point_a.Distance(&point_b);
                int trtype = gtfs_lines->type_of_transport.at(j);
                std::string tr_name;
                if (trtype == 4){
                    tr_name = "metro";
                    dist = dist / static_cast<double>(44000);
                } else if (trtype == 0){
                    tr_name = "tram";
                    dist = dist / static_cast<double>(22000);
                } else if (trtype == 3){
                    tr_name = "bus";
                    dist = dist / static_cast<double>(19000);
                } else {
                    tr_name = "bus";
                    dist = dist / static_cast<double>(19000);
                }

                std::tuple<double, std::string> edg = std::make_tuple(dist, tr_name);
                edge_data.emplace(current_edge, edg);
            }
        }
    }

    std::cout << "Size of edges " << edge_data.size() << std::endl;
    std::cout << "Size of stop coords " << gtfs_points->stop_coord.size() << std::endl;
    std::cout << std::endl;


    for (int i = 0; i < gtfs_lines->stops_in_routes.size(); i++){
        if (gtfs_lines->type_of_transport.at(i) == 4){
            for (int j = 0; j < gtfs_lines->stops_in_routes.at(i).size(); j++){
                OGRGeometry* buffer_zone = NULL;
                buffer_zone = gtfs_points->id_connect.at(gtfs_lines->stops_in_routes.at(i).at(j)).Buffer(250);
                for (int f = 0; f < gtfs_points->id_connect.size(); f++){
                    if ((buffer_zone->Intersects(&gtfs_points->stop_coord.at(f))) && (gtfs_lines->stops_in_routes.at(i).at(j) != gtfs_points->stop_id_vec.at(f))){
                        OGRPoint point_a = gtfs_points->id_connect.at(gtfs_lines->stops_in_routes.at(i).at(j));
                        OGRPoint point_b = gtfs_points->stop_coord.at(f);
                        double dist = point_a.Distance(&point_b) / static_cast<double>(5000); 
                        std::string tr_name = "pedestrian";

                        // strict order
                        int key1 = existed_keys.at(gtfs_lines->stops_in_routes.at(i).at(j));
                        int key2 = existed_keys.at(gtfs_points->stop_id_vec.at(f));
                        edge_vector.push_back(static_cast<unsigned long>(key1));
                        edge_vector.push_back(static_cast<unsigned long>(key2));

                        std::tuple<int, int> current_edge = std::make_tuple(key1, key2);
                        std::tuple<double, std::string> edg = std::make_tuple(dist, tr_name);
                        edge_data.emplace(current_edge, edg);

                        // reverse order
                        edge_vector.push_back(static_cast<unsigned long>(key2));
                        edge_vector.push_back(static_cast<unsigned long>(key1));

                        std::tuple<int, int> reverse_current_edge = std::make_tuple(key2, key1);
                        edge_data.emplace(reverse_current_edge, edg);
                    }
                }
            }
            
        }
    }

    v_ulong unique_id = edge_vector;
    std::vector<unsigned long>::iterator str_it = std::unique(unique_id.begin(), unique_id.end());
    unique_id.resize(std::distance(unique_id.begin(), str_it));

    for (auto const &[k, v]:existed_keys){
        edge_stop_connection.emplace(k, v);
    }
}


void transmodel::Settlement_area::make_transport_connections(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){
    

    for (int i = 0; i < set_id_vec.size(); i++){
        OGRGeometry* buffer_zone = NULL;
        buffer_zone = point_vector.at(i).Buffer(600);
        v_int clstop;
        for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
            if (buffer_zone->Intersects(&gtfs_points->stop_coord.at(j))){
                clstop.push_back(gtfs_points->stop_id_vec.at(j));
            }
        }
        delete buffer_zone;

        if (clstop.size() > 0){
            closest_stops.push_back(clstop);
        } else {
            v_doub distance_vec;
            v_int stops_near;
            for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
                double distance = point_vector.at(i).Distance(&gtfs_points->stop_coord.at(j));
                distance_vec.push_back(distance);
                stops_near.push_back(gtfs_points->stop_id_vec.at(j));
            }

            v_doub sorted_dist = distance_vec;
            std::sort(distance_vec.begin(), distance_vec.end());

            long index_min1 = std::find(sorted_dist.begin(), sorted_dist.end(), distance_vec.at(0)) - sorted_dist.begin();
            long index_min2 = std::find(sorted_dist.begin(), sorted_dist.end(), distance_vec.at(1)) - sorted_dist.begin();

            clstop.push_back(stops_near.at(index_min1));
            clstop.push_back(stops_near.at(index_min2));
            closest_stops.push_back(clstop);
        }
    }

}


void transmodel::Working_places::make_transport_connections(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){

    // from workplace to stop
    std::unordered_map<int, v_int> wp_to_stops;

    for (int i = 0; i < wp_id_vec.size(); i++){
        OGRGeometry* buffer_zone = NULL;
        buffer_zone = point_vector.at(i).Buffer(600);
        v_int clstop;
        for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
            if (buffer_zone->Intersects(&gtfs_points->stop_coord.at(j))){
                clstop.push_back(gtfs_points->stop_id_vec.at(j));
            }
        }
        delete buffer_zone;

        if (clstop.size() > 0){
            closest_stops.push_back(clstop);
        } else {
            v_doub distance_vec;
            v_int stops_near;
            for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
                double distance = point_vector.at(i).Distance(&gtfs_points->stop_coord.at(j));
                distance_vec.push_back(distance);
                stops_near.push_back(gtfs_points->stop_id_vec.at(j));
            }

            v_doub sorted_dist = distance_vec;
            std::sort(distance_vec.begin(), distance_vec.end());

            long index_min1 = std::find(sorted_dist.begin(), sorted_dist.end(), distance_vec.at(0)) - sorted_dist.begin();
            long index_min2 = std::find(sorted_dist.begin(), sorted_dist.end(), distance_vec.at(1)) - sorted_dist.begin();

            clstop.push_back(stops_near.at(index_min1));
            clstop.push_back(stops_near.at(index_min2));
            closest_stops.push_back(clstop);
        }

        wp_to_stops.try_emplace(wp_id_vec.at(i), clstop);

    }

    // vice versa
    v_int all_stops_with_wp;
    v_int stops_with_wp;

    for (auto &[k, val]:wp_to_stops){
        for (int v:val){
            all_stops_with_wp.push_back(v);
        }
    }

    std::unordered_set<int> unique_stops_wp;
    for (int sww:all_stops_with_wp){
        unique_stops_wp.insert(sww);
    }

    for (int usw:unique_stops_wp){
        stops_with_wp.push_back(usw);
    }

    // stop id -> workpalces nearby
    for (int swwp:stops_with_wp){
        v_int stop_wpid;
        for (auto &[work, stopvec]:wp_to_stops){
            if (std::find(stopvec.begin(), stopvec.end(), swwp) != stopvec.end()){
                stop_wpid.push_back(work);
            }
        }
        closest_wp.try_emplace(swwp, stop_wpid);
    }

    // node id -> workpalces nearby
    for (auto &[stid, wpvec]:closest_wp){
        if (ptmodel->edge_stop_connection.contains(stid)){
            ptmodel->node_closest_wp.emplace(ptmodel->edge_stop_connection.at(stid), wpvec);
        }
    }

}


void transmodel::Working_places::make_possible_wp(std::unique_ptr<PT_model>& ptmodel){

    v_int stop_vec;
    auto key_selector = [](auto pair){return pair.first;};
    std::transform(ptmodel->edge_stop_connection.begin(), ptmodel->edge_stop_connection.end(), std::back_inserter(stop_vec), key_selector);

    for (int stop_i:stop_vec){
        v_int all_possible;
        for (v_int ctss:closest_stops){
            for (int st:ctss){
                if (std::find(all_possible.begin(), all_possible.end(), st) == all_possible.end()){
                    all_possible.push_back(st);
                }
            }
        }
        ptmodel->stop_workplaces_poss.emplace(stop_i, all_possible);
    }
}


void transmodel::Working_places::possible_work_stop(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){

    for (auto edge_connect:ptmodel->edge_stop_connection){
        if (gtfs_points->id_connect.contains(edge_connect.first)){

            ptmodel->stop_wp_connection.emplace(edge_connect.first, 0);
            ptmodel->node_wp_connection.emplace(edge_connect.second, 0);

            OGRPoint edge_point = gtfs_points->id_connect.at(edge_connect.first);
            OGRGeometry* buffer_zone = NULL;
            buffer_zone = edge_point.Buffer(600);

            v_int wp_nearby;

            for (int i = 0; i < point_vector.size(); i++){
                OGRPoint pvr = point_vector.at(i);
                
                if (buffer_zone->Intersects(&pvr)){
                    ptmodel->stop_wp_connection.at(edge_connect.first) += workforce.at(i);
                    ptmodel->node_wp_connection.at(edge_connect.second) += workforce.at(i);
                    wp_nearby.push_back(wp_id_vec.at(i));
                }
            }

            ptmodel->workplaces_near_stop.try_emplace(edge_connect.second, wp_nearby);
            ptmodel->stop_workplace.try_emplace(edge_connect.first, wp_nearby);
            wp_nearby.clear();
            delete buffer_zone;
        }
        
    }
}


void transmodel::predict_pt_users(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<Settlement_area>& setarea, std::unique_ptr<Working_places>& wplaces,
        std::unique_ptr<gtfs::GTFS_points>& gtfs_points, int thread_num){

    for (int i = 0; i < wplaces->wp_id_vec.size(); i++){
        wplaces->workforce_id.emplace(wplaces->wp_id_vec.at(i), wplaces->workforce.at(i));
    }

    std::vector<std::tuple<int, int>> existed_data;
    for (auto kv:ptmodel->edge_data){
        existed_data.push_back(kv.first);
    }

    Graph_DJ graph_d;
    graph_d.make_graph(existed_data);

    std::unordered_map<int, v_int> house_and_stops; // house id -> closest stops

    for (int i = 0; i < setarea->point_vector.size(); i++){
        OGRPoint edge_point = setarea->point_vector.at(i);
        OGRGeometry* buffer_zone = NULL;
        buffer_zone = edge_point.Buffer(600);

        v_int avaliable_stops;

        for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
            if (buffer_zone->Intersects(&gtfs_points->stop_coord.at(j))){
                avaliable_stops.push_back(gtfs_points->stop_id_vec.at(j));
            }
        }
        delete buffer_zone;

        if (avaliable_stops.size() == 0){
            v_doub all_distance;
            v_int all_id;
            
            for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
                double dist = setarea->point_vector.at(i).Distance(&gtfs_points->stop_coord.at(j));
                all_distance.push_back(dist);
                all_id.push_back(gtfs_points->stop_id_vec.at(j));
            }

            v_doub sorted_dist = all_distance;
            std::sort(sorted_dist.begin(), sorted_dist.end());

            long index_min1 = std::find(all_distance.begin(), all_distance.end(), sorted_dist.at(0)) - all_distance.begin();
            long index_min2 = std::find(all_distance.begin(), all_distance.end(), sorted_dist.at(1)) - all_distance.begin();

            avaliable_stops.push_back(all_id.at(index_min1));
            avaliable_stops.push_back(all_id.at(index_min2));
        }

        house_and_stops.emplace(setarea->set_id_vec.at(i), avaliable_stops); // house id -> stop id
    }

    // Make map with all pairs between nodes
    std::map<std::tuple<int, int>, std::tuple<double, v_int>> pair_between;

    for (auto &[st_id, nd_id]:ptmodel->edge_stop_connection){

        for (auto &[stid2, ndid2]:ptmodel->edge_stop_connection){

            if (nd_id != ndid2){

                std::unique_ptr<v_int> inner_wp_id = std::make_unique<v_int>();
                std::unique_ptr<v_doub> normal_wp = std::make_unique<v_doub>();

                if (ptmodel->node_closest_wp.contains(ndid2)){
                    v_int path;
                    int total_cost;
                    graph_d.djikstra_search(path, total_cost, nd_id, ndid2);

                    if (path.size() > 0){
                        std::unique_ptr<v_doub> seg_length = std::make_unique<v_doub>();
                        std::unique_ptr<v_string> seg_ttype = std::make_unique<v_string>();

                        for (int p = 0; p < path.size() - 1; p++){
                            if (path.at(p) != path.at(p+1)){
                                std::tuple<int, int> curr_path_place = std::make_tuple(path.at(p), path.at(p+1));
                                if (ptmodel->edge_data.contains(curr_path_place)){

                                    std::tuple<double, std::string> segdata = ptmodel->edge_data.at(curr_path_place);
                                    seg_length->push_back(std::get<0>(segdata));
                                    seg_ttype->push_back(std::get<1>(segdata));
                                }
                                
                            }
                        }
                        path.clear();
                        if ((seg_length->size() > 0) && (seg_ttype->size() > 0)){
                            double summ_length = std::reduce(seg_length->begin(), seg_length->end());
                            if ((summ_length > 0.1) && (summ_length < static_cast<double>(1))){
                                std::unique_ptr<v_string> unique_str = std::make_unique<v_string>();
                                std::unique_ptr<std::unordered_set<std::string>> unique_str_set = std::make_unique<std::unordered_set<std::string>>();
                                for (std::string ste:*seg_ttype){
                                    unique_str_set->insert(ste);
                                    }

                                for (std::string uss:*unique_str_set){
                                    unique_str->push_back(uss);
                                    }
                                int denom = 0;

                                if (std::find(unique_str->begin(), unique_str->end(), std::string("pedestrian")) != unique_str->end()){
                                    denom = (unique_str->size() - 1) * 30;
                                } else {
                                    denom = unique_str->size() * 30;
                                }
                                
                                
                                double total_wp = 0;
                                for (int w_id:ptmodel->node_closest_wp.at(ndid2)){
                                    if (wplaces->workforce_id.contains(w_id)){
                                        total_wp += wplaces->workforce_id.at(w_id);
                                    }
                                }

                                double denom_wp = total_wp / denom;

                                // std::cout << "DENOM WP: " << denom_wp << std::endl;

                                normal_wp->push_back(denom_wp);

                                for (int wid:ptmodel->node_closest_wp.at(ndid2)){
                                    inner_wp_id->push_back(wid);
                                }

                            }
                            
                        }
                    }
                }

                std::tuple<int, int> key_pair = std::make_tuple(st_id, stid2);
                double normalized_wp;
                if (normal_wp->size() > 0){
                    normalized_wp = std::reduce(normal_wp->begin(), normal_wp->end());
                } else {
                    normalized_wp = static_cast<double>(0);
                }
                
                std::tuple<double, v_int> value_pair = std::make_tuple(normalized_wp, *inner_wp_id);
                pair_between.emplace(key_pair, value_pair);
            }
            
        }
    }


    // NEW VERSION
    std::cout << "BIG LOAD" << std::endl;

    // for (auto [house_id, closest_stops]:house_and_stops){
    for (auto keyval:house_and_stops){

        // std::unique_ptr<v_int> inner_wp_id = std::make_unique<v_int>();

        // v_int inner_wp_id;
        double normalized_wp = 0;

        for (int stopid:keyval.second){
            for (auto &[stidwp, wpid_vec]:ptmodel->stop_workplace){
                std::tuple<int, int> first_pair = std::make_tuple(stopid, stidwp);
                if (pair_between.contains(first_pair)){
                    std::tuple<double, v_int> precalculated_data = pair_between.at(first_pair);
                    if ((std::get<0>(precalculated_data) > 0) && (std::get<1>(precalculated_data).size() > 0)){

                        normalized_wp += std::get<0>(precalculated_data);

                        // for (int wid:std::get<1>(precalculated_data)){
                        //     // inner_wp_id.push_back(wid);
                        // }
                    }
                    
                }
            }
        }

        setarea->pt_avaliable.emplace(keyval.first, normalized_wp); // house id -> normalized population

        // setarea->house_and_wp.emplace(keyval.first, inner_wp_id); // house id -> stops (stop id)

        // inner_wp_id.clear();
    }
    
}


void transmodel::PT_model::total_route_wp(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines){

    for (int i = 0; i < gtfs_lines->stops_in_routes.size(); i++){
        std::string route_name = gtfs_lines->trip_id_vec.at(i);
        v_int stops_inside = gtfs_lines->stops_in_routes.at(i);
        int totalwp = 0;

        for (int stop_id:stops_inside){
            if (stop_wp_connection.contains(stop_id)){
                totalwp += stop_wp_connection.at(stop_id);
            }
        }

        route_wp.emplace(route_name, totalwp);
    }
}


void transmodel::PT_model::total_avaliable(std::unique_ptr<gtfs::GTFS_lines>& gtfs_lines, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){

    for (int stopid:gtfs_points->stop_id_vec){
        total_wp_avaliable.emplace(stopid, 0);

        v_string route_names;

        for (int j = 0; j < gtfs_lines->stops_in_routes.size(); j++){
            std::string route_name = gtfs_lines->trip_id_vec.at(j);
            v_int route_stops = gtfs_lines->stops_in_routes.at(j);

            if (std::find(route_stops.begin(), route_stops.end(), stopid) != route_stops.end()){
                route_names.push_back(route_name);
            }
        }

        if (route_names.size() > 0) {
            for (std::string rn:route_names){
                if (route_wp.contains(rn)){
                    total_wp_avaliable.at(stopid) += route_wp.at(rn);
                }
            }
            
        }

    }
}


void transmodel::Settlement_area::stop_selection(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){
    
    for (auto keyval:ptmodel->stop_wp_connection){
        ptmodel->start_pressure.emplace(keyval.first, static_cast<double>(0));
    }

    for (int i = 0; i < point_vector.size(); i++){
        OGRPoint edge_point = point_vector.at(i);
        OGRGeometry* buffer_zone = NULL;
        buffer_zone = edge_point.Buffer(600);

        v_int avaliable_stops;
        v_doub dist_G;

        for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
            if (buffer_zone->Intersects(&gtfs_points->stop_coord.at(j))){
                avaliable_stops.push_back(gtfs_points->stop_id_vec.at(j));
            }
        }
        delete buffer_zone;

        if (avaliable_stops.size() == 0){
            v_doub all_distance;
            v_int all_id;
            
            for (int j = 0; j < gtfs_points->stop_coord.size(); j++){
                double dist = point_vector.at(i).Distance(&gtfs_points->stop_coord.at(j));
                all_distance.push_back(dist);
                all_id.push_back(gtfs_points->stop_id_vec.at(j));
            }

            v_doub sorted_dist = all_distance;
            std::sort(sorted_dist.begin(), sorted_dist.end());

            long index_min1 = std::find(all_distance.begin(), all_distance.end(), sorted_dist.at(0)) - all_distance.begin();
            long index_min2 = std::find(all_distance.begin(), all_distance.end(), sorted_dist.at(1)) - all_distance.begin();

            dist_G.push_back(sorted_dist.at(0));
            dist_G.push_back(sorted_dist.at(1));

            avaliable_stops.push_back(all_id.at(index_min1));
            avaliable_stops.push_back(all_id.at(index_min2));
        }

        for (int astop:avaliable_stops){
            if (gtfs_points->id_connect.contains(astop)){
                OGRPoint current_stop = gtfs_points->id_connect.at(astop);
                double av_dist = edge_point.Distance(&current_stop);
                dist_G.push_back(av_dist);
            } else {
                dist_G.push_back(1000000000);
            }
        }

        // math model

        v_doub mmod_current;

        for (int k = 0; k < avaliable_stops.size(); k++){
            if (ptmodel->stop_wp_connection.contains(avaliable_stops.at(k))){
                int stop_c = avaliable_stops.at(k);
                int av_wp = ptmodel->stop_wp_connection.at(stop_c);
                double av_pop = pt_users.at(i);

                double current_value = (static_cast<double>(av_wp) * av_pop) / pow(dist_G.at(k), 2);
                mmod_current.push_back(current_value);
            } else {
                mmod_current.push_back(static_cast<double>(0));
            }
        }

        double mmod_total = std::reduce(mmod_current.begin(), mmod_current.end()); // start_pressure
        
        for (int k = 0; k < avaliable_stops.size(); k++){
            double spr = mmod_current.at(k) / mmod_total * pt_users.at(i);
            if (ptmodel->start_pressure.contains(avaliable_stops.at(k))){
                ptmodel->start_pressure.at(avaliable_stops.at(k)) += spr / static_cast<double>(3);
            }
            
        }

        for (auto keyval:ptmodel->start_pressure){
            if (isnan(keyval.second)){
                ptmodel->start_pressure.at(keyval.first) = 0;
            }
        }
    }
}


void transmodel::PT_model::model_solve(){
    
    std::vector<std::tuple<int, int>> existed_data;
    for (auto kv:edge_data){
        existed_data.push_back(kv.first);
    }

    for (auto keyval:start_pressure){
        if (edge_stop_connection.contains(keyval.first)){
            solved_model.emplace(edge_stop_connection.at(keyval.first), static_cast<double>(0));
        }
        
    }

    Graph_DJ graph_d;
    graph_d.make_graph(existed_data);

    for (auto &[key, val]:start_pressure){
        if (edge_stop_connection.contains(key)){
            node_population.emplace(edge_stop_connection.at(key), val);
        }
    }

    for (auto &[s, n]:edge_stop_connection){
        nodeid_pressure.emplace(n, static_cast<double>(0));
    }

    v_int out_state;
    for (auto &[kpop, vpop]:node_population){

        v_int avaliable_stops;
        v_doub dist_G;

        v_int node_ids;
        v_doub all_gmodel;

        std::unordered_map<int, v_int> paths_2;

        for (auto &[kwp, vwp]:node_wp_connection){
            
            v_int path;
            v_int upd_path;
            int total_cost;
            graph_d.djikstra_search(path, total_cost, kpop, kwp);

            if (path.size() > 0){

                v_doub seg_length;
                v_string seg_ttype;

                for (int p = 0; p < path.size() - 1; p++){
                    if (path.at(p) != path.at(p+1)){
                        std::tuple<int, int> curr_path_place = std::make_tuple(path.at(p), path.at(p+1));
                        if (edge_data.contains(curr_path_place)){

                            std::tuple<double, std::string> segdata = edge_data.at(curr_path_place);
                            seg_length.push_back(std::get<0>(segdata));
                            seg_ttype.push_back(std::get<1>(segdata));
                        }
                        
                    }
                }

                if ((seg_length.size() > 0) && (seg_ttype.size() > 0)){
                    double summ_length = std::reduce(seg_length.begin(), seg_length.end());
                    if ((summ_length > 0.1) && (summ_length < static_cast<double>(1))){
                        v_string unique_str;
                        std::unordered_set<std::string> unique_str_set;
                        for (std::string ste:seg_ttype){
                            unique_str_set.insert(ste);
                        }

                        for (std::string uss:unique_str_set){
                            unique_str.push_back(uss);
                        }
                        int denom;

                        if (std::find(unique_str.begin(), unique_str.end(), std::string("pedestrian")) != unique_str.end()){
                            denom = (unique_str.size() - 1) * 30;
                        } else {
                            denom = unique_str.size() * 30;
                        }
                        
                        double denom_length = summ_length * static_cast<double>(denom);
                        node_ids.push_back(kwp);
                        double gmodel = (vpop * vwp) / pow(denom, 2);
                        all_gmodel.push_back(gmodel);

                        for (int p = 0; p < path.size() - 1; p++){
                            upd_path.push_back(path.at(p));
                        }

                        paths_2.emplace(kwp, upd_path);

                    }
                    
                }
            }
        }

        double gmodel_total = std::reduce(all_gmodel.begin(), all_gmodel.end());

        for (int node = 0; node < node_ids.size(); node++){
            double current_state = all_gmodel.at(node) / gmodel_total * vpop;
            v_int vector_path = paths_2.at(node_ids.at(node));
            for (int vp:vector_path){
                if (nodeid_pressure.contains(vp)){
                    if (!(isnan(current_state))){
                        nodeid_pressure.at(vp) += current_state;
                    }
                }
            }
            if (!(isnan(current_state)) && (solved_model.contains(node_ids.at(node)))){
                solved_model.at(node_ids.at(node)) += current_state;
            }
        }
    }

}


// CAR PART

void transmodel::Car_model::create_model(std::map<std::tuple<int, int>, std::tuple<std::tuple<int, int>, std::tuple<int, int>, double>> data){

    car_data = data;
    for (auto dt:data){
        std::tuple<int, int> nodeint = dt.first;
        std::tuple<std::tuple<int, int>, std::tuple<int, int>, double> data_dt = dt.second;

        car_node_data.try_emplace(std::get<0>(nodeint), std::get<0>(data_dt));
        car_node_data.try_emplace(std::get<1>(nodeint), std::get<1>(data_dt));
    }
}


void transmodel::Settlement_area::car_settlement(std::unique_ptr<Car_model>& carmodel){

    for (OGRPoint sa_point:point_vector){
        std::tuple<int, int> intgeom = std::make_tuple((int)round(sa_point.getX()), (int)round(sa_point.getY()));
        tuple_view.push_back(intgeom);
    }

    for (int n = 0; n < tuple_view.size(); n++){

        std::map<double, int, std::less<double>> mindist;
        for (auto &[mapkey, mapval]:carmodel->car_node_data){
            double distance = sqrt(pow((std::get<0>(tuple_view.at(n)) - std::get<0>(mapval)), 2) + pow((std::get<1>(tuple_view.at(n)) - std::get<1>(mapval)), 2));
            mindist.emplace(distance, mapkey);
        }

        carmodel->settl_node.try_emplace(n, mindist.begin()->second);
    }
}


void transmodel::Working_places::workplace_settlement(std::unique_ptr<Car_model>& carmodel){

    for (OGRPoint sa_point:point_vector){
        std::tuple<int, int> intgeom = std::make_tuple((int)round(sa_point.getX()), (int)round(sa_point.getY()));
        tuple_view.push_back(intgeom);
    }

    for (int n = 0; n < tuple_view.size(); n++){

        std::map<double, int, std::less<double>> mindist;
        for (auto &[mapkey, mapval]:carmodel->car_node_data){
            double distance = sqrt(pow((std::get<0>(tuple_view.at(n)) - std::get<0>(mapval)), 2) + pow((std::get<1>(tuple_view.at(n)) - std::get<1>(mapval)), 2));
            mindist.emplace(distance, mapkey);
        }

        carmodel->working_node.try_emplace(n, mindist.begin()->second);
    }
}


void transmodel::car_users(std::unique_ptr<Settlement_area>& setarea, std::unique_ptr<Car_model>& carmodel, std::unique_ptr<Working_places>& wplaces, int thread_num){
// 8,5 литров на 100 км 56 рублей литр -> 4.76 руб за км

    std::vector<std::tuple<int, int, double>> existed_data;
    for (auto [k, v]:carmodel->car_data){
        std::tuple<int, int, double> ex_weigth = std::make_tuple(std::get<0>(k), std::get<1>(k), std::get<2>(v));
        existed_data.push_back(ex_weigth);
    }

    Graph_DJ graph_d;
    graph_d.make_weighted_graph(existed_data);

    ssize_t total_size = carmodel->settl_node.size();
    std::vector<std::tuple<int, int>> splitted_i;
    int onechunk = total_size/thread_num;
    int prev_value = 0;
    int next_value = onechunk;
    if (thread_num == 1){
        std::tuple<int, int> splitter = std::make_tuple(prev_value, static_cast<int>(total_size));
        splitted_i.push_back(splitter);
    } else {
        for (int i = 0; i < thread_num; i++){
            if (i != thread_num - 1){
                std::tuple<int, int> splitter = std::make_tuple(prev_value, next_value);
                splitted_i.push_back(splitter);
                prev_value += onechunk;
                next_value += onechunk;
            } else {
                std::tuple<int, int> splitter = std::make_tuple(prev_value, static_cast<int>(total_size));
                splitted_i.push_back(splitter);
            }
        }
    }

    std::vector<std::unordered_map<int, int>> vec_local_settlnode(thread_num); // settlement id -> car node

    int sep = 0;
    for (auto &[k, v]:carmodel->settl_node){
        for (int isp = 0; isp < splitted_i.size(); isp++){
            if ((sep >= std::get<0>(splitted_i.at(isp))) && (sep < std::get<1>(splitted_i.at(isp)))){
                vec_local_settlnode.at(isp).emplace(k, v);
            }
        }
        sep++;

    }

    std::mutex car_mutex;


    auto car_users_find = [](std::unique_ptr<Settlement_area>& setarea, std::unique_ptr<Car_model>& carmodel, 
                            std::unordered_map<int, int>& local_settlnode, Graph_DJ& graph_d, std::mutex& car_mutex, long& car_users_proc,
                            std::unique_ptr<Working_places>& wplaces){
        
        for (auto &[sl_key, sl_val]:local_settlnode){

            car_users_proc++;

            v_int car_workid;
            v_doub car_workers;

            std::unordered_map<int, double> matrix_to_all = graph_d.djikstra_to_all(sl_val);
            
            for (auto &[wg_key, wg_val]:carmodel->working_node){

                if (matrix_to_all.contains(wg_val)){
                    if ((matrix_to_all.at(wg_val) > 0.1) && (matrix_to_all.at(wg_val) < 1)){

                        double denom_length = matrix_to_all.at(wg_val) * static_cast<double>(24) * static_cast<double>(4.76);

                        if ((wplaces->workforce_id.contains(wg_key)) && (denom_length > 0)){
                            double nomenee_wp = wplaces->workforce_id.at(wg_key) / denom_length;
                            car_workers.push_back(nomenee_wp);
                            car_workid.push_back(wg_key);
                        }
                        
                    }
                }

            }

            car_mutex.lock();
            if (car_workers.size() > 0){
                double allcars = std::reduce(car_workers.begin(), car_workers.end());
                setarea->car_avaliable.emplace(sl_key, allcars);
            } else {
                setarea->car_avaliable.emplace(sl_key, static_cast<double>(0));
            }

            setarea->road_house_wp.emplace(sl_key, car_workid);
            car_mutex.unlock();
            
        }

    };

    ssize_t total_num = carmodel->settl_node.size();
    std::vector<long> processing_time;
    processing_time.assign(thread_num, 0);

    std::vector<std::thread> car_thread(thread_num);
    for (int i = 0; i < thread_num; i++){
        car_thread[i] = std::thread(car_users_find, std::ref(setarea), std::ref(carmodel), std::ref(vec_local_settlnode.at(i)),
                            std::ref(graph_d), std::ref(car_mutex), std::ref(processing_time.at(i)), std::ref(wplaces));
    }

    for (auto& ctd:car_thread){
        ctd.join();
    }
}


void transmodel::Settlement_area::pt_users_prediction(){

    for (int i = 0; i < set_id_vec.size(); i++){

        v_int total_places;

        double pt_users_current = static_cast<double>(0);
        double car_users_current = static_cast<double>(0);
        double ped_users_current = static_cast<double>(0);

        v_int unique_places;

        if ((pt_avaliable.contains(i)) && (car_avaliable.contains(set_id_vec.at(i)))){
            pt_users_current = pt_avaliable.at(set_id_vec.at(i));
            car_users_current = car_avaliable.at(set_id_vec.at(i));
        }

        if (ped_avaliable.contains(set_id_vec.at(i))){
            ped_users_current = ped_avaliable.at(set_id_vec.at(i));
        }
        
        double all_users = pt_users_current + car_users_current + ped_users_current;
        double public_users = (pt_users_current / all_users) * population.at(i);

        pt_users.push_back(public_users);
    }
}


void transmodel::Result_model::transfer_from_pt(std::unique_ptr<PT_model>& ptmodel, std::unique_ptr<gtfs::GTFS_points>& gtfs_points){

    for (auto &[stopid, nodeid]:ptmodel->edge_stop_connection){
        if (ptmodel->nodeid_pressure.contains(nodeid)){
            stopid_pressure.try_emplace(stopid, ptmodel->nodeid_pressure.at(nodeid));
        }

        if (ptmodel->solved_model.contains(nodeid)){
            exit_model.try_emplace(stopid, ptmodel->solved_model.at(nodeid));
        }

        if (ptmodel->start_pressure.contains(stopid)){
            entrance_model.try_emplace(stopid, ptmodel->start_pressure.at(stopid));
        }
    }

    for (auto &[stopid, pass]:stopid_pressure){
        if (gtfs_points->total_capacity.contains(stopid)){
            if (gtfs_points->total_capacity.at(stopid) > 0){
                double load_value = pass / gtfs_points->total_capacity.at(stopid) * static_cast<double>(100);
                stopid_load.try_emplace(stopid, load_value);
            } else {
                stopid_load.try_emplace(stopid, static_cast<double>(0));
            }
            
        }
    }

    for (auto &[stopid, nodeid]:ptmodel->edge_stop_connection){
        
        if (gtfs_points->id_connect.contains(stopid)){

            double passenger_load = static_cast<double>(0);
            double exits = static_cast<double>(0);
            double entrances = static_cast<double>(0);

            if (stopid_load.contains(stopid)){
                passenger_load = stopid_load.at(stopid);
            }

            if (exit_model.contains(stopid)){
                exits = exit_model.at(stopid);
            }

            if (entrance_model.contains(stopid)){
                entrances = entrance_model.at(stopid);
            }

            std::tuple<OGRPoint, double, double, double> sd = std::make_tuple(gtfs_points->id_connect.at(stopid), passenger_load, exits, entrances);
            model_for_save.push_back(sd);
        }
    }
}

void transmodel::Result_model::save_model_data(std::string savepath, OGRSpatialReference srs_d){

    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(savepath.data(), 0, 0, 0, GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbPoint, NULL);

    OGRFieldDefn load_field("Load", OFTReal);
    OGRFieldDefn exits_field("Exits", OFTReal);
    OGRFieldDefn entrance_field("Entrances", OFTReal);

    feature_layer->CreateField(&load_field);
    feature_layer->CreateField(&exits_field);
    feature_layer->CreateField(&entrance_field);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < model_for_save.size(); i++){
        OGRFeature *point_feature;
        point_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        point_feature->SetField("Load", std::get<1>(model_for_save.at(i)));
        point_feature->SetField("Exits", std::get<2>(model_for_save.at(i)));
        point_feature->SetField("Entrances", std::get<3>(model_for_save.at(i)));

        point_feature->SetGeometry(&std::get<0>(model_for_save.at(i)));
        feature_layer->CreateFeature(point_feature);
        OGRFeature::DestroyFeature(point_feature);
    }
    GDALClose(dataset);
}


void transmodel::compare_model(std::unique_ptr<Result_model>& newmodel, std::unique_ptr<Result_model>& oldmodel, std::string spath, OGRSpatialReference srs_d){

    std::vector<std::tuple<OGRPoint, double, double, double, double, double>> compare_model_data; // passangers, exits, entrances prev load , future load
    for (auto prepdata:newmodel->model_for_save){
        for (auto olddata:oldmodel->model_for_save){
            if (std::get<0>(prepdata) == std::get<0>(olddata)){
                std::tuple<OGRPoint, double, double, double, double, double> com_tuple;
                double chpassengers = std::get<1>(prepdata) - std::get<1>(olddata);
                double chexits = std::get<2>(prepdata) - std::get<2>(olddata);
                double chentrance = std::get<3>(prepdata) - std::get<3>(olddata);
                double prevload = std::get<1>(olddata);
                double future_load = std::get<1>(prepdata);
                com_tuple = std::make_tuple(std::get<0>(prepdata), chpassengers, chexits, chentrance, prevload, future_load);
                compare_model_data.push_back(com_tuple);
            }
        }
    }

    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(spath.data(), 0, 0, 0, GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbPoint, NULL);

    OGRFieldDefn load_field("Change_ld", OFTReal);
    OGRFieldDefn exits_field("Change_ex", OFTReal);
    OGRFieldDefn entrance_field("Change_en", OFTReal);
    OGRFieldDefn curr_load_field("CurrLoad", OFTReal);
    OGRFieldDefn fut_load_field("FutLoad", OFTReal);

    feature_layer->CreateField(&load_field);
    feature_layer->CreateField(&exits_field);
    feature_layer->CreateField(&entrance_field);
    feature_layer->CreateField(&curr_load_field);
    feature_layer->CreateField(&fut_load_field);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < compare_model_data.size(); i++){
        OGRFeature *point_feature;
        point_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        point_feature->SetField("Change_ld", std::get<1>(compare_model_data.at(i)));
        point_feature->SetField("Change_ex", std::get<2>(compare_model_data.at(i)));
        point_feature->SetField("Change_en", std::get<3>(compare_model_data.at(i)));
        point_feature->SetField("CurrLoad", std::get<4>(compare_model_data.at(i)));
        point_feature->SetField("FutLoad", std::get<5>(compare_model_data.at(i)));

        point_feature->SetGeometry(&std::get<0>(compare_model_data.at(i)));
        feature_layer->CreateFeature(point_feature);
        OGRFeature::DestroyFeature(point_feature);
    }
    GDALClose(dataset);
    
}