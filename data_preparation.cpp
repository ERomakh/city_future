#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <numeric>
#include <thread>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

#include "data_preparation.hpp"

void preparation::Roads::change_topolgy(std::unique_ptr<io::Osm_roads>& osm_r){
    std::vector<std::string> links = {"motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"};

    int barWidth = 100;
    float progress = 0.0;

    linestring lstr_first;
    ssize_t vecsize = osm_r->osm_id_vec.size();
    long alternative_vecsize = osm_r->osm_id_vec.size();
    for (ssize_t k = 0; k != vecsize; k++){

        label_start: {}
        long long first_timestamp = io::time_seconds();

        progress += static_cast<float>(1) / static_cast<float>(vecsize);
        int pos = barWidth * progress;
        std::cout << "Обновление топологии: " << int(progress * 100.0) << " %\r" << std::flush;

        if(!(std::find(links.begin(), links.end(), osm_r->fclass_vec.at(k)) != links.end())){
            auto spat_index_k = osm_r->spatial_index.at(k);
            std::vector<long> already_passed;
            for (ssize_t i = 0; i != osm_r->fclass_vec.size(); i++) {
                if(!(std::find(links.begin(), links.end(), osm_r->fclass_vec.at(i)) != links.end())){
                    if (i != k) {
                        auto spat_index_i = osm_r->spatial_index.at(i);
                        if (!(std::find_first_of(spat_index_k.begin(), spat_index_k.end(), spat_index_i.begin(), 
                                spat_index_i.end()) == spat_index_k.end())){
                            linestring seq_one = osm_r->linestr_vec.at(k);
                            lstr_first = seq_one;
                            for (auto sqo:seq_one){
                                linestring seq_two = osm_r->linestr_vec.at(i);
                                for (auto sqt:seq_two){
                                     if ((std::get<0>(sqo) == std::get<0>(sqt)) && (std::get<1>(sqo) == std::get<1>(sqt))){
                                        already_passed.push_back(i);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (already_passed.size() != 0){
                std::unique_ptr<io::Osm_roads> inner_osm_data = std::make_unique<io::Osm_roads>();
                std::unique_ptr<io::Osm_roads> auxiliary_osm_data = std::make_unique<io::Osm_roads>();
                inner_osm_data->osm_id_vec.push_back(osm_r->osm_id_vec.at(k));
                inner_osm_data->fclass_vec.push_back(osm_r->fclass_vec.at(k));
                inner_osm_data->roads_length.push_back(osm_r->roads_length.at(k));
                inner_osm_data->linestr_vec.push_back(osm_r->linestr_vec.at(k));
                for (ssize_t t:already_passed){
                    for (ssize_t inside = 0; inside != inner_osm_data->osm_id_vec.size(); inside ++){
                        linestring seq_one = inner_osm_data->linestr_vec.at(inside);
                        for (auto sqo:seq_one){
                            linestring seq_two = osm_r->linestr_vec.at(t);
                            for (auto sqt:seq_two){
                                if ((std::get<0>(sqo) == std::get<0>(sqt)) && (std::get<1>(sqo) == std::get<1>(sqt))){
                                    if ((sqo != seq_one.front()) && (sqo != seq_one.back())){
                                        std::vector<point> linestr1;
                                        ssize_t lione = 0;
                                        while (seq_one.at(lione) != sqo) {
                                            long long second_timestamp = io::time_seconds();
                                            long long change_12 = second_timestamp - first_timestamp;
                                            if (change_12 >= 10){
                                                osm_id_vec.push_back(osm_r->osm_id_vec.at(k));
                                                fclass_vec.push_back(osm_r->fclass_vec.at(k));
                                                linestr_vec.push_back(osm_r->linestr_vec.at(k));

                                                k++;

                                                if (k != vecsize - 1){
                                                    goto label_start;
                                                } else {
                                                    goto label_exit;
                                                }
                                            }

                                            linestr1.push_back(seq_one.at(lione));
                                            lione++;
                                        }
                                        linestr1.push_back(sqo);
                                        auxiliary_osm_data->linestr_vec.push_back(linestr1);
                                        linestr1.clear();

                                        std::vector<point> linestr2;
                                        ssize_t litwo = seq_one.size() - 1;
                                        while(seq_one.at(litwo) != sqo){
                                            long long third_timestamp = io::time_seconds();

                                            long long change_13 = third_timestamp - first_timestamp;
                                                if (change_13 >= 10){

                                                    osm_id_vec.push_back(osm_r->osm_id_vec.at(k));
                                                    fclass_vec.push_back(osm_r->fclass_vec.at(k));
                                                    linestr_vec.push_back(osm_r->linestr_vec.at(k));

                                                    k++;

                                                    if (k != vecsize - 1){
                                                        goto label_start;
                                                    } else {
                                                        goto label_exit;
                                                    }
                                                }
                                                  

                                            linestr2.push_back(seq_one.at(litwo));
                                            litwo--;
                                        }
                                        linestr2.push_back(sqo);
                                        std::reverse(linestr2.begin(), linestr2.end());
                                        auxiliary_osm_data->linestr_vec.push_back(linestr2);
                                        linestr2.clear();
                                        
                                        for (int l = 0; l < 2; l++){
                                            alternative_vecsize += 1;
                                            auxiliary_osm_data->osm_id_vec.push_back(alternative_vecsize);
                                            auxiliary_osm_data->fclass_vec.push_back(inner_osm_data->fclass_vec.at(inside));
                                            auxiliary_osm_data->linestr_vec.push_back(inner_osm_data->linestr_vec.at(inside));
                                        }
                                        for (ssize_t u = 0; u != inner_osm_data->osm_id_vec.size(); u++){
                                            if (inner_osm_data->linestr_vec.at(u) != seq_one) {
                                                auxiliary_osm_data->osm_id_vec.push_back(inner_osm_data->osm_id_vec.at(u));
                                                auxiliary_osm_data->fclass_vec.push_back(inner_osm_data->fclass_vec.at(u));
                                                auxiliary_osm_data->linestr_vec.push_back(inner_osm_data->linestr_vec.at(u));
                                            }
                                        }

                                        inner_osm_data->osm_id_vec.swap(auxiliary_osm_data->osm_id_vec);
                                        inner_osm_data->fclass_vec.swap(auxiliary_osm_data->fclass_vec);
                                        inner_osm_data->linestr_vec.swap(auxiliary_osm_data->linestr_vec);

                                        auxiliary_osm_data->osm_id_vec.clear();
                                        auxiliary_osm_data->fclass_vec.clear();
                                        auxiliary_osm_data->linestr_vec.clear();
                                    }
                                }
                            }
                        }
                    }
                    

                }
                for (ssize_t transfer = 0; transfer != inner_osm_data->osm_id_vec.size(); transfer++){
                    osm_id_vec.push_back(inner_osm_data->osm_id_vec.at(transfer));
                    fclass_vec.push_back(inner_osm_data->fclass_vec.at(transfer));
                    linestr_vec.push_back(inner_osm_data->linestr_vec.at(transfer));

                }

            } else if (already_passed.size() == 0){
                osm_id_vec.push_back(osm_r->osm_id_vec.at(k));
                fclass_vec.push_back(osm_r->fclass_vec.at(k));
                linestr_vec.push_back(osm_r->linestr_vec.at(k));
            }
            already_passed.clear();
        } else if (std::find(links.begin(), links.end(), osm_r->fclass_vec.at(k)) != links.end()){
                osm_id_vec.push_back(osm_r->osm_id_vec.at(k));
                fclass_vec.push_back(osm_r->fclass_vec.at(k));
                linestr_vec.push_back(osm_r->linestr_vec.at(k));
        }
    }

    label_exit: {}

    for (linestring& lstr:linestr_vec){
        OGRLineString* ogrline = new OGRLineString;
        for (int x = 0; x != lstr.size(); x++){
            OGRPoint* op = new OGRPoint;
            op->setX(std::get<0>(lstr.at(x)));
            op->setY(std::get<1>(lstr.at(x)));
            ogrline->setPoint(x, op);
            delete op;
        }
        ogr_linestr.push_back(*ogrline);
        roads_length.push_back(ogrline->get_Length());
        delete ogrline;
    }
    std::cout << std::endl;
}


void preparation::Graph_view::create_igraph(std::unique_ptr<Roads>& topo_roads){

    auto make_graph_indexes = [this](std::unique_ptr<Roads>& rds){

        std::map<point, unsigned long> indicies;

        unsigned long count_value = 0;
        for (OGRLineString line:rds->ogr_linestr){
            for (OGRPoint opoint:line){
                point tuple_point = std::make_tuple(opoint.getX(), opoint.getY());
                if (auto search = indicies.find(tuple_point); search != indicies.end()){

                } else {
                    indicies[tuple_point] = count_value;
                    graph_geom_connect.emplace(count_value, opoint);
                    vertex_graph.push_back(count_value);
                    count_value ++;
                }
            }
        }
        return indicies;
    };

    auto graph_data = [this](std::unique_ptr<Roads>& rds, std::map<point, unsigned long>& point_index){
        std::tuple<v_ulong, v_doub> idweghts_data;

        v_ulong value_vec;
        v_doub wdata;

        for (OGRLineString ogrlin:rds->ogr_linestr){
            for (int i = 0; i != ogrlin.getNumPoints() - 2; i++){
                point point1 = std::make_tuple(ogrlin.getX(i), ogrlin.getY(i));
                value_vec.push_back(point_index.at(point1));
                point point2 = std::make_tuple(ogrlin.getX(i+1), ogrlin.getY(i+1));
                value_vec.push_back(point_index.at(point2));

                double distance = sqrt(pow((std::get<0>(point1) - std::get<0>(point2)), 2) + pow((std::get<1>(point1) - std::get<1>(point2)), 2));
                wdata.push_back(distance);
            }
        }
        idweghts_data = std::make_tuple(value_vec, wdata);
        return idweghts_data;
    };

    std::map<point, unsigned long> point_index = make_graph_indexes(topo_roads);
    std::tuple<v_ulong, v_doub> created_ind = graph_data(topo_roads, point_index);
    v_ulong vector_index_values = std::get<0>(created_ind);
    graph_weights = std::get<1>(created_ind);
    for (auto gw:graph_weights){
        if (gw <= 0){
            std::cout << gw << std::endl;
        }
    }

    igraph_vector_int_t edges;
    igraph_integer_t* edges_arr = new igraph_integer_t[vector_index_values.size()];
    std::copy(vector_index_values.begin(), vector_index_values.end(), edges_arr);

    igraph_vector_int_view(&edges, edges_arr, vector_index_values.size());
    delete edges_arr;
    igraph_create(&graph, &edges, point_index.size(), IGRAPH_UNDIRECTED);
}


void preparation::Graph_view::calculate_centrality(){

    
    igraph_integer_t vnum = igraph_vcount(&graph);
    igraph_vector_t weight_vec;

    igraph_real_t* edges_w = new igraph_real_t[graph_weights.size()];
    std::copy(graph_weights.begin(), graph_weights.end(), edges_w);
    igraph_vector_view(&weight_vec, edges_w, graph_weights.size());

    delete edges_w;

    auto bc_calc = [this](igraph_integer_t& vert_number, igraph_vector_t& weight_vec){
        igraph_vector_t bc_vector;
        igraph_vector_init(&bc_vector, vert_number);
        igraph_error_t err = igraph_betweenness(&graph, &bc_vector, igraph_vss_all(), IGRAPH_UNDIRECTED, nullptr); // &weight_vec

        for (unsigned long i = 0; i != vert_number; i++){
            bc_nodes.push_back(static_cast<double>(VECTOR(bc_vector)[i]));
        }
    };

    auto cc_calc = [this](igraph_integer_t& vert_number, igraph_vector_t& weight_vec){
        igraph_vector_t cc_vector;
        igraph_vector_init(&cc_vector, vert_number);
        igraph_error_t err = igraph_closeness(&graph, &cc_vector, NULL, NULL, igraph_vss_all(), IGRAPH_ALL, nullptr, TRUE); // &weight_vec

        for (unsigned long i = 0; i != vert_number; i++){
            cc_nodes.push_back(static_cast<double>(VECTOR(cc_vector)[i]));
        }
    };
    
    std::thread bc_thread = std::thread(bc_calc, std::ref(vnum), std::ref(weight_vec));
    std::thread cc_thread = std::thread(cc_calc, std::ref(vnum), std::ref(weight_vec));

    bc_thread.join();
    cc_thread.join();
}


void preparation::Buildings::create_buildings(std::unique_ptr<io::Osm_building>& shp_build){

    for (unsigned long i = 0; i != shp_build->osm_id_vec.size(); i++){
        osm_id_vec.push_back(shp_build->osm_id_vec.at(i));
        building_type_vec.push_back(shp_build->building_type_vec.at(i));
        levels.push_back(shp_build->levels.at(i));
        ogr_poly_vec.push_back(shp_build->wkt_polygon.at(i));
        square_vec.push_back(shp_build->square_vec.at(i));
        bgeom_array.push_back(shp_build->bgeom_array.at(i));

        poly_id_geom.emplace(shp_build->osm_id_vec.at(i), shp_build->wkt_polygon.at(i));
    }
}

std::vector<std::tuple<int, int>> preparation::Buildings::split_data(int thread_number){
    std::vector<std::tuple<int, int>> splitted_i;

    ssize_t total_size = osm_id_vec.size();

    int onechunk = total_size/thread_number;
    int prev_value = 0;
    int next_value = onechunk;
    if (thread_number == 1){
        std::tuple<int, int> splitter = std::make_tuple(prev_value, static_cast<int>(total_size));
        splitted_i.push_back(splitter);
    } else {
        for (int i=0; i<thread_number; i++){
            if (i != thread_number - 1){
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
    return splitted_i;
}

void preparation::Graph_view::make_array(){

    for (auto kvg:graph_geom_connect){
        point pp = std::make_tuple(kvg.second.getX(), kvg.second.getY());
        graph_array_vec.push_back(pp);
    }
}


void preparation::Buildings::transfer_centrality(std::unique_ptr<Graph_view>& graph_roads, int thread_number){

    graph_roads->make_array();

    std::vector<std::tuple<int, int>> spl_i = split_data(thread_number);
    std::mutex centr_mutex;

    bc_buildings.assign(osm_id_vec.size(), 0);
    cc_buildings.assign(osm_id_vec.size(), 0);

    auto transfer_c = [this](std::unique_ptr<Graph_view>& graph_roads, std::tuple<int, int> tupint, std::mutex& centr_mutex){
        for (unsigned long i = std::get<0>(tupint); i != std::get<1>(tupint); i++){

            std::map<double, std::tuple<double, double>, std::less<double>> map_data;
            point p1 = bgeom_array.at(i);

            for (unsigned long j = 0; j != graph_roads->graph_array_vec.size(); j++){

                point p2 = graph_roads->graph_array_vec.at(j);
                double distance = sqrt(pow((std::get<0>(p1) - std::get<0>(p2)), 2) + pow((std::get<1>(p1) - std::get<1>(p2)), 2));
                std::tuple<double, double> tups = std::make_tuple(graph_roads->bc_nodes.at(j), graph_roads->cc_nodes.at(j));
                map_data.try_emplace(distance, tups);
            }

            auto key_value = *map_data.begin();

            double current_bc = graph_roads->bc_nodes.at(std::get<0>(key_value.second));
            double current_cc = graph_roads->cc_nodes.at(std::get<1>(key_value.second));

            centr_mutex.lock();
            if (std::isnan(current_bc)){
                bc_buildings.at(i) = 0;
            } else {
                bc_buildings.at(i) = current_bc;
            }

            
            if (std::isnan(current_cc)){
                cc_buildings.at(i) = 0;
            } else {
                cc_buildings.at(i) = current_cc;
            }
            centr_mutex.unlock();
        }
    };

    std::vector<std::thread> centrality_thread(thread_number);

    for (int i = 0; i < thread_number; i++){
        centrality_thread[i] = std::thread(transfer_c, std::ref(graph_roads), std::ref(spl_i.at(i)), std::ref(centr_mutex));
    }

    for (auto& ctd:centrality_thread){
        ctd.join();
    }
    
}

void preparation::Buildings::find_density(int thread_number){

    std::vector<std::tuple<int, int>> spl_i = split_data(thread_number);
    std::mutex density_mutex;

    relative_density.assign(osm_id_vec.size(), 0);

    auto reldens = [this](std::mutex& density_mutex, std::tuple<int, int> tupint){
        for (int p = std::get<0>(tupint); p != std::get<1>(tupint); p++){

            OGRPoint* centroid_1 = new OGRPoint;
            ogr_poly_vec.at(p).Centroid(centroid_1);
            

            OGRGeometry* buffer_centroid = NULL;
            buffer_centroid = centroid_1->Buffer(300);
            int counter_value = 0;
            
            for (OGRPolygon poly2:ogr_poly_vec){

                if (buffer_centroid->Intersects(&poly2)){
                    counter_value++;
                }
            }

            density_mutex.lock();
            relative_density.at(p) = counter_value;
            density_mutex.unlock();

            delete centroid_1;
            delete buffer_centroid;
        }
    };

    std::vector<std::thread> density_threads(thread_number);

    for (int i = 0; i < thread_number; i++){
        density_threads[i] = std::thread(reldens, std::ref(density_mutex), std::ref(spl_i.at(i)));
    }

    for (auto& dst:density_threads){
        dst.join();
    }
    
}


v_doub preparation::amenity_density(std::unique_ptr<io::Osm_amenities>& amenities, std::unique_ptr<io::Osm_building>& builds){

    v_doub return_vec;
    for (OGRPolygon poly1:builds->wkt_polygon){

        OGRPoint* centroid_1 = new OGRPoint;
        poly1.Centroid(centroid_1);
        OGRGeometry* buffer_centroid = NULL;
        buffer_centroid = centroid_1->Buffer(300);
        int counter_value = 0;

        for (OGRPoint amen:amenities->amenity_point){

            if (buffer_centroid->Intersects(&amen)){
                counter_value++;
            }
        }
        return_vec.push_back(counter_value);

        delete centroid_1;
        delete buffer_centroid;
    }

    return return_vec;
}

void preparation::Buildings::find_to_center(){

    // Single vertex
    double max_amenity = *std::max_element(amenities_density.begin(), amenities_density.end());
    long index_a = std::find(amenities_density.begin(), amenities_density.end(), max_amenity) - amenities_density.begin();
    OGRPolygon first_obj = ogr_poly_vec.at(index_a);

    for (OGRPolygon current_poly:ogr_poly_vec){
        double distance_local = first_obj.Distance(&current_poly);
        distance_to_center.push_back(distance_local);
    }
}

void preparation::Buildings::get_from_genplan(std::unique_ptr<io::Genplan_data>& genplan_d){

    genplan_categ.assign(ogr_poly_vec.size(), std::string("minus"));

    for (unsigned long i = 0; i != ogr_poly_vec.size(); i++){
        for (int j = 0; j != genplan_d->category_genplan.size(); j++){
            if (genplan_d->polygon_data.at(j).Intersects(&ogr_poly_vec.at(i))){
                genplan_categ.at(i) = genplan_d->category_genplan.at(j);
            }
        }
    }

    for (unsigned long i = 0; i != ogr_poly_vec.size(); i++){
        if (genplan_categ.at(i) == std::string("minus")){
            genplan_categ.at(i) = std::string("res_low");
        }
    }
}

void preparation::Buildings::simplify_categ(){

    v_string little_house = {"house", "detached", "semidetached_house", "terrace", "hut", "cabin", "townhouse", "semi"};
    v_string multilevel_house = {"apartments", "residential", "dormitory", "apartment_building", "hostel"};
    v_string commerical = {"religious", "cathedral", "temple", "synagogue", "church", "mosque", "basilica", "chapel", "retail", "workshop", "market",
        "mall", "shop", "stadium", "swiming_pool", "sport_centre", "swimming_pool", "sports_centre", "grandstand", "sports_hall", "pavilion", "stable",
        "commercial", "office", "offices", "government", "works", "community_centre", "bank", "theatre", "palace", "civic", "hotel", "public", "museum",
        "restaurant", "conservatory", "club", "polyclinic", "hospital", "bathhouse", "clinic", "kindergarten", "school", "college", "yes;school", "library",
        "university"};
    v_string factory_vec = {"factory", "storage_tank", "manufacture", "barn", "reservoir", "service", "electricity", "hangar", "tower", "power"};

    for (unsigned long i = 0; i != ogr_poly_vec.size(); i++){
        if (std::find(little_house.begin(), little_house.end(), building_type_vec.at(i)) != little_house.end()){
            symplified_categ.push_back("detached_h");
        } else if (std::find(multilevel_house.begin(), multilevel_house.end(), building_type_vec.at(i)) != multilevel_house.end()){
            symplified_categ.push_back("multi_h");
        } else if (std::find(commerical.begin(), commerical.end(), building_type_vec.at(i)) != commerical.end()){
            symplified_categ.push_back("commerical");
        } else if (std::find(factory_vec.begin(), factory_vec.end(), building_type_vec.at(i)) != factory_vec.end()){
            symplified_categ.push_back("factory");
        } else {
            symplified_categ.push_back("other");
        }
    }
}