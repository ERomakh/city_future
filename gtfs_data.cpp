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
#include <fstream>
#include <filesystem>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

#include "gtfs_data.hpp"

v_string gtfs::split_string(std::string str, std::string delim){

    v_string fullstr;
    int start, end = -1*delim.size();
    do {
        start = end + delim.size();
        end = str.find(delim, start);
        fullstr.push_back(str.substr(start, end - start));
    } while (end != -1);

    return fullstr;
}

void gtfs::GTFS_points::download_gtfs(std::string path_stops, OGRSpatialReference spartef){

    OGRSpatialReference wgs_srs;
    wgs_srs.SetWellKnownGeogCS("WGS84");

    std::filesystem::path stops_path = path_stops;
    std::string text_line;
    std::vector<std::string> parameters_string;
    std::ifstream text_file;
    if (stops_path.stem() == "stops" && stops_path.extension() == ".txt"){
        text_file.open(stops_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {

                    int sid = std::stoi(tline.at(2));
                    double lat = std::stod(tline.at(0));
                    double lon = std::stod(tline.at(1));
                    OGRPoint* stop_p = new OGRPoint;
                    stop_p->setX(lat);
                    stop_p->setY(lon);
                    stop_p->assignSpatialReference(&wgs_srs);
                    stop_p->transformTo(&spartef);

                    stop_id_vec.push_back(sid);
                    stop_coord.push_back(*stop_p);
                    delete stop_p;

                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}
}


std::unordered_map<int, int> gtfs::GTFS_lines::plain_routes(std::string path_routes){

    std::unordered_map<int, int> plain_data;

    std::filesystem::path route_path = path_routes;
    std::string text_line;
    std::string first_line;

    std::ifstream text_file_open;
    long rt_index;
    long route_id_index;
    if (route_path.stem() == "routes" && route_path.extension() == ".txt"){
        text_file_open.open(route_path);
        while(std::getline(text_file_open, first_line)){
            try{
                v_string fline_names = split_string(first_line, std::string(","));
                std::string route_type_d = "route_type";
                std::string route_id_d = "route_id";
                if (std::find(fline_names.begin(), fline_names.end(), route_type_d) != fline_names.end()){
                    for (int i = 0; i < fline_names.size(); i++){
                        if (fline_names.at(i) == route_type_d){
                            rt_index = i;
                        } else if (fline_names.at(i) == route_id_d){
                            route_id_index = i;
                        }
                    }
                    break;
                }
            } catch (...){

            }
                
        }
        text_file_open.close();
    }



    std::ifstream text_file;
    if (route_path.stem() == "routes" && route_path.extension() == ".txt"){
        text_file.open(route_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {

                    int route_type = std::stoi(tline.at(0)); // 0
                    int route_id = std::stoi(tline.at(4)); // 4

                    plain_data.emplace(route_id, route_type);
                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}

    return plain_data;
}


std::vector<std::tuple<int, std::string, std::string>> gtfs::GTFS_lines::trip_load(std::string path_routes){

    std::vector<std::tuple<int, std::string, std::string>> trip_vector;

    std::filesystem::path route_path = path_routes;
    std::string text_line;
    std::vector<std::string> parameters_string;


    std::ifstream text_file;
    if (route_path.stem() == "trips" && route_path.extension() == ".txt"){
        text_file.open(route_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {
                    
                    std::tuple<int, std::string, std::string> inside_tuple;

                    int route_id_data = std::stoi(tline.at(0));
                    std::string trip_id_data = tline.at(1);
                    std::string shape_id_data = tline.at(4);

                    inside_tuple = std::make_tuple(route_id_data, trip_id_data, shape_id_data);
                    trip_vector.push_back(inside_tuple);

                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}

    return trip_vector;
}


std::unordered_map<std::string, double> gtfs::GTFS_lines::freq_load(std::string path_routes){

    std::unordered_map<std::string, double> frequency_data;

    std::filesystem::path route_path = path_routes;
    std::string text_line;
    std::vector<std::string> parameters_string;
    std::ifstream text_file;
    if (route_path.stem() == "frequencies" && route_path.extension() == ".txt"){
        text_file.open(route_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {
                    
                    std::string trip_id_data = tline.at(0);
                    int headway_sec = std::stoi(tline.at(3));

                    frequency_data.emplace(trip_id_data, headway_sec);

                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}

    return frequency_data;
}


std::unordered_map<std::string, OGRLineString> gtfs::GTFS_lines::shape_locations(std::string path_routes, OGRSpatialReference projected){
    
    std::unordered_map<std::string, OGRLineString> geoline_data;

    OGRSpatialReference wgs_srs;
    wgs_srs.SetWellKnownGeogCS("WGS84");

    v_string shapes_ids;
    v_int number_vec;
    std::vector<OGRPoint> linepoints; 

    std::filesystem::path route_path = path_routes;
    std::string text_line;
    std::vector<std::string> parameters_string;
    std::ifstream text_file;
    if (route_path.stem() == "shapes" && route_path.extension() == ".txt"){
        text_file.open(route_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {
                    
                    std::string shapes_id_data = tline.at(0);
                    
                    double lat = std::stod(tline.at(1));
                    double lon = std::stod(tline.at(2));
                    OGRPoint* stop_l = new OGRPoint;

                    stop_l->setX(lat);
                    stop_l->setY(lon);

                    stop_l->assignSpatialReference(&wgs_srs);
                    stop_l->transformTo(&projected);

                    int number_point = std::stoi(tline.at(3));

                    shapes_ids.push_back(shapes_id_data);
                    linepoints.push_back(*stop_l);
                    delete stop_l;

                    number_vec.push_back(number_point);

                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}

    v_string unique_shapes = shapes_ids;
    std::vector<std::string>::iterator str_it = std::unique(unique_shapes.begin(), unique_shapes.end());
    unique_shapes.resize(std::distance(unique_shapes.begin(), str_it));

    for (std::string shape_id:unique_shapes){

        std::map<int, OGRPoint, std::less<int>> innermap;

        for (ssize_t i = 0; i != shapes_ids.size(); i++){
            if (shapes_ids.at(i) == shape_id){
                innermap.try_emplace(number_vec.at(i), linepoints.at(i));
            }
        }

        OGRLineString line;

        for (auto kv:innermap){
            line.setPoint(kv.first - 1, &kv.second);
        }

        geoline_data.try_emplace(shape_id, line);
    }

    return geoline_data;
}


std::unordered_map<std::string, v_int> gtfs::GTFS_lines::stop_connections(std::string path_routes){

    std::unordered_map<std::string, v_int> stop_route;

    v_string tripid_vec;
    v_int stop_id_vec;
    v_int stop_position_vec;

    std::filesystem::path route_path = path_routes;
    std::string text_line;
    std::vector<std::string> parameters_string;
    std::ifstream text_file;
    if (route_path.stem() == "stop_times" && route_path.extension() == ".txt"){
        text_file.open(route_path);
        if (text_file.is_open()){
            while (std::getline(text_file, text_line)){
                v_string tline = split_string(text_line, std::string(","));

                try {
                    
                    std::string trip_id_data = tline.at(0);
                    int stoppos_id = std::stoi(tline.at(3));
                    int stop_seq = std::stoi(tline.at(4));

                    tripid_vec.push_back(trip_id_data);
                    stop_id_vec.push_back(stoppos_id);
                    stop_position_vec.push_back(stop_seq);

                } catch (...){
                    goto label2;
                }
                label2: {}
            }
            text_file.close();
        } else {
            std::cout << "Файла по этому пути нет" << std::endl;
        }
    } else {}

    v_string unique_id = tripid_vec;
    std::vector<std::string>::iterator str_it = std::unique(unique_id.begin(), unique_id.end());
    unique_id.resize(std::distance(unique_id.begin(), str_it));

    for (std::string tripid:unique_id){

        std::map<int, int, std::less<int>> innermap;

        for (ssize_t i = 0; i != tripid_vec.size(); i++){
            if (tripid_vec.at(i) == tripid){ // ?!
                innermap.try_emplace(stop_position_vec.at(i), stop_id_vec.at(i));
            }
        }

        v_int points_vec;

        for (auto kv:innermap){
            points_vec.push_back(kv.second);
        }

        stop_route.try_emplace(tripid, points_vec);
        points_vec.clear();
    }

    return stop_route;
}


void gtfs::GTFS_lines::make_routes(std::unordered_map<std::string, std::string> gtfs_paths, OGRSpatialReference projected){

    auto extract_numbers = [](std::string trv){
        std::string temp_string;
        for (char x:trv){
            int x_int = x - '0';
            if ((x_int >= 0) && (x_int <= 9)){
                temp_string += std::to_string(x_int);
            }
        }
        return temp_string;
    };

    std::unordered_map<int, int> plain_rt = plain_routes(gtfs_paths.at("routes"));
    std::vector<std::tuple<int, std::string, std::string>> trip_data = trip_load(gtfs_paths.at("trips"));
    std::unordered_map<std::string, double> freq_data = freq_load(gtfs_paths.at("frequencies"));
    std::unordered_map<std::string, OGRLineString> line_data = shape_locations(gtfs_paths.at("shapes"), projected);
    std::unordered_map<std::string, v_int> stops_on_route = stop_connections(gtfs_paths.at("stop_times"));

    for (auto rid:plain_rt){

        for (auto tpd:trip_data){
            if (std::get<0>(tpd) == rid.first){

                route_id_vec.push_back(std::get<0>(tpd));
                type_of_transport.push_back(rid.second);
                trip_id_vec.push_back(std::get<1>(tpd));
                trip_vec.push_back(std::get<2>(tpd));
            }
        }
    }
    
    for (std::string trv:trip_vec){

        for (auto lda:line_data){

            if (extract_numbers(lda.first) == extract_numbers(trv)){
                route_geometry.push_back(lda.second);
            }
        }

    }

    for (std::string tiv:trip_id_vec){
        for (auto fqd:freq_data){
            if (extract_numbers(fqd.first) == extract_numbers(tiv)){
                time_head_vec.push_back(fqd.second);
            }
        }
    }


    for (std::string tiv:trip_id_vec){
        for (auto sor:stops_on_route){
            if (extract_numbers(sor.first) == extract_numbers(tiv)){
                stops_in_routes.push_back(sor.second);
            }
        }
    }
}


void gtfs::GTFS_lines::save_prev_lines(std::string savepath, OGRSpatialReference srs_d){
    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(savepath.data(), 0, 0, 0,
    GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbLineString, NULL);

    OGRFieldDefn type_field("TripId", OFTString);

    feature_layer->CreateField(&type_field);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < route_geometry.size(); i++){
        OGRFeature *linestr_feature;
        linestr_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        linestr_feature->SetField("TripId", trip_id_vec.at(i).data());

        linestr_feature->SetGeometry(&route_geometry.at(i));
        feature_layer->CreateFeature(linestr_feature);
        OGRFeature::DestroyFeature(linestr_feature);
    }
    GDALClose(dataset);
}

void gtfs::GTFS_points::save_prev_points(std::string savepath, OGRSpatialReference srs_d){
    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(savepath.data(), 0, 0, 0,
    GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbPoint, NULL);

    OGRFieldDefn iddata("StopID", OFTInteger);

    feature_layer->CreateField(&iddata);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < stop_coord.size(); i++){
        OGRFeature *linestr_feature;
        linestr_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        linestr_feature->SetField("StopID", stop_id_vec.at(i));

        linestr_feature->SetGeometry(&stop_coord.at(i));
        feature_layer->CreateFeature(linestr_feature);
        OGRFeature::DestroyFeature(linestr_feature);
    }
    GDALClose(dataset);
}


void gtfs::GTFS_points::get_metro_stations(std::string metro_path_st, OGRSpatialReference osrs){

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(metro_path_st.data(), GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        OGRLayer *point_layer;
        point_layer = gdal_data->GetLayer(0);
        for (const auto& point_feature:*point_layer){
            v_string lineid_data;
            for(const auto& field_data: *point_feature){
                std::string fname = field_data.GetName();
                if (fname == "pointid"){
                    stop_id_vec.push_back(field_data.GetAsInteger64());
                } else if (fname == "dir"){
                    lineid_data.push_back(field_data.GetAsString());
                }
            }
            OGRGeometry *general_geometry = point_feature->StealGeometry();
            if (general_geometry != nullptr && general_geometry->getGeometryType() == wkbPoint){
                OGRPoint point_geom;
                point_geom = *general_geometry->toPoint();
                point_geom.transformTo(&osrs);
                stop_coord.push_back(point_geom);
            } else if (general_geometry != nullptr && general_geometry->getGeometryType() == wkbMultiPoint){
                OGRMultiPoint ogrmultipoint_s = *general_geometry->toMultiPoint();
                for (auto single_point:ogrmultipoint_s){
                    single_point->transformTo(&osrs);
                    stop_coord.push_back(*single_point);
                }
            }
        }
    } else {
        std::cout << "Такого шейпфайла нет. Проверь путь до шейпа" << std::endl;
    }
}


void gtfs::GTFS_lines::get_metro_lines(std::string metro_path_lines, OGRSpatialReference osrs){

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(metro_path_lines.data(), GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        OGRLayer *line_layer;
        line_layer = gdal_data->GetLayer(0);
        for (const auto& line_feature:*line_layer){
            v_int lineid_data;
            for(const auto& field_data: *line_feature){
                std::string fname = field_data.GetName();
                if (fname == "lineid"){
                    route_id_vec.push_back(field_data.GetAsInteger());
                } else if (fname == "tr_type"){
                    type_of_transport.push_back(field_data.GetAsInteger());
                } else if (fname == "frequency"){
                    time_head_vec.push_back(field_data.GetAsDouble());
                } else if (fname == "direction"){
                    trip_vec.push_back(field_data.GetAsString());
                }
            }
            OGRGeometry *general_type = line_feature->StealGeometry();
            if (general_type != nullptr && general_type->getGeometryType() == wkbMultiLineString){
                OGRMultiLineString roads_mp = *general_type->toMultiLineString();
                for (auto rmp:roads_mp){
                    rmp->transformTo(&osrs);
                    route_geometry.push_back(*rmp);
                }
            } else if (general_type != nullptr && general_type->getGeometryType() == wkbLineString){
                OGRLineString roads_mp = *general_type->toLineString();
                roads_mp.transformTo(&osrs);
                route_geometry.push_back(roads_mp);
                }
        }
    } else {
        std::cout << "Такого шейпфайла нет. Проверь путь до шейпа" << std::endl;
    }
}


void gtfs::GTFS_points::make_connection(){
    
    for (int i = 0; i < stop_coord.size(); i++){
        id_connect.try_emplace(stop_id_vec.at(i), stop_coord.at(i));
    }
}

void gtfs::GTFS_lines::connect_stations(std::unique_ptr<GTFS_points>& stations){

    for (int i = 0; i < route_geometry.size(); i++){
        if (type_of_transport.at(i) == -1){
            v_int rgeom_metro;
            for (OGRPoint mpoint:route_geometry.at(i)){
                OGRGeometry* buffer_1 = NULL;
                buffer_1 = mpoint.Buffer(3);
                for (int j = 0; j < stations->stop_coord.size(); j++){
                    if (buffer_1->Intersects(&stations->stop_coord.at(j))){
                        rgeom_metro.push_back(stations->stop_id_vec.at(j));
                    }
                }
                delete buffer_1;
            }
            stops_in_routes.push_back(rgeom_metro);
        }
    }
}


void gtfs::GTFS_lines::calculate_capacity(){

    for (int i = 0; i < time_head_vec.size(); i++){
        if (type_of_transport.at(i) == 3){

            double cap = (static_cast<double>(3600) / static_cast<double>(time_head_vec.at(i))) * static_cast<double>(3) * static_cast<double>(112);
            transport_capacity.push_back(cap);

        } else if (type_of_transport.at(i) == 0){

            double cap = (static_cast<double>(3600) / static_cast<double>(time_head_vec.at(i))) * static_cast<double>(3) * static_cast<double>(162);
            transport_capacity.push_back(cap);

        } else if (type_of_transport.at(i) == 4){
            double cap = (static_cast<double>(3600) / static_cast<double>(time_head_vec.at(i))) * static_cast<double>(3) * static_cast<double>(1320);
            transport_capacity.push_back(cap);
        }
    }
}


void gtfs::GTFS_lines::stop_capacity(std::unique_ptr<GTFS_points>& stops){

    for (int stopid:stops->stop_id_vec){
        stops->total_capacity.emplace(stopid, static_cast<double>(0));
    }

    std::cout << "stops_in_routes: " << stops_in_routes.size() << std::endl;
    std::cout << "transport_capacity: " << transport_capacity.size() << std::endl;

    for (int stopid:stops->stop_id_vec){
        for (int strt = 0; strt < stops_in_routes.size(); strt++){
            if (std::find(stops_in_routes.at(strt).begin(), stops_in_routes.at(strt).end(), stopid) != stops_in_routes.at(strt).end()){
                if (stops->total_capacity.contains(stopid)){
                    stops->total_capacity.at(stopid) += transport_capacity.at(strt);
                }
            }
        }
    }
}