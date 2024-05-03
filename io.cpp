#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <functional>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <thread>
#include <numeric>
#include <fstream>
#include <filesystem>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>
#include "io.hpp"

namespace fs = std::filesystem;

std::map<std::tuple<double, double>, std::string> io::make_utm_crs(){
    std::map<std::tuple<double, double>, std::string> utm_map;
    int start_mer = -180;
    int end_mer = -174;
    for (int i = 1; i != 60; i++){
        std::tuple<double, double> cr_tup = std::make_tuple(static_cast<double>(start_mer), static_cast<double>(end_mer));
        OGRSpatialReference oSRS;
        oSRS.SetWellKnownGeogCS( "WGS84" );
        oSRS.SetUTM(i, TRUE);
        char *pszWKT = new char;
        oSRS.exportToWkt( &pszWKT );
        std::string str_utm = pszWKT;
        delete pszWKT;
        utm_map.emplace(cr_tup, str_utm);
        start_mer += 6;
        end_mer += 6;
    }
    return utm_map;
}

OGRSpatialReference io::estimate_utm_crs(OGREnvelope envel, std::map<std::tuple<double, double>, std::string> utm_data){
    std::unique_ptr<OGRLinearRing> linring = std::make_unique<OGRLinearRing>();
    linring->setPoint(0, envel.MinX, envel.MinY);
    linring->setPoint(1, envel.MinX, envel.MaxY);
    linring->setPoint(2, envel.MaxX, envel.MaxY);
    linring->setPoint(3, envel.MaxY, envel.MinY);
    linring->setPoint(4, envel.MinX, envel.MinY);

    OGRPolygon* opoly = new OGRPolygon;
    opoly->addRing(linring->toCurve());

    OGRPoint* opoint = new OGRPoint;
    opoly->Centroid(opoint);
    
    OGRSpatialReference oSRS;

    OGRPoint centroid_data;
    for (auto cv:utm_data){
        if ((opoint->getY() >= std::get<0>(cv.first)) && (opoint->getY() < std::get<1>(cv.first))){
            oSRS.importFromWkt(cv.second.data());
        }
    }
    
    delete opoint;
    delete opoly;

    return oSRS;
}

void io::Osm_building::download_data(std::string pathname){
    GDALAllRegister();

    std::map<std::tuple<double, double>, std::string> utm_data = io::make_utm_crs();

    const char* path_to_file = pathname.data();
    auto lamda_val = [](const OGRFeature::FieldValue& field_data){return std::string(field_data.GetName());};

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(path_to_file, GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        OGRLayer *point_layer;
        std::string fcl;
        std::string onew;
        std::string maxspd;
        point_layer = gdal_data->GetLayer(0);
        OGRErr envel_err = point_layer->GetExtent(&buildings_extent, TRUE);
        osrs_spatref = io::estimate_utm_crs(buildings_extent, utm_data);

        for (const auto& point_feature:*point_layer){

            v_string attr_names;
            std::transform(point_feature->begin(), point_feature->end(), std::back_inserter(attr_names), lamda_val);
            if(!((std::find(attr_names.begin(), attr_names.end(), "osm_id") != attr_names.end()) && 
            (std::find(attr_names.begin(), attr_names.end(), "building") != attr_names.end()) &&
            (std::find(attr_names.begin(), attr_names.end(), "building_l") != attr_names.end()))){
                continue;
            }

            for(const auto& field_data: *point_feature){

                std::string fname = field_data.GetName();
                if (fname == "osm_id"){
                    osm_id_vec.push_back(field_data.GetAsInteger64());
                } else if (fname == "building"){
                    building_type_vec.push_back(field_data.GetAsString());
                } else if (fname == "building_l"){
                    try{
                        levels.push_back(std::stoi(field_data.GetAsString()));
                    } catch (...){
                        levels.push_back(-1);
                    }
                } else {}
            }
            
            OGRGeometry *general_geometry = point_feature->GetGeometryRef();
            if (general_geometry != nullptr){
                OGRPolygon *poly_geom = new OGRPolygon;
                poly_geom = general_geometry->toPolygon();
                poly_geom->transformTo(&osrs_spatref);
                square_vec.push_back(poly_geom->get_Area());
                wkt_polygon.push_back(*poly_geom);
                delete poly_geom;
            }
        }
    } else {
        std::cout << "Такого шейпфайла нет. Проверь путь до шейпа" << std::endl;
    }
}

void io::Osm_roads::download_data(std::string pathname){
    GDALAllRegister();
    std::map<std::tuple<double, double>, std::string> utm_data = io::make_utm_crs();

    const char* path_to_file = pathname.data();
    auto lamda_val = [](const OGRFeature::FieldValue& field_data){return std::string(field_data.GetName());};

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(path_to_file, GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        OGRLayer *line_layer;
        std::string fcl;
        std::string onew;
        std::string maxspd;
        line_layer = gdal_data->GetLayer(0);
        OGRErr envel_err = line_layer->GetExtent(&roads_extent, TRUE);
        osrs_spatref = io::estimate_utm_crs(roads_extent, utm_data);

        for (const auto& line_feature:*line_layer){

            v_string attr_names;
            std::transform(line_feature->begin(), line_feature->end(), std::back_inserter(attr_names), lamda_val);
            if(!((std::find(attr_names.begin(), attr_names.end(), "osm_id") != attr_names.end()) && 
            (std::find(attr_names.begin(), attr_names.end(), "fclass") != attr_names.end()))){
                continue;
            }

            for(const auto& field_data: *line_feature){

                std::string fname = field_data.GetName();
                if (fname == "osm_id"){
                    osm_id_vec.push_back(std::stoi(field_data.GetAsString()));
                } else if (fname == "fclass"){
                    fclass_vec.push_back(field_data.GetAsString());
                }
            }
            
            OGRGeometry *general_type = line_feature->GetGeometryRef();
            if (general_type != nullptr && general_type->getGeometryType() == wkbLineString){
                OGRLineString* linestring_geom = new OGRLineString;
                linestring_geom = general_type->toLineString();
                linestring_geom->transformTo(&osrs_spatref);
                roads_length.push_back(linestring_geom->get_Length());
                wkt_linestring.push_back(*linestring_geom);
                delete linestring_geom;
            } else if (general_type != nullptr && general_type->getGeometryType() == wkbMultiLineString){
                OGRMultiLineString* multi_line = new OGRMultiLineString;
                multi_line = general_type->toMultiLineString();
                for (auto lstr:multi_line){
                    wkt_linestring.push_back(*lstr);
                }
                delete multi_line;
            }
        }
    } else {
        std::cout << "Такого шейпфайла нет. Проверь путь до шейпа" << std::endl;
    }
    return;
}

long long io::time_seconds(){
    const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    const std::chrono::system_clock::duration epoch = now.time_since_epoch();
    const std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    return seconds.count();
}

void io::Osm_roads::make_array(){
    auto lambda_geom_array = [](OGRLineString& wkt_string){
        linestring lstr;
        for (OGRPoint opnt:wkt_string){
            point non_ogr_point = std::make_tuple(opnt.getX(), opnt.getY());
            lstr.push_back(non_ogr_point);
        }
        return lstr;
    };

    for (OGRLineString& wls:wkt_linestring){
        linestr_vec.push_back(lambda_geom_array(wls));
    }
}

void io::Osm_building::make_array(){
    auto lambda_geom_array = [](OGRPolygon simple_poly){
        point pt;
        
        OGRPoint* ogr_point = new OGRPoint;
        simple_poly.Centroid(ogr_point);          
        pt = std::make_tuple(ogr_point->getX(), ogr_point->getY());
        delete ogr_point;
        return pt;
    };

    for (OGRPolygon& spoly:wkt_polygon){
        bgeom_array.push_back(lambda_geom_array(spoly));
    }
}

void io::Osm_roads::spindex_grid(){

    auto envelop_find = [](std::vector<std::vector<std::tuple<double, double>>>& vector_coordinates){
        std::vector<std::tuple<double, double>> envelop_values;

        v_doub x_values;
        v_doub y_values;

        for (unsigned long i = 0; i != vector_coordinates.size(); i++){
            for (auto vc_data:vector_coordinates.at(i)){
                x_values.push_back(std::get<0>(vc_data));
                y_values.push_back(std::get<1>(vc_data));
                }
            }

        envelop_values.push_back(std::make_tuple(*std::min_element(x_values.begin(), x_values.end()),
            *std::min_element(y_values.begin(), y_values.end())));

        envelop_values.push_back(std::make_tuple(*std::max_element(x_values.begin(), x_values.end()),
            *std::max_element(y_values.begin(), y_values.end())));


        return envelop_values;
    };

    auto make_grid = [](std::vector<std::tuple<double, double>>& frame_envelop){

        std::unordered_map<int, v_doub> grid;

        double x_change = (std::get<0>(frame_envelop.at(1)) - std::get<0>(frame_envelop.at(0)))/10;
        double y_change = (std::get<1>(frame_envelop.at(1)) - std::get<1>(frame_envelop.at(0)))/10;
        double base_xcoord = std::get<0>(frame_envelop.at(0));
        double base_xcoord_max = std::get<0>(frame_envelop.at(0)) + x_change;
        int indicator = 0;
        for (int i = 0; i < 10; i++){
            double base_ycoord = std::get<1>(frame_envelop.at(0));
            double base_ycoord_max = std::get<1>(frame_envelop.at(0)) + y_change;
            for (int j = 0; j < 10; j++){
                v_doub vecline;
                vecline.push_back(base_xcoord);
                vecline.push_back(base_ycoord);
                vecline.push_back(base_xcoord_max);
                vecline.push_back(base_ycoord_max);
                base_ycoord += y_change;
                base_ycoord_max += y_change;
                indicator++;
                grid.emplace(indicator, vecline);
                vecline.clear();
            }
            base_xcoord += x_change;
            base_xcoord_max += x_change;
        }

        return grid;
    };

    std::vector<std::tuple<double, double>> envelop_v = envelop_find(linestr_vec);
    std::unordered_map<int, v_doub> grid_data = make_grid(envelop_v);

    for (linestring& lstr:linestr_vec){
        std::vector<int> intindecies;
        for (point& pt:lstr){
            for (auto gdata:grid_data){
                std::vector<double> gdata_boundary = gdata.second;
                if ((std::get<0>(pt) >= gdata_boundary.at(0)) && (std::get<1>(pt) >= gdata_boundary.at(1))){
                    if ((std::get<0>(pt) <= gdata_boundary.at(2)) && (std::get<1>(pt) <= gdata_boundary.at(3))){
                        intindecies.push_back(gdata.first);
                    }
                }
            }
        }
        spatial_index.push_back(intindecies);
        intindecies.clear();
    }
}

std::string io::get_unix_time(){
    auto timestart_full = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const char * tc = ctime(&timestart_full);
    std::string stime = {tc};
    stime.pop_back();
    return stime;
}


OGRMultiPolygon io::pbf_find_city(std::string path_to_pbf, long long city_id){

    OGRMultiPolygon* mp = new OGRMultiPolygon;

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(path_to_pbf.data(), GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        bool bHasLayersNonEmpty;
        do {
            bHasLayersNonEmpty = false;
            for (int lr = 0; lr < gdal_data->GetLayerCount(); lr++){
                OGRLayer *line_layer;
                line_layer = gdal_data->GetLayer(lr);
                if (line_layer != nullptr){
                    OGRFeature* poFeature;
                    while ((poFeature = line_layer->GetNextFeature()) != NULL){
                        bHasLayersNonEmpty = true;
                        for(const auto& field_data: *poFeature){
                            std::string fname = field_data.GetName();
                            if (fname == "osm_id"){
                                if (std::stoll(field_data.GetAsString()) == city_id){
                                    OGRGeometry* general_type = poFeature->StealGeometry();
                                    if (general_type != nullptr && general_type->getGeometryType() == wkbMultiPolygon){
                                        mp = general_type->toMultiPolygon();
                                        goto label_finish;
                                    }
                                }
                            }
                        }
                        OGRFeature::DestroyFeature(poFeature);
                    }
                }
            }
        }
        while (bHasLayersNonEmpty);
        
    }

    label_finish: {}
    
    OGRMultiPolygon mp_value = *mp;
    delete mp;
    return mp_value;
}

void io::open_pbf(std::string path_to_pbf, std::unique_ptr<Osm_building>& osm_bld, std::unique_ptr<Osm_roads>& osm_road,
        std::unique_ptr<Osm_amenities>& apoint, long long city_id){
    
        auto split_string = [](std::string str, std::string delim){
            std::vector<std::string> fullstr;
            int start, end = -1*delim.size();
            do {
                start = end + delim.size();
                end = str.find(delim, start);
                fullstr.push_back(str.substr(start, end - start));
            } while (end != -1);
            return fullstr;
        };

    OGRMultiPolygon city_bound = pbf_find_city(path_to_pbf, city_id);

    OGREnvelope ogr_e;
    city_bound.getEnvelope(&ogr_e);

    std::map<std::tuple<double, double>, std::string> utm_data = make_utm_crs();
    osm_bld->osrs_spatref = io::estimate_utm_crs(ogr_e, utm_data);

    std::string delim = ",";
    std::string fdr = "levels";

    long long local_osmid = static_cast<long long>(1);

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(path_to_pbf.data(), GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        bool bHasLayersNonEmpty;
        do {
            bHasLayersNonEmpty = false;
            for (int lr = 0; lr < gdal_data->GetLayerCount(); lr++){
                OGRLayer *line_layer;
                line_layer = gdal_data->GetLayer(lr);
                if (line_layer != nullptr){
                    OGRFeature* poFeature;
                    while ((poFeature = line_layer->GetNextFeature()) != NULL){

                        bHasLayersNonEmpty = true;

                        std::unordered_map<std::string, long long> uid;
                        // std::unordered_map<std::string, std::string> uid;

                        for (const auto& field_data: *poFeature){
                            
                            // Здания с этажами
                            std::string fname = field_data.GetName();

                            std::string building_tag;
                            int levels = -1;
                            local_osmid += static_cast<long long>(1);
                            if (fname == "osm_id"){
                                uid.emplace(std::string("osm_id"), field_data.GetAsInteger64());
                            }

                            if (fname == "other_tags"){
                                std::string fdata = field_data.GetAsString();
                                std::vector<std::string> other_tags = split_string(fdata, delim);
                                if (other_tags.size() != 0){
                                    for (std::string subs:other_tags){
                                        if (subs.find(fdr) != std::string::npos){
                                            std::vector<std::string> other_tags = split_string(subs, std::string("=>"));
                                            try{
                                                levels = std::stoi(other_tags.at(1));
                                            } catch (...){

                                            }
                                        }
                                    }
                                }
                            }
                            if ((fname == "building") && (std::string(field_data.GetAsString()).length() != 0)){
                                OGRGeometry* general_type = poFeature->StealGeometry();
                                if (general_type != nullptr && general_type->getGeometryType() == wkbMultiPolygon){
                                        if (general_type->Intersects(&city_bound)){
                                            OGRMultiPolygon buildings_mp = *general_type->toMultiPolygon();
                                            for (auto poly:buildings_mp){
                                                poly->transformTo(&osm_bld->osrs_spatref);
                                                osm_bld->square_vec.push_back(poly->get_Area());
                                                osm_bld->wkt_polygon.push_back(*poly);
                                                osm_bld->building_type_vec.push_back(field_data.GetAsString());
                                                osm_bld->levels.push_back(levels);

                                                if (uid.find("osm_id") != uid.end()){
                                                    osm_bld->osm_id_vec.push_back(local_osmid);
                                                } else {
                                                    osm_bld->osm_id_vec.push_back(local_osmid);
                                                }
                                            }
                                        }
                                }
                                delete general_type;
                            }

                            // Roads
                            if (fname == "highway"){
                                if (std::string(field_data.GetAsString()).length() != 0){
                                    OGRGeometry* general_type = poFeature->StealGeometry();
                                        if (general_type->Intersects(&city_bound)){
                                            if (general_type != nullptr && general_type->getGeometryType() == wkbMultiLineString){
                                                OGRMultiLineString roads_mp = *general_type->toMultiLineString();
                                                for (auto rmp:roads_mp){
                                                    rmp->transformTo(&osm_bld->osrs_spatref);
                                                    osm_road->wkt_linestring.push_back(*rmp);
                                                    osm_road->roads_length.push_back(rmp->get_Length());
                                                    osm_road->fclass_vec.push_back(field_data.GetAsString());
                                                    osm_road->osm_id_vec.push_back(local_osmid);
                                                }
                                            } else if (general_type != nullptr && general_type->getGeometryType() == wkbLineString){
                                                OGRLineString roads_mp = *general_type->toLineString();
                                                roads_mp.transformTo(&osm_bld->osrs_spatref);
                                                osm_road->wkt_linestring.push_back(roads_mp);
                                                osm_road->roads_length.push_back(roads_mp.get_Length());
                                                osm_road->fclass_vec.push_back(field_data.GetAsString());
                                                osm_road->osm_id_vec.push_back(local_osmid);
                                                }
                                        }
                                    delete general_type;
                                }
                            }

                            // POIs
                            if (fname == "amenity"){
                                if (std::string(field_data.GetAsString()).length() != 0){
                                    OGRGeometry* general_type = poFeature->StealGeometry();
                                    if (general_type != nullptr && general_type->getGeometryType() == wkbPoint){
                                        if (general_type->Intersects(&city_bound)){
                                        OGRPoint buildings_mp = *general_type->toPoint();
                                        buildings_mp.transformTo(&osm_bld->osrs_spatref);
                                        apoint->amenity_point.push_back(buildings_mp);
                                        }
                                    }
                                    delete general_type;
                                }
                            }

                            if (fname == "leisure"){
                                if (std::string(field_data.GetAsString()).length() != 0){
                                    OGRGeometry* general_type = poFeature->StealGeometry();
                                    if (general_type != nullptr && general_type->getGeometryType() == wkbPoint){
                                        if (general_type->Intersects(&city_bound)){
                                        OGRPoint buildings_mp = *general_type->toPoint();
                                        buildings_mp.transformTo(&osm_bld->osrs_spatref);
                                        apoint->amenity_point.push_back(buildings_mp);
                                        }
                                    }
                                    delete general_type;
                                }
                            }

                            if (fname == "tourism"){
                                if (std::string(field_data.GetAsString()).length() != 0){
                                    OGRGeometry* general_type = poFeature->StealGeometry();
                                    if (general_type != nullptr && general_type->getGeometryType() == wkbPoint){
                                        if (general_type->Intersects(&city_bound)){
                                        OGRPoint buildings_mp = *general_type->toPoint();
                                        buildings_mp.transformTo(&osm_bld->osrs_spatref);
                                        apoint->amenity_point.push_back(buildings_mp);
                                        }
                                    }
                                    delete general_type;
                                }
                            }

                            if (fname == "other_tags"){
                                std::string fdata = field_data.GetAsString();
                                std::vector<std::string> other_tags = split_string(fdata, delim);
                                if (other_tags.size() != 0){
                                    for (std::string subs:other_tags){
                                        if ((subs.find(std::string("amenity")) != std::string::npos) || (subs.find(std::string("leisure")) != std::string::npos)
                                        || (subs.find(std::string("tourism")) != std::string::npos)){
                                            OGRGeometry* general_type = poFeature->StealGeometry();
                                            if (general_type != nullptr && general_type->getGeometryType() == wkbPoint){
                                                if (general_type->Intersects(&city_bound)){
                                                OGRPoint buildings_mp = *general_type->toPoint();
                                                buildings_mp.transformTo(&osm_bld->osrs_spatref);
                                                apoint->amenity_point.push_back(buildings_mp);
                                                }
                                            }
                                            delete general_type;
                                        }
                                    }
                                }
                            }
                            
                        }
                        OGRFeature::DestroyFeature(poFeature);
                    }
                }
            }
        }
        while (bHasLayersNonEmpty);
        
    }

    label_finish: {}
}


void io::Genplan_data::download_poly(std::string path_to_poly){

    std::map<std::tuple<double, double>, std::string> utm_data = io::make_utm_crs();

    const char* path_to_file = path_to_poly.data();

    GDALDatasetUniquePtr gdal_data(GDALDataset::Open(path_to_file, GDAL_OF_VECTOR, NULL, NULL, NULL));
    if (gdal_data != nullptr){
        OGRLayer *point_layer;
        OGREnvelope city_env;
        point_layer = gdal_data->GetLayer(0);
        OGRErr envel_err = point_layer->GetExtent(&city_env, TRUE);
        OGRSpatialReference osrs_spatref = io::estimate_utm_crs(city_env, utm_data);

        for (const auto& point_feature:*point_layer){

            for(const auto& field_data: *point_feature){

                std::string fname = field_data.GetName();
                if (fname == "zone"){
                    category_genplan.push_back(field_data.GetAsString());
                } else {}
            }

            OGRGeometry *general_geometry = point_feature->GetGeometryRef();
            if (general_geometry != nullptr){
                if (general_geometry->getGeometryType() == wkbMultiPolygon){
                    OGRMultiPolygon mult = *general_geometry->toMultiPolygon();
                    for (auto child_mult:mult){
                        child_mult->transformTo(&osrs_spatref);
                        polygon_data.push_back(*child_mult);
                    }

                } else if (general_geometry->getGeometryType() == wkbPolygon){
                    OGRPolygon poly_geom = *general_geometry->toPolygon();
                    poly_geom.transformTo(&osrs_spatref);
                    polygon_data.push_back(poly_geom);
                }
                
            }
        }
    } else {
        std::cout << "Такого шейпфайла нет. Проверь путь до шейпа" << std::endl;
    }
}

void io::Genplan_data::download_txt(std::string path_to_txt){
    std::ifstream text_file;
    std::string text_line;
    std::vector<std::string> parameters_string;

    auto split_string = [](std::string str, std::string delim){
            std::vector<std::string> fullstr;
            int start, end = -1*delim.size();
            do {
                start = end + delim.size();
                end = str.find(delim, start);
                fullstr.push_back(str.substr(start, end - start));
            } while (end != -1);
            return fullstr;
        };

    fs::path txtpath = path_to_txt;
    text_file.open(txtpath);
    if (text_file.is_open()){
        while (std::getline(text_file, text_line)){
            parameters_string.push_back(text_line);
        }
        text_file.close();
    } else {
        std::cout << "Файла по этому пути нет" << std::endl;
    }

    for (std::string sline:parameters_string){
        std::vector<std::string> delimit_str = split_string(sline, std::string(" "));
        workforce_distrib.emplace(delimit_str.at(0), std::stoi(delimit_str.at(1)));
        delimit_str.clear();
    }
}


std::string io::find_pbf(std::string exe_path){

    std::string pbfpath;
    for (const auto& spath:fs::recursive_directory_iterator(exe_path, fs::directory_options::skip_permission_denied)){
        if (!(spath.is_directory())){
            auto filename = spath.path().filename();
            if ((filename.extension() == ".pbf")){
                pbfpath = spath.path().string();
            }
        }
    }
    return pbfpath;
}


std::string io::find_genplan_shp(std::string exe_path){

    std::string shp_genplan;

    for (const auto& spath:fs::recursive_directory_iterator(exe_path, fs::directory_options::skip_permission_denied)){
        if (!(spath.is_directory())){
            auto filename = spath.path().filename();
            if ((filename.extension() == ".shp") && (filename.stem().string().find(std::string("functional")) != std::string::npos)){
                shp_genplan = spath.path().string();
            }
        }
    }

    return shp_genplan;
}


std::string io::find_txt_path(std::string exe_path){

    std::string textpath;

    for (const auto& spath:fs::recursive_directory_iterator(exe_path, fs::directory_options::skip_permission_denied)){
        if (!(spath.is_directory())){
            auto filename = spath.path().filename();
            if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("osstat")) != std::string::npos)){
                textpath = spath.path().string();
            }
        }
    }

    return textpath;
}

std::unordered_map<std::string, std::string> io::gtfs_paths(std::string exe_path){

    std::unordered_map<std::string, std::string> gtfs_paths;

    for (const auto& spath:fs::recursive_directory_iterator(exe_path, fs::directory_options::skip_permission_denied)){
        if (!(spath.is_directory())){
            auto filename = spath.path().filename();
            if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("outes")) != std::string::npos)){
                gtfs_paths.emplace(std::string("routes"), std::string(spath.path().string()));
            } else if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("requen")) != std::string::npos)){
                gtfs_paths.emplace(std::string("frequencies"), std::string(spath.path().string()));
            } else if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("rips")) != std::string::npos)){
                gtfs_paths.emplace(std::string("trips"), std::string(spath.path().string()));
            } else if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("top_time")) != std::string::npos)){
                gtfs_paths.emplace(std::string("stop_times"), std::string(spath.path().string()));
            } else if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("tops")) != std::string::npos)){
                gtfs_paths.emplace(std::string("stops"), std::string(spath.path().string()));
            } else if ((filename.extension() == ".txt") && (filename.stem().string().find(std::string("hapes")) != std::string::npos)){
                gtfs_paths.emplace(std::string("shapes"), std::string(spath.path().string()));
            }
        }
    }
    
    return gtfs_paths;
}