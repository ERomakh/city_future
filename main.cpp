#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <string>
#include <ctime>
#include <map>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <numeric>
#include <future>
#include <climits>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#ifdef __linux__
#include <sstream>
#include <unistd.h>
#endif

#ifdef _WIN32
std::cout << "Only macOS and Linux are supported" << std::endl;
return 0;
#endif

#include <igraph.h>
#include <mlpack.hpp>
#include "cpl_conv.h"

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>

#include "ptmodel.hpp"

using seconds_t = std::chrono::seconds;

namespace fs = std::filesystem;

using namespace mlpack; //
using namespace arma; //
using namespace mlpack::cf; //

int main(){

    CPLSetConfigOption("OGR_INTERLEAVED_READING", "YES");
    CPLSetConfigOption("OGR_GEOMETRY_ACCEPT_UNCLOSED_RING", "NO");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    GDALAllRegister();

    long long tstart = io::time_seconds();
    std::cout << "Время начала работы скрипта: " << io::get_unix_time() << std::endl;

    fs::path exe_lib;

    #ifdef __APPLE__
        char szPath[PATH_MAX];
        uint32_t bufsize = PATH_MAX;
        if (!_NSGetExecutablePath(szPath, &bufsize)){
            exe_lib = fs::path{szPath}.parent_path() / "";
        }

    #endif


    #ifdef __linux__
    char szPath[PATH_MAX];
    ssize_t count = readlink( "/proc/self/exe", szPath, PATH_MAX );
    if( count < 0 || count >= PATH_MAX )
        return {};
    szPath[count] = '\0';
    exe_lib = fs::path{szPath}.parent_path() / "";
    #endif

    std::string pbfpath = io::find_pbf(exe_lib.string());
    if (pbfpath.length() == 0){
        std::cout << "Путь до pbf файла не найден" << std::endl;
        return 0;
    }
    
    std::string shp_genplan = io::find_genplan_shp(exe_lib.string());

    if (shp_genplan.length() == 0){
        std::cout << "Путь до шейпфайла с данными генерального плана не найден" << std::endl;
        return 0;
    }

    std::string textpath = io::find_txt_path(exe_lib.string());

    if (textpath.length() == 0){
        std::cout << "Путь до txt файла с данными генерального плана не найден" << std::endl;
        return 0;
    }

    std::string save_path_blds = exe_lib.string() + "city/save_data/buildings.shp";
    std::string save_density = exe_lib.string() + "city/save_data/densmap.shp";
    std::string save_line_prev = exe_lib.string() + "city/save_data/raw_system.shp";
    std::string save_points_prev = exe_lib.string() + "city/save_data/point_system.shp";
    std::string save_graph_line = exe_lib.string() + "city/save_data/graph_line.shp";
    std::string metro_station_path = exe_lib.string() + "city/metro_system/metro_stations.shp";
    std::string metro_line_path = exe_lib.string() + "city/metro_system/metro_line.shp";
    std::string model_save_path = exe_lib.string() + "city/save_data/solved_md.shp";

    std::string pbf_lines_save = exe_lib.string() + "city/temp_data/graph_roads.shp";
    std::string pbf_buildings_save = exe_lib.string() + "city/temp_data/model_buildings.shp";

    std::string origgtfs = "/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/GTFSEXTENDED";
    // std::string newgtfs = "/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/GTFS_master";
    std::unordered_map<std::string, std::string> gtfs_paths = io::gtfs_paths(origgtfs);
    if (gtfs_paths.size() == 0){
        std::cout << "Данные GTFS не найдены" << std::endl;
        return 0;
    }

    int thread_num = 8;

    // 0 – points; 1 – lines; 2 – multilinestrings; 3 – multipolygons; 4 – other_relations
    // 3437391
    // 1674442

    std::unique_ptr<io::Osm_building> osmbuild = std::make_unique<io::Osm_building>();
    std::unique_ptr<io::Osm_roads> raw_osmroads = std::make_unique<io::Osm_roads>();
    std::unique_ptr<io::Osm_roads> osmroads = std::make_unique<io::Osm_roads>();
    std::unique_ptr<io::Osm_amenities> osm_amenities = std::make_unique<io::Osm_amenities>();

    // read pbf data
    io::open_pbf(pbfpath, osmbuild, raw_osmroads, osm_amenities, 3437391);
    OGRSpatialReference projected_crs = osmbuild->osrs_spatref;
    osmbuild->save_data(pbf_buildings_save, projected_crs);
    raw_osmroads->clear_roads(osmroads);
    osmroads->save_data(pbf_lines_save, projected_crs);
    raw_osmroads.reset();

    std::future<v_doub> amthread = std::async(std::launch::async, preparation::amenity_density, std::ref(osm_amenities), std::ref(osmbuild));
        // Плотность ameneties в радиусе 300 метров от дома

    std::unique_ptr<preparation::Roads> correct_roads = std::make_unique<preparation::Roads>();
    std::unique_ptr<preparation::Buildings> buildings = std::make_unique<preparation::Buildings>();
    std::unique_ptr<preparation::Graph_view> osm_graph = std::make_unique<preparation::Graph_view>();


    // start graph creation
    osmroads->make_array();
    osmroads->spindex_grid();
    correct_roads->change_topolgy(osmroads);
    std::future<std::map<std::tuple<int, int>, std::tuple<std::tuple<int, int>, std::tuple<int, int>, double>>> future_graph = 
                        std::async(std::launch::async, preparation::create_generalized_graph, std::ref(correct_roads));
    std::unique_ptr<gtfs::GTFS_points> gtfs_point_data = std::make_unique<gtfs::GTFS_points>();
    std::unique_ptr<gtfs::GTFS_lines> raw_line_data = std::make_unique<gtfs::GTFS_lines>();
    
    gtfs_point_data->download_gtfs(gtfs_paths.at("stops"), projected_crs);
    gtfs_point_data->make_connection();

    raw_line_data->make_routes(gtfs_paths, projected_crs);
    raw_line_data->calculate_capacity();
    raw_line_data->stop_capacity(gtfs_point_data);

    osmbuild->make_array();
    osm_graph->create_igraph(correct_roads);
    osm_graph->calculate_centrality();

    std::unique_ptr<preparation::Graph_view> gen_graph = std::make_unique<preparation::Graph_view>();
    std::unique_ptr<preparation::Graph_view> model_graph = std::make_unique<preparation::Graph_view>();

    buildings->create_buildings(osmbuild);
    buildings->transfer_centrality(osm_graph, thread_num); // Перенос центральности из одной модели в другую

    v_doub amenity_vector = amthread.get();
    buildings->amenities_density.insert(buildings->amenities_density.begin(), amenity_vector.begin(), amenity_vector.end());

    buildings->find_density(thread_num); // Относительная плотность домов в радиусе 300 метров от текущего дома
    buildings->find_to_center();
    std::unique_ptr<io::Genplan_data> gdata = std::make_unique<io::Genplan_data>();
    gdata->download_poly(shp_genplan);
    gdata->download_txt(textpath);
    buildings->get_from_genplan(gdata);
    buildings->simplify_categ();

    // classification
    std::unique_ptr<cl::Cl_bild> classificated = std::make_unique<cl::Cl_bild>();
    std::unique_ptr<preparation::Buildings> without_fact = std::make_unique<preparation::Buildings>();
    std::unique_ptr<preparation::Buildings> only_house = std::make_unique<preparation::Buildings>();

    std::unique_ptr<cl::Cl_bild> level_class = std::make_unique<cl::Cl_bild>();
    std::unique_ptr<cl::Cl_bild> predicted_levels = std::make_unique<cl::Cl_bild>();

    classificated->logistic_factory(buildings, without_fact, predicted_levels);
    buildings.reset();
    classificated->logistic_office(without_fact, only_house);
    without_fact.reset();
    level_class->house_category(only_house, gdata);
    only_house.reset();
    level_class->fill_levels();
    level_class->linear_regression(predicted_levels, classificated);
    predicted_levels->fill_levels();
    predicted_levels->estimate_population(gdata);
    predicted_levels->estimate_workforce(gdata);

    predicted_levels->save_shapefile(save_path_blds, projected_crs);
    predicted_levels->density_map(save_density, projected_crs, 250);

    // transport model
    std::unique_ptr<transmodel::PT_model> public_model = std::make_unique<transmodel::PT_model>();
    std::unique_ptr<transmodel::Car_model> carg_model = std::make_unique<transmodel::Car_model>();
    std::unique_ptr<transmodel::Settlement_area> living_builds = std::make_unique<transmodel::Settlement_area>();
    std::unique_ptr<transmodel::Working_places> wp_builds = std::make_unique<transmodel::Working_places>();
    std::unique_ptr<transmodel::Result_model> result = std::make_unique<transmodel::Result_model>();

    living_builds->create_settlement(predicted_levels, gdata);
    wp_builds->create_wp(predicted_levels);
    wp_builds->pedestrian_choice(living_builds, thread_num);

    public_model->make_pt_model(raw_line_data, gtfs_point_data);
    living_builds->make_transport_connections(public_model, gtfs_point_data);
    wp_builds->make_transport_connections(public_model, gtfs_point_data);
    wp_builds->make_possible_wp(public_model);
    wp_builds->possible_work_stop(public_model, gtfs_point_data);
    std::cout << "possible_work_stop" << std::endl;
    transmodel::predict_pt_users(public_model, living_builds, wp_builds, gtfs_point_data, thread_num);
    std::cout << "predict_pt_users" << std::endl;
    std::map<std::tuple<int, int>, std::tuple<std::tuple<int, int>, std::tuple<int, int>, double>> graph_data = future_graph.get();
    carg_model->create_model(graph_data);
    std::cout << "car model was created" << std::endl;
    living_builds->car_settlement(carg_model);
    std::cout << "car_settlement" << std::endl;
    wp_builds->workplace_settlement(carg_model);
    std::cout << "workplace_settlement" << std::endl;
    transmodel::car_users(living_builds, carg_model, wp_builds, thread_num);
    std::cout << "car_users" << std::endl;
    living_builds->pt_users_prediction();
    std::cout << "pt_users_prediction" << std::endl;


    public_model->total_route_wp(raw_line_data);
    std::cout << "total_route_wp" << std::endl;
    public_model->total_avaliable(raw_line_data, gtfs_point_data);
    std::cout << "total_avaliable" << std::endl;
    living_builds->stop_selection(public_model, gtfs_point_data);
    std::cout << "stop_selection" << std::endl;

    public_model->model_solve();
    std::cout << "PT model: " << public_model->nodeid_pressure.size() << std::endl;
    std::cout << "model_solve" << std::endl;
    result->transfer_from_pt(public_model, gtfs_point_data);
    std::cout << "Size of result: " << result->entrance_model.size() << std::endl;
    std::cout << "transfer_from_pt" << std::endl;
    result->save_model_data(model_save_path, projected_crs);
    std::cout << "save_model_data" << std::endl;


    // Master plan future
    std::cout << "Start future" << std::endl;

    std::unique_ptr<io::Genplan_data> newgenplan = std::make_unique<io::Genplan_data>();
    newgenplan->download_txt("/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/genplan_extension/Rosstat.txt");
    std::string pathfrom = "/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/genplan_extension/build_from.shp";
    std::string pathto = "/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/genplan_extension/build_to.shp";

    std::string newgtfs = "/Users/evgeniiromakh/Documents/University/Master/Disser/Source/Kazan/GTFS_master";
    std::unordered_map<std::string, std::string> gtfs_paths2 = io::gtfs_paths(newgtfs);
    std::unique_ptr<gtfs::GTFS_points> gtfs_point_future = std::make_unique<gtfs::GTFS_points>();
    std::unique_ptr<gtfs::GTFS_lines> gtfs_future_line_data = std::make_unique<gtfs::GTFS_lines>();

    cl::make_empty_buildings(predicted_levels, gdata, pathfrom, pathto, projected_crs, newgenplan);
    
    gtfs_point_future->download_gtfs(gtfs_paths2.at("stops"), projected_crs);
    gtfs_point_future->make_connection();

    gtfs_future_line_data->make_routes(gtfs_paths2, projected_crs);
    gtfs_future_line_data->calculate_capacity();
    gtfs_future_line_data->stop_capacity(gtfs_point_future);

    std::unique_ptr<transmodel::PT_model> future_public_model = std::make_unique<transmodel::PT_model>();
    std::unique_ptr<transmodel::Car_model> future_car_model = std::make_unique<transmodel::Car_model>();
    std::unique_ptr<transmodel::Settlement_area> future_living_builds = std::make_unique<transmodel::Settlement_area>();
    std::unique_ptr<transmodel::Working_places> future_wp_builds = std::make_unique<transmodel::Working_places>();
    std::unique_ptr<transmodel::Result_model> future_result = std::make_unique<transmodel::Result_model>();

    future_living_builds->create_settlement(predicted_levels, newgenplan);
    future_wp_builds->create_wp(predicted_levels);
    future_wp_builds->pedestrian_choice(future_living_builds, thread_num);

    future_public_model->make_pt_model(gtfs_future_line_data, gtfs_point_future);
    future_living_builds->make_transport_connections(future_public_model, gtfs_point_future);
    future_wp_builds->make_transport_connections(future_public_model, gtfs_point_future);
    future_wp_builds->make_possible_wp(future_public_model);
    future_wp_builds->possible_work_stop(future_public_model, gtfs_point_future);
    std::cout << "possible_work_stop" << std::endl;
    transmodel::predict_pt_users(future_public_model, future_living_builds, future_wp_builds, gtfs_point_future, thread_num);
    std::cout << "predict_pt_users" << std::endl;
    future_car_model->create_model(graph_data);
    std::cout << "car model was created" << std::endl;
    future_living_builds->car_settlement(future_car_model);
    std::cout << "car_settlement" << std::endl;
    future_wp_builds->workplace_settlement(future_car_model);
    std::cout << "workplace_settlement" << std::endl;
    transmodel::car_users(future_living_builds, future_car_model, future_wp_builds, thread_num);
    std::cout << "car_users" << std::endl;
    future_living_builds->pt_users_prediction();
    std::cout << "pt_users_prediction" << std::endl;


    future_public_model->total_route_wp(gtfs_future_line_data);
    std::cout << "total_route_wp" << std::endl;
    future_public_model->total_avaliable(gtfs_future_line_data, gtfs_point_future);
    std::cout << "total_avaliable" << std::endl;
    future_living_builds->stop_selection(future_public_model, gtfs_point_future);
    std::cout << "stop_selection" << std::endl;

    future_public_model->model_solve();
    std::cout << "PT model: " << future_public_model->nodeid_pressure.size() << std::endl;
    std::cout << "model_solve" << std::endl;
    future_result->transfer_from_pt(future_public_model, gtfs_point_future);
    std::cout << "Size of result: " << future_result->entrance_model.size() << std::endl;
    std::cout << "transfer_from_pt" << std::endl;

    std::string future_model_path = exe_lib.string() + "city/save_data/future_solved.shp";
    std::string future_comp_path = exe_lib.string() + "city/save_data/compared_md.shp";
    future_result->save_model_data(future_model_path, projected_crs);
    transmodel::compare_model(future_result, result, future_comp_path, projected_crs);
    std::cout << "save_model_data" << std::endl;

    // save data before recalculation
    // gtfs_point_data->save_prev_points(save_points_prev, projected_crs);
    // std::cout << "save_prev_points" << std::endl;
    // raw_line_data->save_prev_lines(save_line_prev, projected_crs);
    // std::cout << "save_prev_lines" << std::endl;

    long long tend = io::time_seconds();
    long minutes = (tend - tstart)/60;
    float secs = (static_cast<float>(tend - tstart) / static_cast<float>(60) - static_cast<float>(minutes)) * 60;

    std::cout << "Время выполнения скрипта: " << minutes << " минут " << secs << " секунд" << std::endl;

    std::cout << "Время окончания работы скрипта: " << io::get_unix_time() << std::endl;

    return 0;
}