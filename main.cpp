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

#include "gtfs_data.hpp"

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

    std::unordered_map<std::string, std::string> gtfs_paths = io::gtfs_paths(exe_lib);
    if (gtfs_paths.size() == 0){
        std::cout << "Данные GTFS не найдены" << std::endl;
        return 0;
    }

    int thread_num = 8;

    // 0 – points; 1 – lines; 2 – multilinestrings; 3 – multipolygons; 4 – other_relations
    // 3437391
    // 1674442

    std::unique_ptr<io::Osm_building> osmbuild = std::make_unique<io::Osm_building>();
    std::unique_ptr<io::Osm_roads> osmroads = std::make_unique<io::Osm_roads>();
    std::unique_ptr<io::Osm_amenities> osm_amenities = std::make_unique<io::Osm_amenities>();

    io::open_pbf(pbfpath, osmbuild, osmroads, osm_amenities, 3437391);

    std::future<v_doub> amthread = std::async(std::launch::async, preparation::amenity_density, std::ref(osm_amenities), std::ref(osmbuild));
        // Плотность ameneties в радиусе 300 метров от дома

    std::unique_ptr<preparation::Roads> correct_roads = std::make_unique<preparation::Roads>();
    std::unique_ptr<preparation::Buildings> buildings = std::make_unique<preparation::Buildings>();
    std::unique_ptr<preparation::Graph_view> osm_graph = std::make_unique<preparation::Graph_view>();

    OGRSpatialReference projected_crs = osmbuild->osrs_spatref;

    std::unique_ptr<gtfs::GTFS_points> gtfs_point_data = std::make_unique<gtfs::GTFS_points>();
    std::unique_ptr<gtfs::GTFS_lines> raw_line_data = std::make_unique<gtfs::GTFS_lines>();
    gtfs_point_data->download_gtfs(gtfs_paths.at("stops"), projected_crs);
    gtfs_point_data->save_prev_points(save_points_prev, projected_crs);

    raw_line_data->make_routes(gtfs_paths, projected_crs);
    raw_line_data->save_prev_lines(save_line_prev, projected_crs);

    osmbuild->make_array();

    osmroads->make_array();
    osmroads->spindex_grid();
    
    
    correct_roads->change_topolgy(osmroads);
    osm_graph->create_igraph(correct_roads);
    osm_graph->calculate_centrality();

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

    long long tend = io::time_seconds();
    long minutes = (tend - tstart)/60;
    float secs = (static_cast<float>(tend - tstart) / static_cast<float>(60) - static_cast<float>(minutes)) * 60;


    std::cout << "Levels size: " << osmbuild->levels.size() << std::endl;
    std::cout << "ID: " << osmbuild->osm_id_vec.size() << std::endl;

    std::cout << "Время выполнения скрипта: " << minutes << " минут " << secs << " секунд" << std::endl;

    std::cout << "Время окончания работы скрипта: " << io::get_unix_time() << std::endl;

    return 0;
}