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

#include <mlpack.hpp>

#include "ogrsf_frmts.h"
#include <ogr_geometry.h>
#include "classifications.hpp"

// namespace fs = std::filesystem;

// classification

void cl::Cl_bild::logistic_factory(std::unique_ptr<preparation::Buildings>& prepared_bds, std::unique_ptr<preparation::Buildings>& without_factories,
        std::unique_ptr<cl::Cl_bild>& final_data){
    v_doub cat_from_genplan;
    v_doub cat_simpl;
    v_doub amen_d = prepared_bds->amenities_density;
    v_doub center_d = prepared_bds->distance_to_center;

    for (std::string icateg:prepared_bds->symplified_categ){
        if (icateg == "detached_h"){
            cat_simpl.push_back(static_cast<double>(0));
        } else if (icateg == "multi_h"){
            cat_simpl.push_back(static_cast<double>(0));
        } else if (icateg == "commerical"){
            cat_simpl.push_back(static_cast<double>(0));
        } else if (icateg == "factory"){
            cat_simpl.push_back(static_cast<double>(1));
        } else {
            cat_simpl.push_back(static_cast<double>(2));
        }
    }

    for (std::string gen_categ:prepared_bds->genplan_categ){
        if (gen_categ == "res_low"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "res_4_9"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "res_9plus"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "office"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "factory"){
            cat_from_genplan.push_back(static_cast<double>(1));
        } else {
            cat_from_genplan.push_back(static_cast<double>(2));
        }
    }

    v_doub train_gen;
    v_doub train_simpl;
    v_doub train_amenity;
    v_doub train_center;
    std::vector<size_t> lbls;
    v_longlong train_id;

    v_doub test_gen;
    v_doub test_simpl;
    v_doub test_amenity;
    v_doub test_center;
    v_longlong test_id;

    if ((cat_from_genplan.size() == cat_simpl.size()) && (amen_d.size() == center_d.size())){
        for (unsigned long i = 0; i != cat_from_genplan.size(); i++){

            if ((cat_simpl.at(i) == static_cast<double>(1)) && (cat_from_genplan.at(i) == static_cast<double>(1))){

                train_gen.push_back(cat_from_genplan.at(i));
                train_simpl.push_back(cat_simpl.at(i));
                train_amenity.push_back(amen_d.at(i));
                train_center.push_back(center_d.at(i));
                train_id.push_back(prepared_bds->osm_id_vec.at(i));
                // Create factory objects in cl class
                
                OGRPoint centroid_1;
                prepared_bds->ogr_poly_vec.at(i).Centroid(&centroid_1);
                point_vector.push_back(centroid_1);
                category.push_back("factory");
                levels.push_back(prepared_bds->levels.at(i));
                area.push_back(prepared_bds->square_vec.at(i));
                relative_density.push_back(prepared_bds->relative_density.at(i));

                lbls.push_back(static_cast<size_t>(1));

            } else if ((cat_simpl.at(i) == static_cast<double>(0)) && (cat_from_genplan.at(i) == static_cast<double>(0)) 
                                                                    && (prepared_bds->genplan_categ.at(i) != "res_low")){

                train_gen.push_back(cat_from_genplan.at(i));
                train_simpl.push_back(cat_simpl.at(i));
                train_amenity.push_back(amen_d.at(i));
                train_center.push_back(center_d.at(i));
                train_id.push_back(prepared_bds->osm_id_vec.at(i));
                lbls.push_back(static_cast<size_t>(0));
                // Create non factory object in class to continue classify

                without_factories->amenities_density.push_back(prepared_bds->amenities_density.at(i));
                without_factories->bc_buildings.push_back(prepared_bds->bc_buildings.at(i));
                without_factories->building_type_vec.push_back(prepared_bds->building_type_vec.at(i));
                without_factories->cc_buildings.push_back(prepared_bds->cc_buildings.at(i));
                without_factories->distance_to_center.push_back(prepared_bds->distance_to_center.at(i));
                without_factories->genplan_categ.push_back(prepared_bds->genplan_categ.at(i));
                without_factories->levels.push_back(prepared_bds->levels.at(i));
                without_factories->ogr_poly_vec.push_back(prepared_bds->ogr_poly_vec.at(i));
                without_factories->osm_id_vec.push_back(prepared_bds->osm_id_vec.at(i));

                without_factories->poly_id_geom.emplace(prepared_bds->osm_id_vec.at(i), prepared_bds->ogr_poly_vec.at(i));

                without_factories->relative_density.push_back(prepared_bds->relative_density.at(i));
                without_factories->square_vec.push_back(prepared_bds->square_vec.at(i));
                without_factories->symplified_categ.push_back(prepared_bds->symplified_categ.at(i));

            } else if (prepared_bds->genplan_categ.at(i) == "res_low"){
                
                final_data->levels.push_back(static_cast<double>(1));
                
                OGRPoint centroid_1;
                prepared_bds->ogr_poly_vec.at(i).Centroid(&centroid_1);
                final_data->point_vector.push_back(centroid_1);

                final_data->relative_density.push_back(prepared_bds->relative_density.at(i));
                final_data->area.push_back(prepared_bds->square_vec.at(i));
                final_data->category.push_back("low_build");

            } else {

                test_gen.push_back(cat_from_genplan.at(i));
                test_simpl.push_back(cat_simpl.at(i));
                test_amenity.push_back(amen_d.at(i));
                test_center.push_back(center_d.at(i));
                test_id.push_back(prepared_bds->osm_id_vec.at(i));
            }
        }
    } else {
        std::cout << "Something got wrong | classifications.cpp, near 60" << std::endl;
    }

    std::unordered_map<long long, size_t> predict_connection;

    arma::mat test_dataset(4, test_center.size());
    test_dataset.row(0) = arma::conv_to<arma::Row<double>>::from(test_gen);
    test_dataset.row(1) = arma::conv_to<arma::Row<double>>::from(test_simpl);
    test_dataset.row(2) = arma::conv_to<arma::Row<double>>::from(test_amenity);
    test_dataset.row(3) = arma::conv_to<arma::Row<double>>::from(test_center);


    arma::mat factory_train(4, train_simpl.size());
    factory_train.row(0) = arma::conv_to<arma::Row<double>>::from(train_gen);
    factory_train.row(1) = arma::conv_to<arma::Row<double>>::from(train_simpl);
    factory_train.row(2) = arma::conv_to<arma::Row<double>>::from(train_amenity);
    factory_train.row(3) = arma::conv_to<arma::Row<double>>::from(train_center);

    arma::Row<size_t> labels = arma::conv_to<arma::Row<size_t>>::from(lbls);

    mlpack::LogisticRegression lr;
    lr.Train(factory_train, labels);
    arma::Row<size_t> predictions;
    lr.Classify(test_dataset, predictions);

    std::vector<size_t> predicted_val = arma::conv_to<std::vector<size_t>>::from(predictions);

    for (unsigned long i = 0; i != predicted_val.size(); i++){
        predict_connection.emplace(test_id.at(i), predicted_val.at(i));
    }

    for (auto kv_pred:predict_connection){
        if (prepared_bds->poly_id_geom.find(kv_pred.first) != prepared_bds->poly_id_geom.end()){
            if (kv_pred.second == static_cast<size_t>(1)){
                OGRPoint classpoint;
                prepared_bds->poly_id_geom.at(kv_pred.first).Centroid(&classpoint);
                point_vector.push_back(classpoint);
                category.push_back("factory");

                long osmid_index = std::find(prepared_bds->osm_id_vec.begin(), prepared_bds->osm_id_vec.end(), kv_pred.first) - prepared_bds->osm_id_vec.begin();
                levels.push_back(prepared_bds->levels.at(osmid_index));
                area.push_back(prepared_bds->square_vec.at(osmid_index));
                relative_density.push_back(prepared_bds->relative_density.at(osmid_index));
                
            } else {
                long osmid_index = std::find(prepared_bds->osm_id_vec.begin(), prepared_bds->osm_id_vec.end(), kv_pred.first) - prepared_bds->osm_id_vec.begin();

                without_factories->amenities_density.push_back(prepared_bds->amenities_density.at(osmid_index));
                without_factories->bc_buildings.push_back(prepared_bds->bc_buildings.at(osmid_index));
                without_factories->building_type_vec.push_back(prepared_bds->building_type_vec.at(osmid_index));
                without_factories->cc_buildings.push_back(prepared_bds->cc_buildings.at(osmid_index));
                without_factories->distance_to_center.push_back(prepared_bds->distance_to_center.at(osmid_index));
                without_factories->genplan_categ.push_back(prepared_bds->genplan_categ.at(osmid_index));
                without_factories->levels.push_back(prepared_bds->levels.at(osmid_index));
                without_factories->ogr_poly_vec.push_back(prepared_bds->ogr_poly_vec.at(osmid_index));
                without_factories->osm_id_vec.push_back(prepared_bds->osm_id_vec.at(osmid_index));

                without_factories->poly_id_geom.emplace(prepared_bds->osm_id_vec.at(osmid_index), prepared_bds->ogr_poly_vec.at(osmid_index));

                without_factories->relative_density.push_back(prepared_bds->relative_density.at(osmid_index));
                without_factories->square_vec.push_back(prepared_bds->square_vec.at(osmid_index));
                without_factories->symplified_categ.push_back(prepared_bds->symplified_categ.at(osmid_index));
            }
            
        }
    }
}

void cl::Cl_bild::logistic_office(std::unique_ptr<preparation::Buildings>& without_factories, std::unique_ptr<preparation::Buildings>& only_house){

    v_doub cat_from_genplan;
    v_doub cat_simpl;
    v_doub amen_d = without_factories->amenities_density;
    v_doub center_d = without_factories->distance_to_center;
    v_doub local_bc = without_factories->bc_buildings;
    v_doub local_cc = without_factories->cc_buildings;

    for (std::string icateg:without_factories->symplified_categ){
        if (icateg == "detached_h"){
            cat_simpl.push_back(static_cast<double>(0));
        } else if (icateg == "multi_h"){
            cat_simpl.push_back(static_cast<double>(0));
        } else if (icateg == "commerical"){
            cat_simpl.push_back(static_cast<double>(1));
        } else {
            cat_simpl.push_back(static_cast<double>(2));
        }
    }

    for (std::string gen_categ:without_factories->genplan_categ){
        if (gen_categ == "res_low"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "res_4_9"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "res_9plus"){
            cat_from_genplan.push_back(static_cast<double>(0));
        } else if (gen_categ == "office"){
            cat_from_genplan.push_back(static_cast<double>(1));
        } else {
            cat_from_genplan.push_back(static_cast<double>(2));
        }
    }

    v_doub train_gen;
    v_doub train_simpl;
    v_doub train_amenity;
    v_doub train_center;
    v_doub train_bc;
    v_doub train_cc;
    std::vector<size_t> lbls;
    v_longlong train_id;

    v_doub test_gen;
    v_doub test_simpl;
    v_doub test_amenity;
    v_doub test_center;
    v_doub test_bc;
    v_doub test_cc;
    v_longlong test_id;

    if ((cat_from_genplan.size() == cat_simpl.size()) && (amen_d.size() == center_d.size())){
        for (unsigned long i = 0; i != cat_from_genplan.size(); i++){

            if ((cat_simpl.at(i) == 1) && (cat_from_genplan.at(i) == 1)){
                train_gen.push_back(cat_from_genplan.at(i));
                train_simpl.push_back(cat_simpl.at(i));
                train_amenity.push_back(amen_d.at(i));
                train_center.push_back(center_d.at(i));

                train_bc.push_back(local_bc.at(i));
                train_cc.push_back(local_cc.at(i));

                train_id.push_back(without_factories->osm_id_vec.at(i));
                // Create factory objects in cl class
                
                OGRPoint centroid_1;
                without_factories->ogr_poly_vec.at(i).Centroid(&centroid_1);
                point_vector.push_back(centroid_1);
                category.push_back("office");
                levels.push_back(without_factories->levels.at(i));
                area.push_back(without_factories->square_vec.at(i));
                relative_density.push_back(without_factories->relative_density.at(i));

                lbls.push_back(static_cast<size_t>(1));
            } else if ((cat_simpl.at(i) == 0) && (cat_from_genplan.at(i) == 0)){
                train_gen.push_back(cat_from_genplan.at(i));
                train_simpl.push_back(cat_simpl.at(i));
                train_amenity.push_back(amen_d.at(i));
                train_center.push_back(center_d.at(i));
                train_id.push_back(without_factories->osm_id_vec.at(i));
                train_bc.push_back(local_bc.at(i));
                train_cc.push_back(local_cc.at(i));

                only_house->amenities_density.push_back(without_factories->amenities_density.at(i));
                only_house->bc_buildings.push_back(without_factories->bc_buildings.at(i));
                only_house->building_type_vec.push_back(without_factories->building_type_vec.at(i));
                only_house->cc_buildings.push_back(without_factories->cc_buildings.at(i));
                only_house->distance_to_center.push_back(without_factories->distance_to_center.at(i));
                only_house->genplan_categ.push_back(without_factories->genplan_categ.at(i));
                only_house->levels.push_back(without_factories->levels.at(i));
                only_house->ogr_poly_vec.push_back(without_factories->ogr_poly_vec.at(i));
                only_house->osm_id_vec.push_back(without_factories->osm_id_vec.at(i));

                only_house->poly_id_geom.emplace(without_factories->osm_id_vec.at(i), without_factories->ogr_poly_vec.at(i));

                only_house->relative_density.push_back(without_factories->relative_density.at(i));
                only_house->square_vec.push_back(without_factories->square_vec.at(i));
                only_house->symplified_categ.push_back(without_factories->symplified_categ.at(i));

                lbls.push_back(static_cast<size_t>(0));
            } else {
                test_gen.push_back(cat_from_genplan.at(i));
                test_simpl.push_back(cat_simpl.at(i));
                test_amenity.push_back(amen_d.at(i));
                test_center.push_back(center_d.at(i));
                test_id.push_back(without_factories->osm_id_vec.at(i));
                test_bc.push_back(local_bc.at(i));
                test_cc.push_back(local_cc.at(i));
            }
        }
    } else {
        std::cout << "Something got wrong | classifications.cpp, near 260" << std::endl;
    }

    std::unordered_map<long long, size_t> predict_connection;

    arma::mat test_dataset(6, test_center.size());
    test_dataset.row(0) = arma::conv_to<arma::Row<double>>::from(test_gen);
    test_dataset.row(1) = arma::conv_to<arma::Row<double>>::from(test_simpl);
    test_dataset.row(2) = arma::conv_to<arma::Row<double>>::from(test_amenity);
    test_dataset.row(3) = arma::conv_to<arma::Row<double>>::from(test_center);
    test_dataset.row(4) = arma::conv_to<arma::Row<double>>::from(test_bc);
    test_dataset.row(5) = arma::conv_to<arma::Row<double>>::from(test_cc);

    arma::mat office_train(6, train_simpl.size());
    office_train.row(0) = arma::conv_to<arma::Row<double>>::from(train_gen);
    office_train.row(1) = arma::conv_to<arma::Row<double>>::from(train_simpl);
    office_train.row(2) = arma::conv_to<arma::Row<double>>::from(train_amenity);
    office_train.row(3) = arma::conv_to<arma::Row<double>>::from(train_center);
    office_train.row(4) = arma::conv_to<arma::Row<double>>::from(train_bc);
    office_train.row(5) = arma::conv_to<arma::Row<double>>::from(train_cc);

    arma::Row<size_t> labels = arma::conv_to<arma::Row<size_t>>::from(lbls);

    mlpack::LogisticRegression lr;
    lr.Train(office_train, labels);
    arma::Row<size_t> predictions;
    lr.Classify(test_dataset, predictions);

    std::vector<size_t> predicted_val = arma::conv_to<std::vector<size_t>>::from(predictions);

    for (unsigned long i = 0; i != predicted_val.size(); i++){
        predict_connection.emplace(test_id.at(i), predicted_val.at(i));
    }

    for (auto kv_pred:predict_connection){
        if (without_factories->poly_id_geom.find(kv_pred.first) != without_factories->poly_id_geom.end()){
            if (kv_pred.second == static_cast<size_t>(1)){
                OGRPoint classpoint;
                without_factories->poly_id_geom.at(kv_pred.first).Centroid(&classpoint);
                point_vector.push_back(classpoint);

                category.push_back("office");
                long osmid_index = std::find(without_factories->osm_id_vec.begin(), without_factories->osm_id_vec.end(), 
                    kv_pred.first) - without_factories->osm_id_vec.begin();
                levels.push_back(without_factories->levels.at(osmid_index));
                area.push_back(without_factories->square_vec.at(osmid_index));
                relative_density.push_back(without_factories->relative_density.at(osmid_index));
            } else {

                long osmid_index = std::find(without_factories->osm_id_vec.begin(), without_factories->osm_id_vec.end(), 
                    kv_pred.first) - without_factories->osm_id_vec.begin();

                only_house->amenities_density.push_back(without_factories->amenities_density.at(osmid_index));
                only_house->bc_buildings.push_back(without_factories->bc_buildings.at(osmid_index));
                only_house->building_type_vec.push_back(without_factories->building_type_vec.at(osmid_index));
                only_house->cc_buildings.push_back(without_factories->cc_buildings.at(osmid_index));
                only_house->distance_to_center.push_back(without_factories->distance_to_center.at(osmid_index));
                only_house->genplan_categ.push_back(without_factories->genplan_categ.at(osmid_index));
                only_house->levels.push_back(without_factories->levels.at(osmid_index));
                only_house->ogr_poly_vec.push_back(without_factories->ogr_poly_vec.at(osmid_index));
                only_house->osm_id_vec.push_back(without_factories->osm_id_vec.at(osmid_index));

                only_house->poly_id_geom.emplace(without_factories->osm_id_vec.at(osmid_index), without_factories->ogr_poly_vec.at(osmid_index));

                only_house->relative_density.push_back(without_factories->relative_density.at(osmid_index));
                only_house->square_vec.push_back(without_factories->square_vec.at(osmid_index));
                only_house->symplified_categ.push_back(without_factories->symplified_categ.at(osmid_index));
            }
            
        }
    }
}


void cl::Cl_bild::house_category(std::unique_ptr<preparation::Buildings>& only_house, std::unique_ptr<io::Genplan_data>& gendata){

    for (unsigned long i = 0; i != only_house->symplified_categ.size(); i++){
        for (unsigned long j = 0; j != gendata->category_genplan.size(); j++){
            if (only_house->ogr_poly_vec.at(i).Intersects(&gendata->polygon_data.at(j))){
                if (gendata->category_genplan.at(j) == "res_9plus"){
                    category.push_back("high_build");
                    levels.push_back(only_house->levels.at(i));
                    area.push_back(only_house->square_vec.at(i));
                    relative_density.push_back(only_house->relative_density.at(i));

                    OGRPoint classpoint;
                    only_house->poly_id_geom.at(only_house->osm_id_vec.at(i)).Centroid(&classpoint);
                    point_vector.push_back(classpoint);

                } else if (gendata->category_genplan.at(j) == "res_4_9"){
                    category.push_back("med_build");
                    levels.push_back(only_house->levels.at(i));
                    area.push_back(only_house->square_vec.at(i));
                    relative_density.push_back(only_house->relative_density.at(i));

                    OGRPoint classpoint;
                    only_house->poly_id_geom.at(only_house->osm_id_vec.at(i)).Centroid(&classpoint);
                    point_vector.push_back(classpoint);

                } else {
                    category.push_back("low_build");
                    levels.push_back(static_cast<double>(1));
                    area.push_back(only_house->square_vec.at(i));
                    relative_density.push_back(only_house->relative_density.at(i));

                    OGRPoint classpoint;
                    only_house->poly_id_geom.at(only_house->osm_id_vec.at(i)).Centroid(&classpoint);
                    point_vector.push_back(classpoint);

                }
            }
        }
    }
}


void cl::Cl_bild::fill_levels(){
    for (unsigned long i = 0; i != point_vector.size(); i++){
        if (category.at(i) == "factory"){
            levels.at(i) = 0;
        } else if (category.at(i) == "low_build"){
            levels.at(i) = 1;
        }
    }
}


void cl::Cl_bild::linear_regression(std::unique_ptr<cl::Cl_bild>& classificated, std::unique_ptr<cl::Cl_bild>& factory_office){
    
    v_doub train_category;
    v_doub train_levels;
    v_doub train_relative_density;
    v_doub train_area;
    std::vector<OGRPoint> train_point;
    v_string train_cat_string;

    v_doub test_category;
    v_doub test_levels;
    v_doub test_relative_density;
    v_doub test_area;
    v_string test_categ_string;
    std::vector<OGRPoint> test_point;

    for (unsigned long i = 0; i != point_vector.size(); i++){

        if ((category.at(i) == "med_build") && (levels.at(i) != static_cast<double>(-1))){

            train_category.push_back(static_cast<double>(0));
            train_levels.push_back(levels.at(i));
            train_relative_density.push_back(relative_density.at(i));
            train_area.push_back(area.at(i));
            train_point.push_back(point_vector.at(i));

            train_cat_string.push_back(category.at(i));

        } else if ((category.at(i) == "high_build") && (levels.at(i) != static_cast<double>(-1))){

            train_category.push_back(static_cast<double>(1));
            train_levels.push_back(levels.at(i));
            train_relative_density.push_back(relative_density.at(i));
            train_area.push_back(area.at(i));
            train_point.push_back(point_vector.at(i));

            train_cat_string.push_back(category.at(i));

        } else if ((category.at(i) == "office") && (levels.at(i) != static_cast<double>(-1))){

            train_category.push_back(static_cast<double>(2));
            train_levels.push_back(levels.at(i));
            train_relative_density.push_back(relative_density.at(i));
            train_area.push_back(area.at(i));
            train_point.push_back(point_vector.at(i));

            train_cat_string.push_back(category.at(i));

        } else if ((category.at(i) == "med_build") && (levels.at(i) == static_cast<double>(-1))){

            test_category.push_back(static_cast<double>(0));
            test_relative_density.push_back(relative_density.at(i));
            test_area.push_back(area.at(i));
            test_point.push_back(point_vector.at(i));

            test_categ_string.push_back(category.at(i));

        } else if ((category.at(i) == "high_build") && (levels.at(i) == static_cast<double>(-1))){

            test_category.push_back(static_cast<double>(1));
            test_relative_density.push_back(relative_density.at(i));
            test_area.push_back(area.at(i));
            test_point.push_back(point_vector.at(i));

            test_categ_string.push_back(category.at(i));

        } else if ((category.at(i) == "office") && (levels.at(i) == static_cast<double>(-1))){

            test_category.push_back(static_cast<double>(2));
            test_relative_density.push_back(relative_density.at(i));
            test_area.push_back(area.at(i));
            test_point.push_back(point_vector.at(i));

            test_categ_string.push_back(category.at(i));
        }
    }

    arma::rowvec predictions;
    std::vector<double> predicted_val;
    if (train_category.size() != 0){
        arma::mat test_dataset(3, test_area.size());
        test_dataset.row(0) = arma::conv_to<arma::Row<double>>::from(test_category);
        test_dataset.row(1) = arma::conv_to<arma::Row<double>>::from(test_relative_density);
        test_dataset.row(2) = arma::conv_to<arma::Row<double>>::from(test_area);


        arma::mat factory_train(3, train_area.size());
        factory_train.row(0) = arma::conv_to<arma::Row<double>>::from(train_category);
        factory_train.row(1) = arma::conv_to<arma::Row<double>>::from(train_relative_density);
        factory_train.row(2) = arma::conv_to<arma::Row<double>>::from(train_area);

        arma::rowvec responses = arma::conv_to<arma::Row<double>>::from(train_levels);

        mlpack::LinearRegression linear_reg(factory_train, responses);
        linear_reg.Predict(test_dataset, predictions);
        predicted_val = arma::conv_to<std::vector<double>>::from(predictions);
    } else {
        for (unsigned long j = 0; j != test_category.size(); j++){
            if (test_category.at(j) == static_cast<double>(0)){
                predicted_val.push_back(static_cast<double>(5));
            } else if (test_category.at(j) == static_cast<double>(1)){
                predicted_val.push_back(static_cast<double>(10));
            } else if (test_category.at(j) == static_cast<double>(1)){
                predicted_val.push_back(static_cast<double>(5));
            }
        }
    }

    for (unsigned long i = 0; i != predicted_val.size(); i++){

        classificated->point_vector.push_back(test_point.at(i));
        classificated->category.push_back(test_categ_string.at(i));
        classificated->levels.push_back(predicted_val.at(i));
        classificated->area.push_back(test_area.at(i));
    }

    for (unsigned long i = 0; i != train_area.size(); i++){

        classificated->point_vector.push_back(train_point.at(i));
        classificated->category.push_back(train_cat_string.at(i));
        classificated->levels.push_back(train_levels.at(i));
        classificated->area.push_back(train_area.at(i));
    }

    for (unsigned long i = 0; i != factory_office->point_vector.size(); i++){
        classificated->point_vector.push_back(factory_office->point_vector.at(i));
        classificated->category.push_back(factory_office->category.at(i));
        classificated->levels.push_back(factory_office->levels.at(i));
        classificated->area.push_back(factory_office->area.at(i));
    }
}


void cl::Cl_bild::estimate_population(std::unique_ptr<io::Genplan_data>& gendata){

    int total_popul = gendata->workforce_distrib.at("population");
    double total_volume = static_cast<double>(0);

    for (unsigned long i = 0; i != point_vector.size(); i++){
        if (category.at(i) == "office"){
            double volume_b = (abs(levels.at(i)) * area.at(i)) / static_cast<double>(5);
            total_volume += volume_b;
        } else {
            double volume_b = abs(levels.at(i)) * area.at(i);
            total_volume += volume_b;
        }
        
    }

    for (unsigned long i = 0; i != point_vector.size(); i++){
        if (category.at(i) == "office"){
            double volume_b = (abs(levels.at(i)) * area.at(i)) / static_cast<double>(5);
            double current_pop = abs((volume_b / total_volume) * static_cast<double>(total_popul));
            population.push_back(current_pop);
        } else {
            double volume_b = levels.at(i) * area.at(i);
            double current_pop = abs((volume_b / total_volume) * static_cast<double>(total_popul));
            population.push_back(current_pop);
        }
    }
}

void cl::Cl_bild::estimate_workforce(std::unique_ptr<io::Genplan_data>& gendata){

    int factory_workers = gendata->workforce_distrib.at("factory_workers");
    int total_workers = gendata->workforce_distrib.at("total_workers") - factory_workers;

    double total_work_volume = static_cast<double>(0);
    double total_factory_volume = static_cast<double>(0);

    for (unsigned long i = 0; i != point_vector.size(); i++){
        if (category.at(i) == "office"){
            total_work_volume += (area.at(i) * abs(levels.at(i)));
        } else if (category.at(i) == "high_build"){
            total_work_volume += area.at(i) / static_cast<double>(4);
        } else if (category.at(i) == "med_build"){
            total_work_volume += area.at(i) / static_cast<double>(4);
        } else if (category.at(i) == "factory"){
            total_factory_volume += area.at(i);
        }
    }

    workplaces.assign(point_vector.size(), 0);

    for (unsigned long i = 0; i != point_vector.size(); i++){

        if (category.at(i) == "office"){
            workplaces.at(i) = abs(((area.at(i) * levels.at(i)) / total_work_volume) * static_cast<double>(total_workers));
        } else if (category.at(i) == "high_build"){
            workplaces.at(i) = abs(((area.at(i) / static_cast<double>(4)) / total_work_volume) * static_cast<double>(total_workers));
        } else if (category.at(i) == "med_build"){
            workplaces.at(i) = abs(((area.at(i) / static_cast<double>(4)) / total_work_volume) * static_cast<double>(total_workers));
        } else if (category.at(i) == "low_build"){
            workplaces.at(i) = abs(((area.at(i) / static_cast<double>(4)) / total_work_volume) * static_cast<double>(total_workers));
        } else if (category.at(i) == "factory"){
            workplaces.at(i) = abs((area.at(i) / total_factory_volume) * static_cast<double>(factory_workers));
        }
    }
}


void cl::Cl_bild::save_shapefile(std::string save_path, OGRSpatialReference srs_d){
    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(save_path.data(), 0, 0, 0,
    GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbPoint, NULL);

    OGRFieldDefn popul_field("Population", OFTReal);
    OGRFieldDefn workplace_field("Workplaces", OFTReal);
    OGRFieldDefn categ_field("Category", OFTString);

    feature_layer->CreateField(&popul_field);
    feature_layer->CreateField(&workplace_field);
    feature_layer->CreateField(&categ_field);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < point_vector.size(); i++){
        OGRFeature *point_feature;
        point_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        point_feature->SetField("Population", population.at(i));
        point_feature->SetField("Workplaces", workplaces.at(i));
        point_feature->SetField("Category", category.at(i).data());

        point_feature->SetGeometry(&point_vector.at(i));
        feature_layer->CreateFeature(point_feature);
        OGRFeature::DestroyFeature(point_feature);
    }
    GDALClose(dataset);
}


void cl::Cl_bild::density_map(std::string fishnet_path, OGRSpatialReference srs_d, int sq_size_len){ // В метрах

    v_doub x_coord;
    v_doub y_coord;

    std::vector<OGRPolygon> square_polygons;
    v_doub pop_inside;
    v_doub work_inside;

    for (OGRPoint pvr:point_vector){
        x_coord.push_back(pvr.getX());
        y_coord.push_back(pvr.getY());
    }

    double min_x = *std::min_element(x_coord.begin(), x_coord.end());
    double max_x = *std::max_element(x_coord.begin(), x_coord.end());
    double min_y = *std::min_element(y_coord.begin(), y_coord.end());
    double max_y = *std::max_element(y_coord.begin(), y_coord.end());

    int change_x = (max_x - min_x) / sq_size_len + 2;
    int change_y = (max_y - min_y) / sq_size_len + 2;

    double base_xcoord = min_x;
    double base_xcoord_max = min_x + static_cast<double>(sq_size_len);

    for (int x = 0; x < change_x; x++){

        double base_ycoord = min_y;
        double base_ycoord_max = min_y + static_cast<double>(sq_size_len);

        for (int y = 0; y < change_y; y++){

            std::unique_ptr<OGRLinearRing> linring = std::make_unique<OGRLinearRing>();
            linring->setPoint(0, base_xcoord, base_ycoord);
            linring->setPoint(1, base_xcoord, base_ycoord_max);
            linring->setPoint(2, base_xcoord_max, base_ycoord_max);
            linring->setPoint(3, base_xcoord_max, base_ycoord);
            linring->setPoint(4, base_xcoord, base_ycoord);
            OGRPolygon* opoly = new OGRPolygon;
            opoly->addRing(linring->toCurve());
            square_polygons.push_back(*opoly);
            base_ycoord += static_cast<double>(sq_size_len);
            base_ycoord_max += static_cast<double>(sq_size_len);
            delete opoly;
        }

        base_xcoord += static_cast<double>(sq_size_len);
        base_xcoord_max += static_cast<double>(sq_size_len);
    }

    for (OGRPolygon opoly:square_polygons){

        double local_population = 0;
        double local_workforce = 0;

        for (unsigned long i = 0; i != point_vector.size(); i++){

            if (opoly.Intersects(&point_vector.at(i))){
                local_population += population.at(i);
                local_workforce += workplaces.at(i);
            }
        }

        pop_inside.push_back(local_population / (static_cast<double>(pow(sq_size_len, 2)) / static_cast<double>(1000000)));
        work_inside.push_back(local_workforce / (static_cast<double>(pow(sq_size_len, 2)) / static_cast<double>(1000000)));
    }

    // save fishnet
    const char *driver_name = "ESRI Shapefile";
    GDALDriver *gdal_driver;
    gdal_driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    GDALDataset *dataset;
    dataset = gdal_driver->Create(fishnet_path.data(), 0, 0, 0,
    GDT_Unknown, NULL);

    if (dataset == NULL) {
        std::cout << "Не получилось создать шейпфайл" << std::endl;
        GDALClose(dataset);
        return;
    }

    OGRLayer *feature_layer;
    feature_layer = dataset->CreateLayer("outpoint", &srs_d, wkbPolygon, NULL);

    OGRFieldDefn popul_field("Population", OFTReal);
    OGRFieldDefn workplace_field("Workplaces", OFTReal);

    feature_layer->CreateField(&popul_field);
    feature_layer->CreateField(&workplace_field);

    char* recode(int nEncoding = 0);
    
    for (ssize_t i = 0; i < square_polygons.size(); i++){
        OGRFeature *polygon_feature;
        polygon_feature = OGRFeature::CreateFeature(feature_layer->GetLayerDefn());
        polygon_feature->SetField("Population", pop_inside.at(i));
        polygon_feature->SetField("Workplaces", work_inside.at(i));

        polygon_feature->SetGeometry(&square_polygons.at(i));
        feature_layer->CreateFeature(polygon_feature);
        OGRFeature::DestroyFeature(polygon_feature);
    }
    GDALClose(dataset);
}