#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

class MapHandler{
public:
    MapHandler(){
        cv::FileStorage historymap_setting("../map/historymap.yaml",
                                           cv::FileStorage::READ);
        historymap_rows = historymap_setting["map_rows"];
        historymap_cols = historymap_setting["map_cols"];
        historymap_center_rows = historymap_setting["center_rows"];
        historymap_center_cols = historymap_setting["center_cols"];
        historymap_resolution = historymap_setting["resolution"];


        ifstream init_pose_file("../map/init.pose");
        if(!init_pose_file.is_open()){
            cerr << "Fail to load init.pose" << endl;
        }
        init_pose_file >> init_utm_t.x() >> init_utm_t.y() >> init_utm_t.z();


        map = cv::imread("../map/map.png", cv::IMREAD_UNCHANGED);
        if(map.data == nullptr){
            cerr << "Fail to load map.pnd" << endl;
        }


        ifstream map_set_file("../map/map_set.txt");
        if(!map_set_file.is_open()){
            cerr << "Fail to load map_set.txt" << endl;
        }
        map_set_file >> orgin_x_in_map >> orgin_y_in_map >> map_resolution;
    }

    void get_current_map(Vector3d& cur_utm, double heading){
        cv::Mat history_map(historymap_rows, historymap_cols,
                            CV_8UC1, cv::Scalar(0));

        cv::Mat map2 = map.clone();

        Vector3d cur_local = utm2local(cur_utm);

        double max_x = historymap_resolution * historymap_center_rows;
        double max_y = historymap_resolution * historymap_center_cols;
        double left_top_x = cur_local.x() + max_x * cos(heading) - max_y * sin(heading);
        double left_top_y = cur_local.y() + max_x * sin(heading) + max_y * cos(heading);

        for(int u = 0;u<historymap_rows;u++){
            for(int v = 0;v<historymap_cols;v++){
                double offset_u = -u * historymap_resolution;
                double offset_v = -v * historymap_resolution;

                Vector3d point_local;
                point_local.x() = left_top_x + offset_u * cos(heading) - offset_v * sin(heading);
                point_local.y() = left_top_y + offset_u * sin(heading) + offset_v * cos(heading);
                point_local.z() = 0;

                Vector3d point_map = local2map(point_local);

                if(point_map.x() < 0 || point_map.x() > map.rows
                || point_map.y() < 0 || point_map.y() > map.cols){
                    history_map.at<uchar>(u, v) = 255;
                }else{
                    Vector2i p11(floor(point_map.x()), floor(point_map.y()));
                    Vector2i p12(floor(point_map.x()), ceil(point_map.y()));
                    Vector2i p21(ceil(point_map.x()), floor(point_map.y()));
                    Vector2i p22(ceil(point_map.x()), ceil(point_map.y()));

                    map2.at<uchar>(p11.x(), p11.y()) = 255;
                    map2.at<uchar>(p12.x(), p12.y()) = 255;
                    map2.at<uchar>(p21.x(), p21.y()) = 255;
                    map2.at<uchar>(p22.x(), p22.y()) = 255;

                    if(map.at<uchar>(p11.x(), p11.y()) != 0
                    || map.at<uchar>(p12.x(), p12.y()) != 0
                    || map.at<uchar>(p21.x(), p21.y()) != 0
                    || map.at<uchar>(p22.x(), p22.y()) != 0)
                        history_map.at<uchar>(u, v) = 255;

                }
            }
        }

        cv::imwrite("map2.png", map2);
        cv::imwrite("history_map.png", history_map);
    }

private:
    Vector3d utm2local(Vector3d& utm){
        Vector3d local;
        local.x() = utm.x() - init_utm_t.x();
        local.y() = utm.y() - init_utm_t.y();
        local.z() = 0;

        return local;
    }

    Vector3d local2map(Vector3d& local){
        Vector3d map;
        map.x() = local.x()/map_resolution + orgin_x_in_map;
        map.y() = local.y()/map_resolution + orgin_y_in_map;
        map.z() = 0;

        return map;
    }

public:
    cv::Mat map;  // map in image format

    // local orgin in image coordinate
    int orgin_x_in_map;
    int orgin_y_in_map;
    double map_resolution;

    // local map origin utm position
    Vector3d init_utm_t;

    // history map settings
    int historymap_rows;
    int historymap_cols;
    int historymap_center_rows;
    int historymap_center_cols;
    double historymap_resolution;
};

int main() {
    double test_x = 288772.8836895769;
    double test_y = 3497206.347378659;

    Vector3d test_utm(test_x, test_y, 0);
    Quaterniond test_quaterniond;
    test_quaterniond.x() = 0;
    test_quaterniond.y() = 0;
    test_quaterniond.z() = 0.2502511625488316;
    test_quaterniond.w() = 0.9681809519108286;

    AngleAxisd rotation_vector(test_quaterniond);
    double test_heading = rotation_vector.angle();

    MapHandler maphandler;
    maphandler.get_current_map(test_utm, test_heading);

    return 0;
}
