#include "../include/pb_core/pbnbv.h"
#include "../include/pb_core/jsonparser.hpp"

#include <glog/logging.h>


bool update_flag_ = false;
std::string pcd_file_path;
std::string pcd_file_path_tmp = "/root/work_place/pb_nbv/src/pb_core/cache/point_cloud_";
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
int nbv_cnt = 0;

int main(int argc, char** argv){

    // 初始化Glog
    google::InitGoogleLogging(argv[0]);

    // 设置日志的输出文件
    google::SetLogDestination(google::INFO, "/root/work_place/MMR/log/");

    // 设置日志的输出级别
    FLAGS_stderrthreshold = google::INFO;

    pbnbv pb_nbv;

    Eigen::Matrix4d nbv;

    std::thread loop_thread([&]() {
        while (true)
        {
            char c = getchar();
            if (c == 'q')
            {
                LOG(INFO) << "exit";
                exit(0);
            }else{
                if (nbv_cnt > 0){
                    pcd_file_path = pcd_file_path_tmp + std::to_string(nbv_cnt) + ".pcd";
                    // pcl 读取点云文件
                    pcl::io::loadPCDFile(pcd_file_path, *pcl_cloud);

                    pb_nbv.capture(pcl_cloud, nbv);
                    pb_nbv.execute(nbv);

                }else if (nbv_cnt == 0){
                    pb_nbv.execute(nbv);
                }
                nbv_cnt++;
            }
        }
    });

    loop_thread.detach();

    pb_nbv.visualization_start();

    
}