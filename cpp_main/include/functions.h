#pragma once
#include "utils.h"


int stream_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int time_files_test();


int replay(string folder_name);


int stream_map_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int single_frame_map_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int log_replay(string folder_name, int width=30, int height=30, int res=10);


int pointcloud_debug(int width, int height, int res);


int map_projection_debug(std::shared_ptr<Mike> node, int width, int height, int res, string mode="color");


int delay_test(std::shared_ptr<Mike> node);
