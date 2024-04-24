#pragma once
#include "utils.h"


int stream_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int time_files_test();


int replay_from_images(string folder_name, string mode="trajectory");


int stream_map_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int single_frame_map_test(std::shared_ptr<Mike> node, int width=30, int height=30, int res=10);


int replay_from_odometry(string folder_name, int width=30, int height=30, int res=10);


int delay_test(std::shared_ptr<Mike> node);


int field_trip(std::shared_ptr<Mike> node, int width, int height, int res);


int simple_test();


int recording(std::shared_ptr<Mike> node);


int stream_map_test_from_recording(string folder, int width=30, int height=30, int res=10);
 
