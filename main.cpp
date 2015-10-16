#include <memory>
#include <opencv2/opencv.hpp>

#include "src/utils.h"
#include "src/feintrack_params.h"
#include "src/feintrack.h"

////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    //std::string input_file_name = "/home/snuzhny/Documents/automotive/pedestrians_datasets/ikoretsk/video/camera1.mov";
	std::string input_file_name = "0";
    if (argc > 1)
    {
        input_file_name = argv[1];
    }

    cv::VideoCapture capture;
    if (input_file_name.empty())
    {
        capture.open(0);
    }
    else if (input_file_name.size() == 1)
    {
        capture.open(atoi(input_file_name.c_str()));
    }
    else
    {
        capture.open(input_file_name.c_str());
    }

    if (!capture.isOpened())
    {
        std::cerr << "File " <<  input_file_name << " not opened!" << std::endl;
        return 1;
    }
	int framesNum = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_COUNT));

	int fps = static_cast<int>(capture.get(CV_CAP_PROP_FPS));
	if (fps < 1)
	{
		fps = 25;
	}

    int frameWidth = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH));
    int frameHeight = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    auto ftrack = std::shared_ptr<feintrack::CFeinTrack>(new feintrack::CFeinTrack);

    ftrack->set_sensitivity(80);
    ftrack->set_fps(fps);
    ftrack->set_show_objects(true);
    ftrack->set_bgrnd_type(feintrack::norm_back);
    ftrack->set_use_recognition(false);
    ftrack->set_use_morphology(true);
    ftrack->set_show_left_objects(true);
    ftrack->set_show_trajectory(false);
    ftrack->set_selection_time(12);
    ftrack->set_min_region_width(std::max(5, frameWidth / 100));
    ftrack->set_min_region_height(ftrack->get_min_region_width());
    ftrack->set_analyze_area(feintrack::RECT_(0, 100, 0, 100));
    ftrack->set_use_square_segmentation(true);
    ftrack->set_detect_patches_of_sunlight(false);
    ftrack->set_cut_shadows(true);

    ftrack->set_left_object_time1_sec(5);
    ftrack->set_left_object_time2_sec(10);
    ftrack->set_left_object_time3_sec(15);

#if 0
    feintrack::zones_cont zones;

    feintrack::CZone zone;
    zone.left = 0;
    zone.right = frameWidth - 1;
    zone.top = 0;
    zone.bottom = frameHeight / 3;
    zone.uid = 1;
    zone.name = "Zone 1";
    zone.min_obj_width = 5;
    zone.min_obj_height = 5;
    zone.use_detection = true;
    zones.push_back(zone);

    zone.top = zone.bottom;
    zone.bottom = (2 * frameHeight) / 3;
    zone.uid = 2;
    zone.name = "Zone 2";
    zones.push_back(zone);

    zone.top = zone.bottom;
    zone.bottom = frameHeight - 1;
    zone.uid = 3;
    zone.name = "Zone 3";
    zones.push_back(zone);

    ftrack->set_zones_list(zones);
#endif

    feintrack::lines_cont lines;
    ftrack->set_lines_list(lines);

    ftrack->enable_back_update(true);

    ftrack->set_use_cuda(false, 0);


    uint32_t frame_num(0);

    feintrack::color_type cl_type = feintrack::buf_rgb24;

#if ADV_OUT
    cv::Mat adv_img;
#endif

    for (int vc = 0; vc < 3; ++vc)
    {
        frame_num = 0;
        //capture.set(CV_CAP_PROP_POS_FRAMES, 100);

        cv::Mat frame;
        for (capture >> frame; !frame.empty(); capture >> frame)
        {
#if ADV_OUT
            if (adv_img.empty())
            {
                adv_img = cv::Mat(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            }
#endif

            cv::Mat curr_frame;
            switch (cl_type)
            {
            case feintrack::buf_gray:
            {
                cv::cvtColor(frame, curr_frame, CV_RGB2GRAY);
                break;
            }

            case feintrack::buf_rgb32:
            {
                cv::cvtColor(frame, curr_frame, CV_RGB2RGBA);
                break;
            }

            default:
                curr_frame = frame;
                break;
            }

            int64 t1 = cv::getTickCount();

#if !ADV_OUT
			ftrack->new_frame((const uchar*)curr_frame.data, static_cast<uint32_t>(curr_frame.step[0]), curr_frame.cols, curr_frame.rows, cl_type);
#else
            ftrack->new_frame((const uchar*)curr_frame.data, static_cast<uint32_t>(curr_frame.step[0]), curr_frame.cols, curr_frame.rows, cl_type, (uchar*)adv_img.data);
#endif
            int64 t2 = cv::getTickCount();
            double freq = cv::getTickFrequency();


#if 1
            // Обводка объектов
            feintrack::CObjRect *rect_arr;
            size_t rect_count;
            ftrack->get_objects(rect_arr, rect_count);

            if (rect_count && rect_arr)
            {
                cv::Scalar colors[feintrack::max_types] = {
                    cv::Scalar(0, 255, 0),
                    cv::Scalar(0, 0, 0),
                    cv::Scalar(0, 0, 0),
                    cv::Scalar(0, 0, 0),
                    cv::Scalar(0, 0, 0)
                };

                for (size_t i = 0; i < rect_count; ++i)
                {
                    assert(rect_arr[i].type < feintrack::max_types);

                    cv::rectangle(frame, cv::Point(rect_arr[i].left, rect_arr[i].top), cv::Point(rect_arr[i].right, rect_arr[i].bottom), colors[rect_arr[i].type]);

                    // Вывод траектории
                    cv::Point p1(rect_arr[i].traectory[0].x, rect_arr[i].traectory[0].y);
                    cv::Point p2;
                    for (size_t j = 1, stop = rect_arr[i].traectory_size - 1; j < stop; ++j)
                    {
                        p2.x = rect_arr[i].traectory[j].x;
                        p2.y = rect_arr[i].traectory[j].y;

                        cv::line(frame, p1, p2, cv::Scalar(0, 0, 255));
                        p1 = p2;
                    }

#if 1
                    char object_name[256];
                    std::string type_str("u");
                    switch (rect_arr[i].type)
                    {
                    case feintrack::unknown_object: type_str = "u";  break;
                    case feintrack::human:          type_str = "h";  break;
                    case feintrack::vehicle:        type_str = "v";  break;
                    case feintrack::animal:         type_str = "a";  break;
                    case feintrack::humans:         type_str = "hh"; break;
                    case feintrack::max_types:      assert(0);       break;
                    }
                    sprintf(object_name, "%u %s", rect_arr[i].uid, type_str.c_str());
                    cv::putText(frame, object_name, cv::Point(rect_arr[i].left, rect_arr[i].top), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
#endif
                }
            }

            // Обводка оставленных объектов
            feintrack::CLeftObjRect *left_rect_arr;
            size_t left_rect_count;
            ftrack->get_left_objects(left_rect_arr, left_rect_count);

            for (size_t i = 0; i < left_rect_count; ++i)
            {
                switch (left_rect_arr[i].type)
                {
                case feintrack::CLeftObjRect::first:
                    cv::rectangle(frame, cv::Point(left_rect_arr[i].left, left_rect_arr[i].top), cv::Point(left_rect_arr[i].right, left_rect_arr[i].bottom), cv::Scalar(0, 0, 255));
                    break;

                case feintrack::CLeftObjRect::second:
                    cv::rectangle(frame, cv::Point(left_rect_arr[i].left, left_rect_arr[i].top), cv::Point(left_rect_arr[i].right, left_rect_arr[i].bottom), cv::Scalar(255, 0, 255));
                    break;
                }
            }
#endif

            cv::imshow("frame", frame);

#if ADV_OUT
            cv::imshow("adv_img", adv_img);
#endif

            std::cout << "Frame " << frame_num << " of " << framesNum << ": " << rect_count << " objects are tracking at " << ((t2 - t1) / freq) << " sec" << std::endl;

            int workTime = static_cast<int>(1000. * (t2 - t1) / freq);
            int waitTime = (workTime >= 1000 / fps) ? 1 : (1000 / fps - workTime);
            if (cv::waitKey(waitTime) > 0)
            {
                break;
            }

            ++frame_num;
        }
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////
