
#include <opencv2/core/version.hpp>

#if (CV_VERSION_EPOCH > 2)
#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <opencv2/photo.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/contrib.hpp>
#include <opencv2/ml.hpp>

#include <opencv2/core/ocl.hpp>

#else // (CV_VERSION_EPOCH > 2)

#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>

//#include <opencv2/photo/photo.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/ml/ml.hpp>

//#include <opencv2/ocl/ocl.hpp>

#endif

#include "src/utils.h"
#include "src/feintrack_dll.h"

////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    std::string input_file_name = "/home/snuzhny/Documents/automotive/pedestrians_datasets/ikoretsk/video/camera1.mov";
    if (argc > 1)
    {
        input_file_name = argv[1];
    }

    cv::VideoCapture capture;
    if (input_file_name.empty())
    {
        capture.open(0);
    }
    else
    {
        capture.open(input_file_name.c_str());
    }
    if (!capture.isOpened())
    {
        std::cout << "File " <<  input_file_name << " not opened!" << std::endl;
        return 1;
    }
#if (CV_VERSION_EPOCH > 2)
    int framesNum = static_cast<int>(cv::CAP_PROP_FRAME_COUNT);
#else
    int framesNum = static_cast<int>(CV_CAP_PROP_FRAME_COUNT);
#endif

    void* feintrack = vl_feintrack::AddFeintrack();
    vl_feintrack::CFeintrackParams ftp;
    vl_feintrack::GetFeintrackConfigStruct(feintrack, &ftp);

    ftp.set_fps(25);
    ftp.set_show_objects(true);
    ftp.set_min_region_width(5);
    ftp.set_min_region_height(5);
    ftp.set_left_object_time1_sec(15);
    ftp.set_left_object_time2_sec(30);
    ftp.set_left_object_time3_sec(60);
    ftp.set_show_left_objects(true);
    ftp.set_use_square_segmentation(true);
    ftp.set_detect_patches_of_sunlight(false);
    ftp.set_cut_shadows(false);
    ftp.set_analyze_area(vl_feintrack::RECT_(0, 100, 0, 100));
    ftp.set_sensitivity(80);
    ftp.set_use_recognition(false);
    ftp.set_use_morphology(true);
    ftp.set_selection_time(12);
    ftp.set_show_trajectory(false);
    ftp.set_use_cuda(false, 0);
#if 1
    ftp.set_bgrnd_type(vl_feintrack::norm_back);
#else
    ftp.set_bgrnd_type(vl_feintrack::gaussian_mixture);
#endif

    vl_feintrack::SetFeintrackConfigStruct(feintrack, &ftp);

    vl_feintrack::EnableBackUpdate(feintrack, true);


    unsigned long frame_num(0);

    vl_feintrack::color_type cl_type = vl_feintrack::buf_rgb24;

    bool init_zones = false;

#ifdef ADV_OUT
    cv::Mat adv_img;
#endif

    cv::Mat frame;
    for (;;)
    {
        capture >> frame;
        if (frame.empty())
        {
            break;
        }
#ifdef ADV_OUT
        if (adv_img.empty())
        {
            adv_img = cv::Mat(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        }
#endif

        if (!init_zones)
        {
#if 0
            vl_feintrack::zones_cont zones;

            vl_feintrack::CZone zone;
            zone.left = 0;
            zone.right = frame->width - 1;
            zone.top = 0;
            zone.bottom = frame->height / 3;
            zone.uid = 1;
            zone.name = "Zone 1";
            zone.min_obj_width = 5;
            zone.min_obj_height = 5;
            zone.use_detection = true;
            zones.push_back(zone);

            zone.top = zone.bottom;
            zone.bottom = (2 * frame->height) / 3;
            zone.uid = 2;
            zone.name = "Zone 2";
            zones.push_back(zone);

            zone.top = zone.bottom;
            zone.bottom = frame->height - 1;
            zone.uid = 3;
            zone.name = "Zone 3";
            zones.push_back(zone);

            ftp.set_zones(zones);

            vl_feintrack::SetFeintrackConfigStruct(feintrack, &ftp);
#endif

            init_zones = true;
        }

        cv::Mat curr_frame;
        switch (cl_type)
        {
        case vl_feintrack::buf_gray:
        {
            cv::cvtColor(frame, curr_frame, CV_RGB2GRAY);
            break;
        }

        case vl_feintrack::buf_rgb32:
        {
            cv::cvtColor(frame, curr_frame, CV_RGB2RGBA);
            break;
        }

        default:
            curr_frame = frame;
            break;
        }

        int64 t1 = cv::getTickCount();
#ifndef ADV_OUT
        FeintrackFrameAnalyze(feintrack, (const uchar*)curr_frame.data, curr_frame.cols, curr_frame.rows, cl_type);
#else
        FeintrackFrameAnalyze(feintrack, (const uchar*)curr_frame.data, curr_frame.cols, curr_frame.rows, cl_type, (uchar*)adv_img.data);
#endif
        int64 t2 = cv::getTickCount();
        double freq = cv::getTickFrequency();


#if 1
        // Обводка объектов
        vl_feintrack::CObjRect *rect_arr;
        size_t rect_count;
        GetObjects(feintrack, rect_arr, rect_count);

        if (rect_count && rect_arr)
        {
            cv::Scalar colors[5] = {
                cv::Scalar(0, 255, 0),
                cv::Scalar(0, 0, 0),
                cv::Scalar(0, 0, 0),
                cv::Scalar(0, 0, 0),
                cv::Scalar(0, 0, 0)
            };

            for (size_t i = 0; i < rect_count; ++i)
            {
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
                case vl_feintrack::unknown_object: type_str = "u";  break;
                case vl_feintrack::human:          type_str = "h";  break;
                case vl_feintrack::vehicle:        type_str = "v";  break;
                case vl_feintrack::animal:         type_str = "a";  break;
                case vl_feintrack::humans:         type_str = "hh"; break;
                }
                sprintf(object_name, "%u %s", rect_arr[i].uid, type_str.c_str());
                cv::putText(frame, object_name, cv::Point(rect_arr[i].left, rect_arr[i].top), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
#endif
            }
        }

        // Обводка оставленных объектов
        vl_feintrack::CLeftObjRect *left_rect_arr;
        size_t left_rect_count;
        vl_feintrack::GetLeftObjects(feintrack, left_rect_arr, left_rect_count);

        for (size_t i = 0; i < left_rect_count; ++i)
        {
            switch (left_rect_arr[i].type)
            {
            case vl_feintrack::CLeftObjRect::first:
                cv::rectangle(frame, cv::Point(left_rect_arr[i].left, left_rect_arr[i].top), cv::Point(left_rect_arr[i].right, left_rect_arr[i].bottom), cv::Scalar(0, 0, 255));
                break;

            case vl_feintrack::CLeftObjRect::second:
                cv::rectangle(frame, cv::Point(left_rect_arr[i].left, left_rect_arr[i].top), cv::Point(left_rect_arr[i].right, left_rect_arr[i].bottom), cv::Scalar(255, 0, 255));
                break;
            }
        }
#endif

        cv::imshow("frame", frame);

#ifdef ADV_OUT
        cv::imshow("adv_img", adv_img);
#endif

        std::cout << "Frame " << frame_num << " of " << framesNum << ": " << rect_count << " objects are tracking at " << ((t2 - t1) / freq) << " sec" << std::endl;

        int workTime = static_cast<int>(1000. * (t2 - t1) / freq);
        int waitTime = (workTime >= 40) ? 1 : (40 - workTime);
        if (cv::waitKey(waitTime) > 0)
            break;

        ++frame_num;
    }

    vl_feintrack::DelFeintrack(feintrack);
    return 0;
}
////////////////////////////////////////////////////////////////////////////
