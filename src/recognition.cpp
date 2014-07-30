#include "recognition.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	
	CRecognition::CRecognition()
	{
#ifdef USE_HOG_RECOGNIZE
#if 0
		hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
#else
		hog.winSize.width = 48;
		hog.winSize.height = 96;
		hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
#endif
#endif
	}
	////////////////////////////////////////////////////////////////////////////

	CRecognition::~CRecognition()
	{
	}
	////////////////////////////////////////////////////////////////////////////
	
	bool CRecognition::is_head(int head_height, const hist_cont& h_hist, double& head_area, double& head_center, double& head_r_mu)
	{
		head_center = calc_center_mass(h_hist, head_area);
		head_r_mu = calc_mu(h_hist, head_center, head_area);

		return (head_area > sqrt(head_r_mu) * head_height) &&
			(head_height < 3. * head_r_mu);
	}
	////////////////////////////////////////////////////////////////////////////
	
	bool CRecognition::is_body(int body_height, const hist_cont& h_hist, double head_center, double head_r_mu, double legs_center, double& body_area)
	{
		double body_center = calc_center_mass(h_hist, body_area);
		double body_r_mu = calc_mu(h_hist, body_center, body_area);

		double begin = body_center - sqrt(body_r_mu);
		double end = body_center + sqrt(body_r_mu);

		return in_range(legs_center, begin, end) &&
			in_range(head_center, begin, end) &&
			(body_r_mu > head_r_mu) &&
			(body_area > sqrt(body_r_mu) * body_height) &&
			(body_height < 4. * body_r_mu);
	}
	////////////////////////////////////////////////////////////////////////////

    object_types CRecognition::recognize_object(const CObjectRegion& region, const uchar* buf, uint32_t pitch, uint32_t frame_width, uint32_t frame_height, const mask_type* mask)
	{
		object_types ret_val(unknown_object);

#if 1
		ret_val = is_human(region, buf, pitch, frame_width, frame_height, mask);
		if (ret_val == unknown_object)
		{
			ret_val = is_vehicle(region, frame_width, mask);
			if (ret_val == unknown_object)
				ret_val = is_animal(region, frame_width, mask);
		}
#else
		IplImage img;
		img.nSize = sizeof(IplImage);
		img.nChannels = 3;
		img.depth = IPL_DEPTH_8U;

		img.origin = 1;

		img.BorderMode[0] = img.BorderMode[1] = img.BorderMode[2] = img.BorderMode[3] = 0;
		img.BorderConst[0] = img.BorderConst[1] = img.BorderConst[2] = img.BorderConst[3] = 0;

		img.colorModel[0] = 'R';
		img.colorModel[1] = 'G';
		img.colorModel[2] = 'B';
		img.colorModel[3] = '\0';

		img.channelSeq[0] = 'B';
		img.channelSeq[1] = 'G';
		img.channelSeq[2] = 'R';
		img.channelSeq[3] = '\0';

		img.ID = 0;
		img.dataOrder = 0;
		img.align = 4;
		img.roi = 0;
		img.maskROI = 0;
		img.imageId = 0;
		img.tileInfo = 0;
		img.alphaChannel = 0;

		img.width = frame_width;
		img.height = frame_height;
		img.widthStep = pitch;
		img.imageSize = img.widthStep * img.height;
		img.imageData = const_cast<char* >(reinterpret_cast<const char* >(buf));
		img.imageDataOrigin = img.imageData;

		nnfloat probality_val = 0;
		nnfloat out_vector[3] = { 0.0 };
		CInputImage input_image('u');

		// Создание входного вектора для нейросети
		cvSetImageROI(&img, cvRect(region.get_left(), region.get_top(), region.width(), region.height()));
		cvResize(&img, tmp_img);
		cvResetImageROI(&img);
		const unsigned char* ibuf = (const unsigned char* )tmp_img->imageData;
		for (int y = 0; y < IMG_HEIGHT; ++y)
		{
			for (int x = 0; x < IMG_WIDTH; ++x)
			{
				// 1.0 is white, -1.0 is black
				input_image.data[x + y * (IMG_WIDTH + 1)] = ((nnfloat)RGB_2_Y(ibuf + x + y * tmp_img->widthStep)) / 128. - 1.;
			}
			input_image.data[IMG_WIDTH + y * (IMG_WIDTH + 1)] = 1.;
		}
		for (int x = 0; x < IMG_WIDTH; ++x)
			input_image.data[x + IMG_HEIGHT * (IMG_WIDTH + 1)] = 1.;
		input_image.data[(IMG_HEIGHT + 1) * (IMG_WIDTH + 1) - 1] = (nnfloat)region.width() / (nnfloat)region.height();

		// Распознавание символа
        nn.Calculate(input_image.data, CInputImage::DATA_SIZE, out_vector, 3, nullptr);
		probality_val = 0.5;
		size_t ind = std::numeric_limits<size_t>::max();
		for (size_t i = 0; i < 3; ++i)
		{
			if (probality_val < out_vector[i])
			{
				probality_val = out_vector[i];
				ind = i;
			}
		}
		if (ind != std::numeric_limits<size_t>::max())
		{
			switch (ind)
			{
			case 0: ret_val = vehicle; break;
			case 1: ret_val = human; break;
			case 2: ret_val = unknown; break;
			}
		}
#endif

		return ret_val;
	}
	////////////////////////////////////////////////////////////////////////////
	
    object_types CRecognition::is_vehicle(const CObjectRegion& region, uint32_t frame_width, const mask_type* mask)
	{
#if 1
        frame_width;
        mask;

		double k = (double)region.width() / (double)region.height();
		if ((k > 2.1) && (k < 3.))
			return vehicle;
		else
			return unknown_object;
#else
		// Гистограмма по строкам
		hist_cont horz_hist(region.width(), 0.);
		// Постоение гистограмы
		build_horz_hist(region, 0, region.height(), horz_hist);

		// Гистограмма по столбцам
		hist_cont vert_hist(region.height(), 0.);
		// Постоение гистограмы
		build_vert_hist(region, vert_hist);

		double area;
		double r = calc_center_mass(horz_hist, area);
		double c = calc_center_mass(vert_hist, area);

		double mrr = calc_mu(horz_hist, r, area);
		double mcc = calc_mu(vert_hist, c, area);
		double mrc = calc_s_mu(region, r, c, area);

		double bol = sqrt(8. * (mrr + mcc + sqrt(sqr(mrr - mcc) + 4. * sqr(mrc))));
		double mol = sqrt(8. * (mrr + mcc - sqrt(sqr(mrr - mcc) + 4. * sqr(mrc))));

		double alpha = 0.;
		if (mrr < mcc)
			alpha = atan((-2. * mrc) / (mrr - mcc + sqrt(sqr(mrr - mcc) + 4. * sqr(mrc))));
		else
			alpha = atan((mrr + mcc + sqrt(sqr(mrr - mcc) + 4. * sqr(mrc))) / (-2. * mrc));

		return unknown_object;
#endif
	}
	////////////////////////////////////////////////////////////////////////////
	
    object_types CRecognition::is_animal(const CObjectRegion& region, uint32_t /*frame_width*/, const mask_type* /*mask*/)
	{
		double k = (double)region.width() / (double)region.height();
		if ((k > 1.2) && (k < 1.8))
			return animal;
		else
			return unknown_object;
	}
	////////////////////////////////////////////////////////////////////////////
	
    object_types CRecognition::is_human(const CObjectRegion& region, const uchar* buf, uint32_t pitch, uint32_t frame_width, uint32_t frame_height, const mask_type* mask)
	{
#ifdef USE_HOG_RECOGNIZE
        mask;

		int w_2 = region.width() / 2;
		int left = std::max<int>(0, region.get_left() - w_2);
		int right = std::min<int>(frame_width - 1, region.get_right() + w_2);
		int width = right - left + 1;

		int h_2 = region.height() / 2;
		int top = std::max<int>(0, region.get_top() - h_2);
		int bottom = std::min<int>(frame_height - 1, region.get_bottom() + h_2);
		int height = bottom - top + 1;

		if (width < 48 || height < 96)
			return unknown_object;

		std::vector<cv::Rect> found;
		cv::Mat img(height, width, CV_8UC3, const_cast<uchar*>(buf + top * pitch + left * 3), pitch);
		hog.detectMultiScale(img, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.5, 2);

#if 0
		static int ii = 0;
		char file_name[256];
		sprintf_s(file_name, sizeof(file_name), "D:\\video_bmp\\qqq\\%i.bmp", ii++);
		cv::imwrite(file_name, img);
#endif

		if (found.size() == 0)
			return unknown_object;
		else if (found.size() == 1)
			return human;
		else
			return humans;
#else
        buf;
        pitch;
        frame_height;

		// Небольшие объекты, а также объекты, ширина которых больше высоты, на распознавание не подаются
		if (3 * region.width() > 2 * region.height())
			return unknown_object;

		// Параметры человека
		const double head_k = 0.14;
		const double body_k = 0.42;
		const double legs_k = 0.44;

		int head_height = int(head_k * region.height());
		int body_height = int(body_k * region.height());
		int legs_height = int(legs_k * region.height());

		// Гистограмма по строкам
		hist_cont horz_hist(region.width(), 0.);

		// Постоение гистограмы для области головы
		build_horz_hist(region, 0, head_height, horz_hist, frame_width, mask);
		double head_area, head_center, head_r_mu;
		if (!is_head(head_height, horz_hist, head_area, head_center, head_r_mu))
			return unknown_object;

		fill(horz_hist.begin(), horz_hist.end(), 0);
		// Построение гистограммы для верхней подобласти области ног
		build_horz_hist(region, head_height + body_height + 1, head_height + body_height + 1 + legs_height / 2, horz_hist, frame_width, mask);
		double legs_area1;
		double legs_center1 = calc_center_mass(horz_hist, legs_area1);
		double legs_r_mu1 = calc_mu(horz_hist, legs_center1, legs_area1);
		// Построение гистограммы для нижней подобласти области ног
		build_horz_hist(region, head_height + body_height + 1 + legs_height / 2 + 1, region.height(), horz_hist, frame_width, mask);
		double legs_area2;
		double legs_center2 = calc_center_mass(horz_hist, legs_area2);
		double legs_r_mu2 = calc_mu(horz_hist, legs_center2, legs_area2);
		// Если момент 2-го порядка верхней части больше момента 2-го порядка нижней части - выходим
		if (legs_r_mu1 > legs_r_mu2)
			return unknown_object;

		build_horz_hist(region, head_height + body_height + 1, region.height(), horz_hist, frame_width, mask);
		double legs_area;
		double legs_center = calc_center_mass(horz_hist, legs_area);

		fill(horz_hist.begin(), horz_hist.end(), 0);
		// Построение гистограммы для области туловища
		build_horz_hist(region, head_height + 1, head_height + body_height, horz_hist, frame_width, mask);
		double body_area;
		if (!is_body(body_height, horz_hist, head_center, head_r_mu, legs_center, body_area))
			return unknown_object;

		// Если суммарная площадь объекта мала по отношению к общей площади региона, то результат распознавания принимается как отрицательный
		if (head_area + legs_area + body_area < (region.width() * region.height()) / 4)
			return unknown_object;

		return human;
#endif
	}
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
