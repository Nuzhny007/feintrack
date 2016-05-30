#pragma once
#include "utils.h"

/**
***************************************************************************************************
*  StillObjectModel
*
*   @brief
*      Probability model of the lost object
***************************************************************************************************
*/
class StillObjectModel
{
public:
    StillObjectModel()
        :
          m_overlapProb(0)
    {

    }

    ///< Initialization of the object model
    void SetModel(
            cv::Mat objectImage ///< Image with not overlapped lost object
            )
    {
        objectImage.convertTo(m_mean, CV_32FC1);
        m_variance = cv::Mat(objectImage.rows, objectImage.cols, CV_32FC1, cv::Scalar(20));

#if DBG_LOST_WND
        cv::cvtColor(objectImage, m_result, CV_GRAY2BGR);
#endif
    }

    ///< Update lost object model
    void Update(
            cv::Mat objectImage ///< Image with not overlapped lost object
            )
    {
        const float eps = 1.5;
        const float alpha = 0.01;

        size_t sum = 0;
        for (int y = 0; y < objectImage.rows; ++y)
        {
            for (int x = 0; x < objectImage.cols; ++x)
            {
                float v = static_cast<float>(objectImage.at<uchar>(y, x));
                float mean = m_mean.at<float>(y, x);
                float var = m_variance.at<float>(y, x);

                if (fabs(v - mean) < eps * var)
                {
                    mean = (1 - alpha) * mean + alpha * v;
                    var = sqrt((1 - alpha) * sn_utils::sqr(var) + alpha * sn_utils::sqr(v - mean));

                    m_mean.at<float>(y, x) = mean;
                    m_variance.at<float>(y, x) = var;

#if DBG_LOST_WND
                    m_result.at<cv::Vec3b>(y, x) = cv::Vec3b(mean, mean, mean);
#endif
                }
                else
                {
                    ++sum;

#if DBG_LOST_WND
                    m_result.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
#endif
                }
            }
        }

        m_overlapProb = static_cast<float>(sum) / (objectImage.cols * objectImage.rows);
    }

    ///< Get the probability of the event: object was overlapped
    float GetOverlappedProbability() const
    {
        return m_overlapProb;
    }

#if DBG_LOST_WND
    ///< Show current model
    void Visualize(
            cv::Mat dbgFrame ///< [out] Frame to draw
            )
    {
        m_result.copyTo(dbgFrame);
    }
#endif

private:
    cv::Mat m_mean;          ///< Mean luma values for each pixel
    cv::Mat m_variance;      ///< Variance for luma values for each pixel

#if DBG_LOST_WND
    cv::Mat m_result;        ///< Work result
#endif

    float m_overlapProb;     ///< Probability of overlapping the current object
};

/**
***************************************************************************************************
*  StillObjectModelSimple
*
*   @brief
*      Current model of the lost object
***************************************************************************************************
*/
class StillObjectModelSimple
{
public:
    StillObjectModelSimple()
        :
          m_overlapProb(0)
    {

    }

    ///< Initialization of the object model
    void SetModel(
            cv::Mat objectImage ///< Image with not overlapped lost object
            )
    {
        objectImage.copyTo(m_objectModel);

#if DBG_LOST_WND
        objectImage.copyTo(m_result);
#endif
    }

    ///< Update lost object model
    void Update(
            cv::Mat objectImage ///< Image with not overlapped lost object
            )
    {
        cv::Mat diff;
        cv::absdiff(objectImage, m_objectModel, diff);
        cv::threshold(diff, diff, 50, 255, CV_THRESH_BINARY);
        cv::Scalar sum = cv::sum(diff);
        m_overlapProb = sum[0] / (255. * m_objectModel.cols * m_objectModel.rows);

#if DBG_LOST_WND
        diff.copyTo(m_result);
#endif
    }

    ///< Get the probability of the event: object was overlapped
    float GetOverlappedProbability() const
    {
        return m_overlapProb;
    }

#if DBG_LOST_WND
    ///< Show current model
    void Visualize(
            cv::Mat dbgFrame ///< [out] Frame to draw
            )
    {
        m_result.copyTo(dbgFrame);
    }
#endif

private:
    cv::Mat m_objectModel;          ///< Image area with object (Luma)
    float m_overlapProb;            ///< Probability of overlapping the current object

#if DBG_LOST_WND
    cv::Mat m_result;               ///< Work result
#endif
};

/**
***************************************************************************************************
*  LostObject
*
*   @brief
*      Information about lost object detected on the video
***************************************************************************************************
*/
class LostObject
{
public:
    ///< Variable states of the lost objects
    enum States
    {
        Potential,                  ///< Object is not lost now but it may be so in future
        Lost,                       ///< Lost object
        RemovedAsOverlapped,        ///< Lost object that is overlapped by another object a long time
        RemovedWithTimeout          ///< Lost object was be lost a long-long time
    };

    int m_stillFrames;              ///< Frames count when object was motionless
    int m_overlapFrames;            ///< Frames count when object was overlapped by another object

    int m_parentID;                 ///< Identificator of parent object

    cv::Rect m_lastPos;             ///< Last position on frame where was observed motionless object

    StillObjectModel m_objectModel; ///< Updated in time model of the observated object

    States m_state;                 ///< Current state of the lost object

    ///< Default constructor
    LostObject()
        :
          m_stillFrames(0),
          m_overlapFrames(0),
          m_parentID(0),
          m_state(Potential)
    {

    }

    ///< Constructor
    LostObject(
            cv::Mat frameGray,              ///< [in] Frame with object
            int parentID,                   ///< [in] Moving objects unique identifier
            cv::Rect objectRect,            ///< [in] Bounding rectangle
            int stillFrames                 ///< [in] Frames when object was still
            )
        :
          m_stillFrames(stillFrames),
          m_overlapFrames(0),
          m_parentID(parentID),
          m_lastPos(objectRect),
          m_state(Potential)
    {
        if (m_lastPos.x + m_lastPos.width > frameGray.cols - 1)
        {
            m_lastPos.width = frameGray.cols - 1 - m_lastPos.x;
        }
        if (m_lastPos.y + m_lastPos.height > frameGray.rows - 1)
        {
            m_lastPos.height = frameGray.rows - 1 - m_lastPos.y;
        }

        cv::Mat objROI(frameGray, m_lastPos);
        m_objectModel.SetModel(objROI);
    }

    ///< Update lost object model and state
    void Update(
            cv::Mat frameGray   ///< [in] Current frame
        #if DBG_LOST_WND
            , cv::Mat dbgFrame  ///< [in] Copy of frame to debug draw
        #endif
            )
    {
        ++m_stillFrames;

        cv::Mat objROI(frameGray, m_lastPos);
        m_objectModel.Update(objROI);
        if (m_objectModel.GetOverlappedProbability() > 0.3f)
        {
            ++m_overlapFrames;
        }
        else
        {
            m_overlapFrames = 0;
        }
#if DBG_LOST_WND
        Visualize(dbgFrame, true);
#endif
    }

    ///< Visualization
    void Visualize(
            cv::Mat dbgFrame,          ///< [out] Image to debug draw
            bool showDebugInfo = false ///< [in]  Show current object model
            )
    {
        if (showDebugInfo)
        {
#if DBG_LOST_WND
            m_objectModel.Visualize(cv::Mat(dbgFrame, m_lastPos));
#endif
        }

        cv::Scalar color = (m_state == Potential) ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 255);

        cv::rectangle(dbgFrame, m_lastPos, color);

        char text[256];
        sprintf(text, "%i", m_parentID);
        cv::putText(dbgFrame, text, m_lastPos.tl(), cv::FONT_HERSHEY_SIMPLEX, 1.0, color);
    }
};
typedef std::deque<LostObject> lost_cont;
