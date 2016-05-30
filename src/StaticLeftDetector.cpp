#include "LostDetector.h"
#include "../common/utils.h"

/**
***************************************************************************************************
*  StaticLeftDetector::StaticLeftDetector
*
*   @brief
*      Constructor
***************************************************************************************************
*/
StaticLeftDetector::StaticLeftDetector()
    :
      m_motionlessFramesThreshold(25),
      m_lostFramesThreshold(125),
      m_overlapFramesThreshold(100),
      m_lostObjectTimeout(250),
      m_maxObjectSize(0, 0)
{
}

/**
***************************************************************************************************
*  StaticLeftDetector::~StaticLeftDetector
*
*   @brief
*      Destructor
***************************************************************************************************
*/
StaticLeftDetector::~StaticLeftDetector()
{

}

/**
***************************************************************************************************
*  StaticLeftDetector::SetParams
*
*   @brief
*      Set environment params
***************************************************************************************************
*/
void StaticLeftDetector::SetParams(
        int fps,
        int motionlessTimeThreshold,
        int lostTimeThreshold,
        int overlapTimeThreshold,
        int lostObjectTimeout,
        cv::Size maxObjectSize)
{
    m_motionlessFramesThreshold = fps * motionlessTimeThreshold;
    m_lostFramesThreshold = fps * lostTimeThreshold;
    m_overlapFramesThreshold = fps * overlapTimeThreshold;
    m_lostObjectTimeout = fps * lostObjectTimeout;
    m_maxObjectSize = maxObjectSize;
}

/**
***************************************************************************************************
*  StaticLeftDetector::Process
*
*   @brief
*      Analyze trajectories of the motion objects and detection a lost objects
***************************************************************************************************
*/
void StaticLeftDetector::Process(
        cv::Mat frameBGR24,           ///< [in] Input frame
        const objects_cont& objects   ///< [in] Detected moving objects on video
        )
{
    m_removedObjects.clear();

    if (objects.empty() && m_lostObjects.empty())
    {
        return;
    }

    if (m_maxObjectSize.width == 0 ||
            m_maxObjectSize.height == 0)
    {
        m_maxObjectSize.width = frameBGR24.cols;
        m_maxObjectSize.height = frameBGR24.rows;
    }

    cv::Mat frameGray;
    cv::cvtColor(frameBGR24, frameGray, CV_BGR2GRAY);

#if DBG_LOST_WND
    cv::Mat dbgFrame;
    frameBGR24.copyTo(dbgFrame);
#endif

    // Update all lost object on current frame
    for (auto lost = m_lostObjects.begin(); lost != m_lostObjects.end(); )
    {
#if DBG_LOST_WND
        lost->Update(frameGray, dbgFrame);
#else
        lost->Update(frameGray);
#endif

        if (lost->m_stillFrames < m_lostObjectTimeout)
        {
            if (lost->m_overlapFrames >= m_overlapFramesThreshold)
            {
                if (lost->m_state == LostObject::Lost)
                {
                    lost->m_state = LostObject::RemovedAsOverlapped;
                    m_removedObjects.push_back(*lost);
                }

#if DBG_LOST_LOG
                std::cout << "Object " << lost->m_parentID << " is overlapped and will be deleted." << std::endl;
#endif

                lost = m_lostObjects.erase(lost);
            }
            else
            {
                if (lost->m_state == LostObject::Potential &&
                        lost->m_stillFrames >= m_lostFramesThreshold &&
                        lost->m_overlapFrames == 0)
                {
                    lost->m_state = LostObject::Lost;
#if DBG_LOST_LOG
                    std::cout << "Object " << lost->m_parentID << " now is Lost!" << std::endl;
#endif
                }
                ++lost;
            }
        }
        else
        {
            if (lost->m_state == LostObject::Lost)
            {
                lost->m_state = LostObject::RemovedWithTimeout;
                m_removedObjects.push_back(*lost);
            }

#if DBG_LOST_LOG
            std::cout << "Object " << lost->m_parentID << " deleted as " << ((lost->m_state == LostObject::Potential) ? "potentional" : "lost") << std::endl;
#endif

            lost = m_lostObjects.erase(lost);
        }
    }

#if DBG_LOST_WND
    cv::imshow("update losts", dbgFrame);
#endif

    const float ANGLE_THRESH = M_PI / 60.f;

    // Detection a potential lost objects - objects that have all trajectories points in small area a short time
    for (auto obj : objects)
    {
        // First: delete potential lost objects that are moving now
        for (auto lost = m_lostObjects.begin(); lost != m_lostObjects.end();)
        {
            bool needRemove = false;

            if (obj.ID == lost->m_parentID)
            {
                float kx;
                float bx;
                float ky;
                float by;
                const int startPos = obj.m_trajectory.size() - m_motionlessFramesThreshold;
                sn_utils::LinearLeastSquares(obj.m_trajectory, startPos, obj.m_trajectory.size(), kx, bx, ky, by);

                if (fabs(atan(kx)) > ANGLE_THRESH ||
                        fabs(atan(ky)) > ANGLE_THRESH)
                {
                    // Remove from Lost objects
                    if (lost->m_state == LostObject::Potential &&
                            lost->m_overlapFrames > 0)
                    {
                        needRemove = true;
                    }
                }
            }

            if (needRemove)
            {
                lost = m_lostObjects.erase(lost);
            }
            else
            {
                ++lost;
            }
        }

        // Second: find a potential lost objects among founded objects
        if (obj.w < m_maxObjectSize.width &&
                obj.h < m_maxObjectSize.height &&
                obj.m_trajectory.size() >= static_cast<size_t>(m_motionlessFramesThreshold))
        {
            float kx;
            float bx;
            float ky;
            float by;
            const int startPos = obj.m_trajectory.size() - m_motionlessFramesThreshold;
            sn_utils::LinearLeastSquares(obj.m_trajectory, startPos, obj.m_trajectory.size(), kx, bx, ky, by);

#if DBG_LOST_LOG
            std::cout << "atan(kx) = " << atan(kx) << ",  atan(ky) = " << atan(ky) << std::endl;
#endif

            if (fabs(atan(kx)) < ANGLE_THRESH &&
                    fabs(atan(ky)) < ANGLE_THRESH)
            {
                // Potential lost object
                // Find it in existing objects list
                bool founded = false;
                for (auto& lost : m_lostObjects)
                {
                    if (obj.ID == lost.m_parentID)
                    {
                        // Update state?

                        founded = true;
                        break;
                    }
                }
                if (!founded)
                {
                    // Create new potential lost object
                    m_lostObjects.push_back(LostObject(frameGray, obj.ID, obj.GetRectangle(), m_motionlessFramesThreshold));

#if DBG_LOST_LOG
                    std::cout << "Object " << obj.ID << " was added to potential lost objects." << std::endl;
#endif
                }
            }
        }
    }
}

/**
***************************************************************************************************
*  StaticLeftDetector::GetObjects
*
*   @brief
*      Get lost objects
***************************************************************************************************
*/
void StaticLeftDetector::GetObjects(
        lost_cont& lostObjects        ///< [out] Result list with lost objects
        ) const
{
    lostObjects.clear();

    for (const auto& lost : m_lostObjects)
    {
        if (lost.m_state == LostObject::Lost)
        {
            lostObjects.push_back(lost);
        }
    }
}

/**
***************************************************************************************************
*  StaticLeftDetector::GetRemovedObjects
*
*   @brief
*      Get removed on last frame objects
***************************************************************************************************
*/
void StaticLeftDetector::GetRemovedObjects(
        lost_cont& lostObjects        ///< [out] Result list with lost objects
        ) const
{
    lostObjects.assign(m_removedObjects.begin(), m_removedObjects.end());
}

