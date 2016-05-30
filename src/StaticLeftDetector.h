#pragma once
#include "LeftObject.h"

/**
***************************************************************************************************
*  StaticLeftDetector
*
*   @brief
*      Detection lost objects on frame
***************************************************************************************************
*/
class StaticLeftDetector
{
public:
    ///< Constructor
    StaticLeftDetector();
    ///< Destructor
    ~StaticLeftDetector();

    // Set environment params
    void SetParams(int fps,
                   int motionlessTimeThreshold,
                   int lostTimeThreshold,
                   int overlapTimeThreshold,
                   int lostObjectTimeout,
                   cv::Size maxObjectSize);

    // Analyze trajectories of the motion objects and detection a lost objects
    void Process(cv::Mat frameBGR24, const objects_cont& objects);

    // Get lost objects
    void GetObjects(lost_cont& lostObjects) const;

    // Get removed on last frame objects
    void GetRemovedObjects(lost_cont& lostObjects) const;

    ///< Get needed points of the objects trajectory
    int GetAnalyzetrajectorySize() const
    {
        return m_motionlessFramesThreshold;
    }

    ///< Get frames count after that the motionless object will be added to the lost objects
    int GetMinFramesForLostObject() const
    {
        return m_lostFramesThreshold;
    }

private:
    lost_cont m_lostObjects;         ///< Founded lost objects
    lost_cont m_removedObjects;      ///< Lost objects which was removed at the last frame

    int m_motionlessFramesThreshold; ///< Frames count after that the moving objects will be added to the potential lost objects
    int m_lostFramesThreshold;       ///< Frames count after that the motionless object will be added to the lost objects
    int m_overlapFramesThreshold;    ///< Frames count after that the overlapped lost object will be deleted from lost objects list
    int m_lostObjectTimeout;         ///< Frames count after that the lost object will be deleted from lost objects list

    cv::Size m_maxObjectSize;        ///< Maximum size of the lost object (for filtration vehicles, people etc)
};
