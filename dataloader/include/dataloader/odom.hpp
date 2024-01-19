#ifndef FUSION_ROS_ODOM_HPP
#define FUSION_ROS_ODOM_HPP

#include <memory>

namespace Fusion{
    class ODOM{
    public:
        ODOM(double timestamp, double speed): timestamp_(timestamp), speed_(speed){}
        double timestamp_;
        double speed_;
    };

    using ODOMPtr = std::shared_ptr<ODOM>;
}

#endif //FUSION_ROS_ODOM_HPP
