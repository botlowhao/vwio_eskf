#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <vector>

class VisualOdometryOptimizer
{

public:
    VisualOdometryOptimizer(size_t window_size) : window_size_(window_size)
    {
        // Initialize sliding window
        visual_odometry_window_.reserve(window_size_);
    }

    void addVisualOdometry(const Eigen::Vector3d &visual_odom)
    {
        // delete the old message of vo if the sliding window was filled
        if (visual_odometry_window_.size() > window_size_)
            visual_odometry_window_.erase(visual_odometry_window_.begin());

        // add the newest message of vo
        visual_odometry_window_.push_back(visual_odom);
    }

    void optimize(const WIO_Data &wio_data, nav_msgs::Odometry &vo)
    {
        // define the Ceres Solver Optimize problem
        ceres::Problem problem;

        // add Optimize variable
        double position[3];
        position[0] = wio_data.position[0];
        position[1] = wio_data.position[1];
        position[2] = 0.0;

        problem.AddParameterBlock(position, 3);

        // Add residual term
        for (size_t i = 0; i < visual_odometry_window_.size(); ++i)
        {
            ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<VisualOdometryCostFunctor, 3, 3>(
                    new VisualOdometryCostFunctor(visual_odometry_window_[i]));

            problem.AddResidualBlock(cost_function, nullptr, position);
        }

        // Set the ceres solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        // Run optimization
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Check if optimization was successful
        if (summary.IsSolutionUsable())
        {
            // Visual odometry data update
            vo.pose.pose.position.x = position[0];
            vo.pose.pose.position.y = position[1];
            vo.pose.pose.position.z = 0.0;
            ROS_INFO("\033[1;32mOptimization successful!\033[0m");
        }
        else
        {
            ROS_INFO("\033[1;32mOptimization failed!\033[0m");
        }
    }

private:
    struct VisualOdometryCostFunctor
    {
        VisualOdometryCostFunctor(const Eigen::Vector3d &target) : target_(target) {}

        template <typename T>
        bool operator()(const T *const position, T *residual) const
        {
            residual[0] = position[0] - T(target_[0]);
            residual[1] = position[1] - T(target_[1]);
            residual[2] = position[2] - T(0.0);
            return true;
        }

    private:
        const Eigen::Vector3d target_;
    };

    size_t window_size_;
    std::vector<Eigen::Vector3d> visual_odometry_window_;
};
