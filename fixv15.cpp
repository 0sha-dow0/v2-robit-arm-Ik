std::vector<Eigen::Matrix<double, 5, 1>> seeds;
seeds.push_back(last_solution_);
seeds.push_back(current_joints_);
double target_yaw = atan2(target_pos(1), target_pos(0));
Eigen::Matrix<double, 5, 1> yaw_seed = current_joints_;
yaw_seed(0) = target_yaw;
seeds.push_back(yaw_seed);
Eigen::Matrix<double, 5, 1> z; z.setZero(); seeds.push_back(z);