// Build piecewise cubic references whose coefficients match their final published durations.
#include "trajectory_generation/reference_path.h"
#include <numeric>
#include <cmath>

namespace {
// Bound position/velocity display arrays to 600 samples while covering the complete physical duration.
std::vector<double> referenceSampleTimes(const std::vector<double>& durations, double nominal_dt) {
    if (durations.empty()) return {};
    const double total = std::accumulate(durations.begin(), durations.end(), 0.0);
    if (!std::isfinite(total) || total <= 0 || !std::isfinite(nominal_dt) || nominal_dt <= 0) return {};
    const size_t count = static_cast<size_t>(std::min(600.0, std::max(2.0, std::ceil(total/nominal_dt)+1.0)));
    std::vector<double> samples;
    samples.reserve(count);
    for (size_t i=0; i<count; ++i) samples.push_back(total*(static_cast<double>(i)/(count-1)));
    return samples;
}
}  // namespace




Refenecesmooth::~Refenecesmooth(){}

void Refenecesmooth::init(std::shared_ptr<GlobalMap> &_global_map){
    global_map = _global_map;
}

void Refenecesmooth::setGlobalPath(Eigen::Vector3d velocity, std::vector<Eigen::Vector2d>& global_path, double reference_amax, double desire_speed, bool xtl)
{
    m_global_path = global_path;
    max_accleration = reference_amax;
    ROS_WARN("[Reference] path size : %d", (int)global_path.size());

    // Defensive: clamp absurd velocities (e.g. from uninitialized memory)
    const double MAX_SANE_VEL = 20.0;  // m/s — well above any real robot speed
    for (int k = 0; k < 3; k++) {
        if (!std::isfinite(velocity[k]) || std::abs(velocity[k]) > MAX_SANE_VEL) {
            ROS_WARN("[Reference] clamping insane state_vel[%d] = %e → 0", k, velocity[k]);
            velocity[k] = 0.0;
        }
    }
    state_vel = velocity;
    desire_veloity = desire_speed;
    isxtl = xtl;
}

void Refenecesmooth::reset(const Eigen::Matrix3d &headState,
           const Eigen::Matrix3d &tailState,
           const int &pieceNum)
{
    N = pieceNum;
    headPVA = headState;
    tailPVA = tailState;
    m_bandedMatrix.create(N+1, 1, 1);
    b.resize(N+1, 1);
    b2.resize(N+1, 1);
    m_polyMatrix_x.resize(N, 4);
    m_polyMatrix_y.resize(N, 4);
    return;
}

void Refenecesmooth::reset(const int &pieceNum) {
    N = pieceNum;
    m_bandedMatrix.create(N + 1, 1, 1);
    b.resize(N + 1, 1);
    b2.resize(N + 1, 1);
    m_polyMatrix_x.resize(N, 4);
    m_polyMatrix_y.resize(N, 4);
    return;
}

void Refenecesmooth::solveTrapezoidalTime()
{
    double path_length = 0;
    if(m_global_path.size()<2){
        traj_length = 0.0;
        ROS_ERROR("[Reference] global_path size < 2, No path");
        return;
    }
    m_trapezoidal_time.clear();
    for(int i = 0; i < m_global_path.size() - 1; i++)
    {
        double pz = 0.0;
        int idx, idy, idz, idx_end, idy_end, idz_end;
        global_map->coord2gridIndex(m_global_path[i].x(), m_global_path[i].y(), pz, idx, idy, idz);
        global_map->coord2gridIndex(m_global_path[i+1].x(), m_global_path[i+1].y(), pz, idx_end, idy_end, idz_end);

        double path_distance = (double)std::sqrt(pow(m_global_path[i+1].x() - m_global_path[i].x(), 2) + pow(m_global_path[i+1].y() - m_global_path[i].y(), 2));
        double height = global_map->GridNodeMap[idx_end][idy_end]->height;
        double slope = abs(height - global_map->GridNodeMap[idx][idy]->height)/ path_distance;

        int sample_num = path_distance / 0.3;  // 采样间隔0.3来确定坡道
        bool exist_height_change = false;
        for(int j = 0; j <= sample_num; j++){
            Eigen::Vector3d sample_point;
            sample_point.x() = m_global_path[i].x() + 0.3 * j * (m_global_path[i+1].x() - m_global_path[i].x()) / path_distance;
            sample_point.y() = m_global_path[i].y() + 0.3 * j * (m_global_path[i+1].y() - m_global_path[i].y()) / path_distance;
            Eigen::Vector3i sample_index = global_map->coord2gridIndex(sample_point);

            if((global_map->GridNodeMap[idx][idy]->height - height) > 0.05){   // 判断是否存在高度变化 TODO 暂时设置为false，即为上坡减速
                exist_height_change = true;
            }
        }

        double slope_cof = slope_coeff;
        double bridge_cof = bridge_coeff;
        double slope_cof_max = slope_coeff_max;
        if(isxtl){
            slope_cof = slope_coeff_xtl;
            bridge_cof = bridge_coeff_xtl;
            slope_cof_max = slope_coeff_xtl_max;
        }

        path_length += path_distance;
        traj_length = path_length;
        double time = 0.3 / desire_veloity;
        if(global_map->GridNodeMap[idx][idy]->exist_second_height ||
           global_map->GridNodeMap[idx_end][idy_end]->exist_second_height){
            time = time * bridge_cof;  // 如果存在过桥洞的段直接减速！
        }else if(exist_height_change){  // 只针对下坡减速
            double ratio = (slope * slope_cof) > slope_cof_max ? slope_cof_max: (slope * slope_cof);
            time = time * (1.0 + ratio);
        }
        m_trapezoidal_time.push_back(time);
    }
//    for(int i = 0; i < m_global_path.size() - 1; i++){
//        std::cout<<"m_trapezoidal_time: "<<m_trapezoidal_time[i]<<std::endl;
//    }
    ROS_WARN("[Reference] path_length: %f", path_length);
}

void Refenecesmooth::solvePolyMatrix()
{
    if (m_global_path.size() < 2) {
        ROS_ERROR("[Reference] global_path size < 2, stop solving！");
        return;
    }

    ROS_DEBUG("[Reference] solvePolyMatrix ENTER: N=%d, path_size=%zu, trap_time_size=%zu",
             (int)(m_global_path.size()-1), m_global_path.size(), m_trapezoidal_time.size());
    // Print first two waypoints and first trapezoidal time
    ROS_DEBUG("[Reference]   path[0]=(%.4f,%.4f) path[1]=(%.4f,%.4f) t[0]=%.6f state_vel=(%.4f,%.4f)",
             m_global_path[0].x(), m_global_path[0].y(),
             m_global_path[1].x(), m_global_path[1].y(),
             m_trapezoidal_time[0], state_vel[0], state_vel[1]);

    reset(m_global_path.size() - 1);
    m_bandedMatrix.reset();
    m_bandedMatrix(0, 0) = 2 * m_trapezoidal_time[0];
    m_bandedMatrix(0, 1) = m_trapezoidal_time[0];
    for(int i = 1 ;i < N ;i++)
    {
        m_bandedMatrix(i, i - 1) = m_trapezoidal_time[i - 1];
        m_bandedMatrix(i, i) = 2 * (m_trapezoidal_time[i - 1] + m_trapezoidal_time[i]);
        m_bandedMatrix(i, i + 1) = m_trapezoidal_time[i];
    }

    m_bandedMatrix(N, N - 1) = m_trapezoidal_time[N - 1];
    m_bandedMatrix(N, N) = 2 * m_trapezoidal_time[N - 1];

    b(0, 0) = 6 * (((m_global_path[1].x() - m_global_path[0].x()) / m_trapezoidal_time[0]) - state_vel[0]);
    b(N, 0) = -6 * ((m_global_path[N].x() - m_global_path[N - 1].x()) / m_trapezoidal_time[N - 1]);
//    b(0, 0) = 0.0;
//    b(N, 0) = 0.0;

    for(int i = 1; i < N; i++)
    {
        b(i, 0) = 6 * ((m_global_path[i + 1].x() - m_global_path[i].x()) / m_trapezoidal_time[i] - (m_global_path[i].x() - m_global_path[i - 1].x()) / m_trapezoidal_time[i - 1]);
    }

    b2(0, 0) = 6 * (((m_global_path[1].y() - m_global_path[0].y()) / m_trapezoidal_time[0]) - state_vel[1]);
    b2(N, 0) = -6 * ((m_global_path[N].y() - m_global_path[N - 1].y()) / m_trapezoidal_time[N - 1]);

//    b2(0, 0) = 0.0;
//    b2(N, 0) = 0.0;

    for (int i = 1; i < N; i++) {
        b2(i, 0) = 6 * ((m_global_path[i + 1].y() - m_global_path[i].y()) / m_trapezoidal_time[i] -
                       (m_global_path[i].y() - m_global_path[i - 1].y()) / m_trapezoidal_time[i - 1]);
    }

    m_bandedMatrix.factorizeLU();
    m_bandedMatrix.solve(b);
    m_bandedMatrix.solve(b2);

    // ── NaN diagnostic: check spline second-derivative solve results ──
    ROS_DEBUG("[Reference] banded solve done. b[0]=%f b[N]=%f b2[0]=%f b2[N]=%f",
             b(0,0), b(N,0), b2(0,0), b2(N,0));
    for (int i = 0; i <= N; i++) {
        if (!std::isfinite(b(i, 0)) || !std::isfinite(b2(i, 0))) {
            ROS_ERROR("[Reference] NaN in spline solve: b(%d)=%f  b2(%d)=%f", i, b(i,0), i, b2(i,0));
        }
    }

    // 解得的中间变量矩阵m1m2,用于求解系数矩阵 y = a + bx + cx2 + dx3
    for(int i = 0; i < N; i++){
        if (m_trapezoidal_time[i] <= 1e-15) {
            ROS_ERROR("[Reference] ZERO trapezoidal_time[%d] = %e — clamping to 0.01", i, m_trapezoidal_time[i]);
            m_trapezoidal_time[i] = 0.01;  // prevent division by zero
        }
        m_polyMatrix_x(i, 0) = (b(i + 1, 0) - b(i, 0)) / (6 * m_trapezoidal_time[i]);
        m_polyMatrix_x(i, 1) = b(i, 0) / 2;
        m_polyMatrix_x(i, 2) = (m_global_path[i + 1].x() - m_global_path[i].x()) / m_trapezoidal_time[i] - m_trapezoidal_time[i] * (2 * b(i, 0) + b(i + 1, 0)) / 6;
        m_polyMatrix_x(i, 3) = m_global_path[i].x();

        m_polyMatrix_y(i, 0) = (b2(i + 1, 0) - b2(i, 0)) / (6 * m_trapezoidal_time[i]);
        m_polyMatrix_y(i, 1) = b2(i, 0) / 2;
        m_polyMatrix_y(i, 2) = (m_global_path[i + 1].y() - m_global_path[i].y()) / m_trapezoidal_time[i] -
                               m_trapezoidal_time[i] * (2 * b2(i, 0) + b2(i + 1, 0)) / 6;
        m_polyMatrix_y(i, 3) = m_global_path[i].y();

        // ── NaN diagnostic: check polynomial coefficient output ──
        bool row_has_nan = false;
        for (int c = 0; c < 4; c++) {
            if (!std::isfinite(m_polyMatrix_x(i, c)) || !std::isfinite(m_polyMatrix_y(i, c))) {
                row_has_nan = true;
            }
        }
        if (row_has_nan) {
            ROS_ERROR("[Reference] NaN in polyMatrix row %d: x=[%f %f %f %f] y=[%f %f %f %f] t=%f",
                      i, m_polyMatrix_x(i,0), m_polyMatrix_x(i,1), m_polyMatrix_x(i,2), m_polyMatrix_x(i,3),
                      m_polyMatrix_y(i,0), m_polyMatrix_y(i,1), m_polyMatrix_y(i,2), m_polyMatrix_y(i,3),
                      m_trapezoidal_time[i]);
        }
    }
    ROS_DEBUG("[Reference] solvePolyMatrix EXIT: polyX[0]=[%f %f %f %f]",
             m_polyMatrix_x(0,0), m_polyMatrix_x(0,1), m_polyMatrix_x(0,2), m_polyMatrix_x(0,3));
    return;
}

// Allocate durations and converge before exporting; failed feasibility produces no usable trajectory.
void Refenecesmooth::getRefTrajectory(std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<double> &times)
{
    ref_trajectory.clear();
    reference_path.clear();
    reference_velocity.clear();
    // Clear every exported representation together so a failure cannot reuse an older solution.
    const auto clear_solution = [this]() {
        m_trapezoidal_time.clear();
        m_polyMatrix_x.resize(0, 4);
        m_polyMatrix_y.resize(0, 4);
        traj_length = 0.0;
    };
    if (m_global_path.size() < 2 || !std::isfinite(max_accleration) || max_accleration <= 0 ||
        !std::isfinite(dt) || dt <= 0) {
        clear_solution();
        ROS_ERROR("[Reference] invalid path or reference limits");
        return;
    }
    if (times.size() == m_global_path.size() - 1) {
        m_trapezoidal_time = times;
    } else {
        solveTrapezoidalTime();
    }
    for (double duration : m_trapezoidal_time) {
        if (!std::isfinite(duration) || duration <= 0) {
            clear_solution();
            ROS_ERROR("[Reference] invalid segment duration");
            return;
        }
    }

    bool feasible = false;
    for (int iteration = 0; iteration < 64; ++iteration) {
        solvePolyMatrix();
        if (!m_polyMatrix_x.allFinite() || !m_polyMatrix_y.allFinite()) break;
        // This exact endpoint check may extend times. Only the no-change branch
        // may export the coefficients, which then match those same durations.
        if (!checkfeasible()) {
            feasible = true;
            break;
        }
    }
    if (!feasible) {
        clear_solution();
        ROS_ERROR("[Reference] acceleration feasibility did not converge; reject trajectory");
        return;
    }

    // Limit visualization allocations, never shorten physical trajectory time to
    // fit the display budget: shortening would violate the just-checked limits.
    for (double sample_time : referenceSampleTimes(m_trapezoidal_time, dt)) {
        int index;
        double offset;
        getSegmentIndex(sample_time, index, offset);
        const double t = sample_time - offset;
        Eigen::Vector3d point;
        point.x() = ((m_polyMatrix_x(index,0)*t+m_polyMatrix_x(index,1))*t+m_polyMatrix_x(index,2))*t+m_polyMatrix_x(index,3);
        point.y() = ((m_polyMatrix_y(index,0)*t+m_polyMatrix_y(index,1))*t+m_polyMatrix_y(index,2))*t+m_polyMatrix_y(index,3);
        point.z() = 0.0;
        ref_trajectory.push_back(point);
        reference_path.push_back(point);
    }
}

// Sample velocity at the same bounded visualization times as reference positions.
void Refenecesmooth::getRefVel()
{
    reference_velocity.clear();
    for (double sample_time : referenceSampleTimes(m_trapezoidal_time, dt)) {
        int index;
        double offset;
        getSegmentIndex(sample_time, index, offset);
        const double t = sample_time - offset;
        reference_velocity.emplace_back(
            (3*m_polyMatrix_x(index,0)*t+2*m_polyMatrix_x(index,1))*t+m_polyMatrix_x(index,2),
            (3*m_polyMatrix_y(index,0)*t+2*m_polyMatrix_y(index,1))*t+m_polyMatrix_y(index,2), 0.0);
    }
}

// Cubic acceleration is affine, so its norm peaks at an endpoint; mark each neighbor at most once.
bool Refenecesmooth::checkfeasible()
{
    std::vector<bool> extend(m_trapezoidal_time.size(), false);
    bool changed = false;
    for (int i = 0; i < static_cast<int>(m_trapezoidal_time.size()); ++i) {
        const double t = m_trapezoidal_time[i];
        const double start_acc = std::hypot(2*m_polyMatrix_x(i,1), 2*m_polyMatrix_y(i,1));
        const double end_acc = std::hypot(6*m_polyMatrix_x(i,0)*t+2*m_polyMatrix_x(i,1),
                                          6*m_polyMatrix_y(i,0)*t+2*m_polyMatrix_y(i,1));
        if (std::max(start_acc, end_acc) > max_accleration) {
            changed = true;
            extend[i] = true;
            if (i > 0) extend[i-1] = true;
            if (i+1 < static_cast<int>(extend.size())) extend[i+1] = true;
        }
    }
    for (size_t i = 0; i < extend.size(); ++i)
        if (extend[i]) m_trapezoidal_time[i] *= 1.1;
    return changed;
}

void Refenecesmooth::getSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < m_trapezoidal_time.size(); i++) {
        sum_time += m_trapezoidal_time[i];
        if (sum_time >= time) {
            segment_index = i;
            total_time = sum_time - m_trapezoidal_time[i];
            break;
        }
    }
}
