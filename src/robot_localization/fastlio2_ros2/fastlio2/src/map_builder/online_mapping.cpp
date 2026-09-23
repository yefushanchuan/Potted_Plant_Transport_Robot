#include "map_builder.h"
#include <Eigen/Eigenvalues>
#include <set>
#include <tuple>

bool LidarProcessor::matchTimedPlane(const V3D& world, agrobot_time::Plane& plane) {
    PointType p{}; p.getVector3fMap() = world.cast<float>();
    PointVec near; std::vector<float> distances;
    const int count = std::max(5, m_config.near_search_num);
    m_ikdtree->Nearest_Search(p, count, near, distances);
    if (int(near.size()) < count || distances.back() > m_config.near_search_radius) return false;
    V3D center = V3D::Zero();
    for (const auto& q : near) center += q.getVector3fMap().cast<double>();
    center /= near.size();
    M3D cov = M3D::Zero();
    for (const auto& q : near) { V3D d=q.getVector3fMap().cast<double>()-center; cov.noalias() += d*d.transpose(); }
    Eigen::SelfAdjointEigenSolver<M3D> eig(cov / near.size());
    if (eig.info()!=Eigen::Success || eig.eigenvalues()(1)<1e-6 ||
        eig.eigenvalues()(0)>0.1*eig.eigenvalues()(1)) return false;
    plane.normal=eig.eigenvectors().col(0); plane.offset=-plane.normal.dot(center);
    for (const auto& q : near)
        if (std::abs(plane.normal.dot(q.getVector3fMap().cast<double>())+plane.offset)>m_config.plane_fitting_tolerance) return false;
    return true;
}

void LidarProcessor::insertTimedCloud(const CloudType::Ptr& body) {
    // Map voxels may use centroids AFTER deskew; observations must keep raw point times.
    CloudType::Ptr down(new CloudType);
    if (m_config.scan_down_sampling_rate > 0) { m_scan_filter.setInputCloud(body); m_scan_filter.filter(*down); }
    else *down=*body;
    auto world=transformCloud(down,r_wl(),t_wl());
    if (!m_ikdtree->Root_Node) initCloudMap(world->points);
    else { trimCloudMap(); m_ikdtree->Add_Points(world->points,true); }
}

void MapBuilder::processOnline(SyncPackage& package) {
    output_ready_=false; time_report_=agrobot_time::Report();
    auto fail=[&](const char* reason) {
        time_report_.reason=reason;
        time_report_.accepted=false;
        time_report_.td=time_filter_->initialized()?time_filter_->state().td:m_config.time_options.initial_td;
        time_report_.td_std=time_filter_->initialized()?std::sqrt(time_filter_->covariance()(18,18)):m_config.time_options.initial_td_std;
    };
    if (online_lost_) { fail("imu_gap_restart_required"); return; }
    const double duration=package.cloud_end_time-package.cloud_start_time;
    if (!std::isfinite(duration) || duration<=0 || duration>m_config.time_options.max_scan_duration) {
        fail("invalid_scan_duration"); return;
    }
    std::vector<agrobot_time::Imu> imu;
    for (const auto& s : package.imus) imu.push_back({s.time,s.acc,s.gyro});
    if (imu.empty() || !package.cloud || package.cloud->empty()) { fail("empty_input"); return; }
    std::vector<agrobot_time::TimedPoint> raw, selected;
    raw.reserve(package.cloud->size());
    std::set<std::tuple<int,int,int>> voxels;
    for (const auto& p : *package.cloud) {
        agrobot_time::TimedPoint q{p.getVector3fMap().cast<double>(),package.cloud_start_time+p.curvature*0.001};
        raw.push_back(q);
        const double leaf=m_config.scan_down_sampling_rate;
        if (leaf<=0 || voxels.emplace(int(std::floor(p.x/leaf)),int(std::floor(p.y/leaf)),int(std::floor(p.z/leaf))).second)
            selected.push_back(q);
    }
    if (selected.size()>size_t(m_config.time_options.max_points)) {
        std::vector<agrobot_time::TimedPoint> limited;
        for (int i=0;i<m_config.time_options.max_points;++i)
            limited.push_back(selected[size_t(i)*selected.size()/m_config.time_options.max_points]);
        selected.swap(limited);
    }
    bool bootstrap=false;
    if (!time_filter_->initialized()) {
        for (const auto& s:imu) if (init_imus_.empty() || s.time>init_imus_.back().time) init_imus_.push_back(s);
        if (int(init_imus_.size())<m_config.imu_init_num || init_imus_.back().time-init_imus_.front().time<m_config.online_init_duration) {
            fail("initializing_stationary_imu"); return;
        }
        V3D acc=V3D::Zero(),gyro=V3D::Zero();
        for (const auto& s:init_imus_) { acc+=s.acc; gyro+=s.gyro; }
        acc/=init_imus_.size(); gyro/=init_imus_.size();
        double av=0,gv=0;
        for (const auto& s:init_imus_) { av+=(s.acc-acc).squaredNorm(); gv+=(s.gyro-gyro).squaredNorm(); }
        if (std::sqrt(av/init_imus_.size())>0.5 || std::sqrt(gv/init_imus_.size())>0.05 ||
            gyro.norm()>0.15 || acc.norm()<8 || acc.norm()>12) {
            init_imus_.clear(); fail("initialization_requires_stationary_si_imu"); return;
        }
        agrobot_time::State seed;
        seed.time=package.cloud_end_time+m_config.time_options.initial_td;
        seed.bg=gyro;
        if (m_config.gravity_align) seed.R=Eigen::Quaterniond::FromTwoVectors(acc,V3D::UnitZ()).toRotationMatrix();
        seed.gravity=-seed.R*acc;
        time_filter_->reset(seed); init_imus_.clear(); bootstrap=true;
    } else {
        if (!time_filter_->predictTo(package.cloud_end_time+time_filter_->state().td,imu)) {
            // Never silently seed a new origin into an existing map after an inertial gap.
            online_lost_=true; fail("imu_gap_restart_required"); return;
        }
        time_report_=time_filter_->update(selected,imu,[this](const V3D& q,agrobot_time::Plane& plane) {
            return m_lidar_processor->matchTimedPlane(q,plane);
        });
        if (!time_report_.accepted) return;
    }
    agrobot_time::State output;
    std::vector<V3D> deskewed;
    if (!time_filter_->poseAt(package.cloud_end_time,imu,output) ||
        !time_filter_->deskew(raw,package.cloud_end_time,imu,deskewed)) {
        fail("output_coverage"); if (bootstrap) online_lost_=true; return;
    }
    auto cloud=std::make_shared<CloudType>(*package.cloud);
    for (size_t i=0;i<deskewed.size();++i) cloud->points[i].getVector3fMap()=deskewed[i].cast<float>();
    // Legacy State is an OUTPUT adapter only. No second filter update.
    auto& state=m_kf->x(); state.r_wi=output.R; state.t_wi=output.p; state.v=output.v;
    state.bg=output.bg; state.ba=output.ba; state.g=output.gravity;
    state.r_il.setIdentity(); state.t_il.setZero();
    package.cloud=cloud;
    m_lidar_processor->insertTimedCloud(cloud);
    m_status=BuilderStatus::MAPPING; output_ready_=true;
    if (bootstrap) { time_report_.reason="map_initialized"; time_report_.accepted=true; }
    time_report_.td=time_filter_->state().td;
    time_report_.td_std=std::sqrt(time_filter_->covariance()(18,18));
}
