#ifndef _IMU_PROCESSING_HPP
#define _IMU_PROCESSING_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <lidar_imu_init/States.h>
#include <geometry_msgs/Vector3.h>
#include "sophus/se3.hpp"
#include "sophus/interpolate.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_R_LI_cov(const V3D &R_LI_cov);
  void set_T_LI_cov(const V3D &T_LI_cov);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_mean_acc_norm(const double &mean_acc_norm);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr pcl_un_);
  void undistort_without_imu(StatesGroup &state_inout, PointCloudXYZI::Ptr pcl_out);
  void undistort_without_imu(StatesGroup & state_begin, StatesGroup &state_end, PointCloudXYZI& pcl_out);
  Eigen::Matrix4d delta_T = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_t = Eigen::Vector3d::Zero();


    //  ros::NodeHandle nh;
  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_R_LI;
  V3D cov_T_LI;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int    lidar_type;
  bool   imu_en;
  bool   undistort_iter;
  bool LI_init_done = false;
  double IMU_mean_acc_norm;

 private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  void propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_in_out);
  void Forward_propagation_without_imu(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_lidar_end_time_;
  double time_last_scan;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)
{
  imu_en = true;
  undistort_iter = true;
  init_iter_num = 1;
  cov_acc         = V3D(0.1, 0.1, 0.1);
  cov_gyr         = V3D(0.1, 0.1, 0.1);
  cov_R_LI        = V3D(0.00001, 0.00001, 0.00001);
  cov_T_LI        = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_gyr    = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc    = V3D(0.0001, 0.0001, 0.0001);
  mean_acc        = V3D(0, 0, -1.0);
  mean_gyr        = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  last_imu_.reset(new sensor_msgs::Imu());
  fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}


void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_R_LI_cov(const V3D &R_LI_cov)
{
    cov_R_LI = R_LI_cov;
}

void ImuProcess::set_T_LI_cov(const V3D &T_LI_cov)
{
    cov_T_LI = T_LI_cov;
}

void ImuProcess::set_mean_acc_norm(const double &mean_acc_norm){
    IMU_mean_acc_norm = mean_acc_norm;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurements to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N ++;
  }

  state_inout.gravity = - mean_acc / mean_acc.norm() * G_m_s2;
  state_inout.rot_end = Eye3d;
  state_inout.bias_g.setZero();
  last_imu_ = meas.imu.back();
}



void ImuProcess::Forward_propagation_without_imu(const MeasureGroup &meas, StatesGroup &state_inout,
                             PointCloudXYZI &pcl_out) {
    pcl_out = *(meas.lidar);
    StatesGroup state_begin = state_inout;
    /*** sort point clouds by offset time ***/
    const double &pcl_beg_time = meas.lidar_beg_time;
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

    MD(DIM_STATE, DIM_STATE) F_x, cov_w;
    double dt = 0.0;

    if (b_first_frame_) {
        dt = 0.1;
        b_first_frame_ = false;
    } else {
        dt = pcl_beg_time - time_last_scan;
        time_last_scan = pcl_beg_time;
    }

    /* covariance propagation */
    F_x.setIdentity();
    cov_w.setZero();
    /** In CV model, bias_g represents angular velocity **/
    /** In CV model，bias_a represents linear acceleration **/
    M3D Exp_f = Exp(state_inout.bias_g, dt);
    F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
    F_x.block<3, 3>(0, 15) = Eye3d * dt;
    F_x.block<3, 3>(3, 12) = Eye3d * dt;

    cov_w.block<3, 3>(15, 15).diagonal() = cov_gyr_scale ;
    cov_w.block<3, 3>(12, 12).diagonal() = cov_acc_scale ;

    /** Forward propagation of covariance**/
    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w ;

    /** Forward propagation of attitude **/
    state_inout.rot_end = state_inout.rot_end * Exp_f;

    /** Position Propagation **/
    state_inout.pos_end += state_inout.vel_end * dt;
    cout << "==================" << endl;
    cout << "current vel = " << state_inout.vel_end.transpose() << endl;
    cout << "current pos = " << state_inout.pos_end.transpose() << endl;
    cout << "last vel    = " << state_begin.vel_end.transpose() << endl;
    cout << "last pose   = " << state_begin.pos_end.transpose() << endl;
    cout << "dt          = " << dt << endl;
    cout << "==================" << endl;



    /**CV model： un-distort pcl using linear interpolation **/
    if(!undistort_iter)
    {
        undistort_without_imu(state_begin, state_inout, pcl_out);
    }

}

void ImuProcess::undistort_without_imu(StatesGroup &state_inout, PointCloudXYZI::Ptr pcl_out)
{
    if(lidar_type != L515)
    {
        sort(pcl_out->points.begin(), pcl_out->points.end(), time_list);
        const double &pcl_end_offset_time = pcl_out->points.back().curvature / double(1000);
        auto it_pcl = pcl_out->points.end() - 1;
        double dt_j = 0.0;
        for(; it_pcl != pcl_out->points.begin(); it_pcl --)
        {
            dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
            M3D R_jk(Exp(state_inout.bias_g, - dt_j));
            V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
            // Using rotation and translation to un-distort points
            V3D p_jk;
            p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

            V3D P_compensate =  R_jk * P_j + p_jk;

            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);
        }
    }
}

void ImuProcess::undistort_without_imu(StatesGroup & state_begin, StatesGroup &state_end, PointCloudXYZI& pcl_out)
{
    if(lidar_type != L515)
    {
        Sophus::SE3d T_begin(state_begin.rot_end, state_begin.pos_end);
        Sophus::SE3d T_end(state_end.rot_end, state_end.pos_end);

        sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
        const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);
        const double &pcl_begin_offset_time = pcl_out.points.front().curvature / double(1000);
        double dt = pcl_end_offset_time - pcl_begin_offset_time;
        auto it_pcl = pcl_out.points.end() - 1;
        double dt_j = 0.0;

        for(; it_pcl != pcl_out.points.begin(); it_pcl --)
        {
            V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
            dt_j= it_pcl->curvature/double(1000) - pcl_begin_offset_time;
            double scale = dt_j / dt;
            Sophus::SE3d T_j = Sophus::interpolate(T_begin, T_end, scale);
            Sophus::SE3d T_kj = T_end.inverse() * T_j;

            V3D P_compensate = T_kj * P_j;

            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);
        }
    }
}

void ImuProcess::propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the current frame-head ***/
  pcl_out = *(meas.lidar);

  StatesGroup state_begin = state_inout;
  cout << "use cv model" << endl;

  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  double imu_end_time = v_imu.back()->header.stamp.toSec();
  double pcl_beg_time, pcl_end_time;

  if (lidar_type == L515)
  {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }
  else
  {
    pcl_beg_time = meas.lidar_beg_time;
    /*** sort point clouds by offset time ***/
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  }


  /*** Initialize IMU pose ***/
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

  /*** forward propagation at each imu point ***/
  V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  
  double dt = 0;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);


    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      V3D angvel_now(head->angular_velocity.x, head->angular_velocity.y, head->angular_velocity.z);
      V3D acc_now(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
      fout_imu << setw(10) << head->header.stamp.toSec() << "  " << angvel_now.transpose()<< " " << acc_now.transpose() << endl;

    angvel_avr -= state_inout.bias_g;
    acc_avr     = acc_avr / IMU_mean_acc_norm * G_m_s2 - state_inout.bias_a;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
        dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    else
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    
    /* covariance propagation */
    M3D acc_avr_skew;
    M3D Exp_f   = Exp(angvel_avr, dt);
    acc_avr_skew<<SKEW_SYM_MATRX(acc_avr);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3,3>(0,0)  = Exp(angvel_avr, -dt);
    F_x.block<3,3>(0,15)  = - Eye3d * dt;
    F_x.block<3,3>(3,12)  = Eye3d * dt;
    F_x.block<3,3>(12,0)  = - R_imu * acc_avr_skew * dt;
    F_x.block<3,3>(12,18) = - R_imu * dt;
    F_x.block<3,3>(12,21) = Eye3d * dt;

    cov_w.block<3,3>(0,0).diagonal()   = cov_gyr * dt * dt;
    cov_w.block<3,3>(6,6).diagonal()   = cov_R_LI * dt * dt;
    cov_w.block<3,3>(9,9).diagonal()   = cov_T_LI * dt * dt;
    cov_w.block<3,3>(12,12)            = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3,3>(15,15).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
    cov_w.block<3,3>(18,18).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    /* propagation of IMU attitude (global frame)*/
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    acc_imu = R_imu * acc_avr + state_inout.gravity;

    /* propagation of IMU position (global frame)*/
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU (global frame)*/
    vel_imu = vel_imu + acc_imu * dt;

    /* save the poses at each IMU measurements (global frame)*/
    angvel_last = angvel_avr;
    acc_s_last  = acc_imu;
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  state_inout.vel_end = vel_imu + note * acc_imu * dt;
  state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;


  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  if (lidar_type != L515)
  {
    #ifdef DEBUG_PRINT
      cout<<"[ IMU Process ]: vel "<<state_inout.vel_end.transpose()<<" pos "<<state_inout.pos_end.transpose()<<" ba"<<state_inout.bias_a.transpose()<<" bg "<<state_inout.bias_g.transpose()<<endl;
      cout<<"propagated cov: "<<state_inout.cov.diagonal().transpose()<<endl;
    #endif
    /*** un-distort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1; //a single point in k-th frame
      double dt_j = 0.0;
      for(; it_pcl != pcl_out.points.begin(); it_pcl --)
      {
          V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
          dt_j= it_pcl->curvature/double(1000) - pcl_out.points.begin()->curvature/double(1000);
          dt = pcl_out.points.back().curvature/double(1000) - pcl_out.points.begin()->curvature/double(1000);
          Eigen::Quaterniond q_begin(state_begin.rot_end);
          Eigen::Quaterniond q_end(state_inout.rot_end);
          Eigen::Quaterniond q_current = q_begin.slerp( (dt_j / dt), q_end);

          Eigen::Vector3d t_begin = state_begin.pos_end;
          Eigen::Vector3d t_end = state_inout.pos_end;
          Eigen::Vector3d t_current = t_begin + dt_j * (t_end - t_begin) / dt;

          Eigen::Vector3d pjw = q_current * P_j + t_current;

          Eigen::Vector3d P_compensate;
          Eigen::Matrix4d T_wk = Eigen::Matrix4d::Identity();
          T_wk.block<3, 3>(0, 0) = state_inout.rot_end;
          T_wk.block<3, 1>(0, 3) = state_inout.pos_end;

          P_compensate = T_wk.inverse().block<3, 3>(0, 0).matrix() * pjw + T_wk.inverse().block<3, 1>(0, 3);

          it_pcl->x = P_compensate(0);
          it_pcl->y = P_compensate(1);
          it_pcl->z = P_compensate(2);
      }
  }
}


void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en)
  {
    if(meas.imu.empty())  return;
    ROS_ASSERT(meas.lidar != nullptr);

    if (imu_need_init_)
    {
        if(1)
        {
            /// The very first lidar frame
            IMU_init(meas, stat, init_iter_num);
            imu_need_init_ = true;
            last_imu_   = meas.imu.back();
            if (init_iter_num > MAX_INI_COUNT)
            {
                cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
                imu_need_init_ = false;

                cov_acc = cov_acc_scale;
                cov_gyr = cov_gyr_scale;

                ROS_INFO("IMU Initialization Done: Gravity: %.4f %.4f %.4f, Acc norm: %.4f", stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm());
                IMU_mean_acc_norm = mean_acc.norm();
            }
        }
        else{
            cout << endl;
            printf(BOLDMAGENTA "[Refinement] Switch to LIO mode, online refinement begins.\n\n" RESET);
            last_imu_   = meas.imu.back();
            imu_need_init_ = false;
            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
        }
        return;
    }
    propagation_and_undist(meas, stat, *cur_pcl_un_);
  }
  else
  {
     Forward_propagation_without_imu(meas, stat, *cur_pcl_un_);
  }
}
#endif