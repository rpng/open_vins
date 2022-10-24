
#include "../include/butterworth/butterworth.h"

// constructor
Filter::Filter(ros::NodeHandle& nodeHandle,
       ros::NodeHandle& privateNodeHandle) :
  nh(nodeHandle),
  pnh(privateNodeHandle)
{
  initalised = false;
}

float Filter::apply(filter_state &state, float sample) {
  if(!initalised){
    initalised = true;
    return reset(state, sample);
  }
  state.xs[0] = state.xs[1];
  state.xs[1] = state.xs[2];
  state.xs[2] = sample / gain_;
  state.ys[0] = state.ys[1];
  state.ys[1] = state.ys[2];
  state.ys[2] = (state.xs[0] + state.xs[2]) + 2 * state.xs[1] 
                     + (a_[0] * state.ys[0]) + (a_[1] * state.ys[1]);
  
  float prev_output = state.ys[1];
  float output = state.ys[2];
  float dt = std::fabs(output - prev_output);

  if (dt>5.0) { // check if filter state has converged yet
    return sample;
  }

  return output;
}

float Filter::reset(filter_state &state, float sample) {
  for (int i; i<3; i++) {
    state.xs[i] = sample;
    state.ys[i] = sample;
  }
  return sample;
}

void Filter::imuCallback(const sensor_msgs::ImuConstPtr& msg_in) {
  sensor_msgs::Imu msg_out = *msg_in;

  msg_out.linear_acceleration.x = apply(ax, msg_in->linear_acceleration.x);
  msg_out.linear_acceleration.y = apply(ay, msg_in->linear_acceleration.y);
  msg_out.linear_acceleration.z = apply(az, msg_in->linear_acceleration.z);

  msg_out.angular_velocity.x = apply(wx, msg_in->angular_velocity.x);
  msg_out.angular_velocity.y = apply(wy, msg_in->angular_velocity.y);
  msg_out.angular_velocity.z = apply(wz, msg_in->angular_velocity.z);

  ros::Time shifted_stamp = msg_out.header.stamp;
  shift_stamp(shifted_stamp, filter_delay);
  msg_out.header.stamp = shifted_stamp;
  imu_pub.publish(msg_out);
}

void Filter::shift_stamp(ros::Time& stamp, double delay) {
  stamp = ros::Time(stamp.toSec() - delay);
}

void Filter::setup() {
  imu_sub = nh.subscribe("/race4/zedm/zed_node/imu/data_raw", 100, &Filter::imuCallback, this);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/race4/zedm/zed_node/imu/data_raw_filtered", 100);

  pnh.param("corner_freq", corner_freq, 50);
  pnh.param("filter_delay", filter_delay, 0.0);

  // Coefficients from http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
  // filtertype = Butterworth
  // passtype = Lowpass
  // ripple =
  // order = 2
  // samplerate = 200
  /*
  switch (corner_freq) { // corner frq in Hz
    case 10:
      gain_ = 4.979245121e+01;
      a_[0] = -0.6413515381;
      a_[1] = 1.5610180758;
      break;
    case 20:
      gain_ = 1.482463775e+01;
      a_[0] = -0.4128015981;
      a_[1] = 1.1429805025;
      break;
    case 30:
      gain_ = 7.627390391e+00;
      a_[0] = -0.2722149379;
      a_[1] = 0.7477891783;
      break;
    case 40:
      gain_ = 4.840925170e+00;
      a_[0] = -0.1958157127;
      a_[1] = 0.3695273774;
      break;
    case 50:
      gain_ = 3.414213562e+00;
      a_[0] = -0.1715728753;
      a_[1] = 0.0000000000;
      break;
    default:
      ROS_WARN("Cutoff frequency %dHz not supported!", corner_freq);
      corner_freq = 20;
      gain_ = 1.482463775e+01;
      a_[0] = -0.4128015981;
      a_[1] = 1.1429805025;
  }
  */

  // freq = 400
  switch (corner_freq) { // corner frq in Hz
    case 10:
      gain_ =1.804169259e+002; 
      a_[0] = -0.8008026467;
      a_[1] =  1.7786317778;
      break;
    case 20:
      gain_ = 4.979245121e+001;
      a_[0] = -0.6413515381;
      a_[1] = 1.5610180758;
      break;
    case 30:
      gain_ = 2.424034560e+001;
      a_[0] = -0.5139818942;
      a_[1] =  1.3489677453;
      break;
    case 40:
      gain_ = 1.482463775e+001;
      a_[0] = -0.4128015981 ;
      a_[1] =  1.1429805025;
      break;
    case 50:
      gain_ =1.024264069e+001;
      a_[0] = -0.3333333333;
      a_[1] = 0.9428090416;
      break;
    default:
      ROS_WARN("Cutoff frequency %dHz not supported!", corner_freq);
      corner_freq = 20;
      gain_ = 1.482463775e+01;
      a_[0] = -0.4128015981;
      a_[1] = 1.1429805025;
  }


  ROS_INFO("[IMU filter] Using butterworth filter with corner frequency %dHz", corner_freq);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "butterworth_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Filter *butterworth_node= new Filter(nh, pnh);
  butterworth_node->setup();

  ros::spin();

  return 0;
}
