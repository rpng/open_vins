#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

struct filter_state {
  float xs[3] = {0,0,0};
  float ys[3] = {0,0,0};
};

class Filter {
 	public:
 		Filter(ros::NodeHandle& nodeHandle,
         ros::NodeHandle& privateNodeHandle);

		void setup();
	private:
		float apply(filter_state &state, float sample);
		float reset(filter_state &state, float sample);
		void imuCallback(const sensor_msgs::ImuConstPtr& msg_in);
		void shift_stamp(ros::Time& stamp, double delay);

		ros::NodeHandle nh;
		ros::NodeHandle pnh;

		ros::Subscriber imu_sub;
		ros::Publisher imu_pub;
		
		bool initalised;
		int corner_freq;
		double filter_delay;

		double a_[2];
		double gain_;

		// accelerometer filter queues
		filter_state ax;
		filter_state ay;
		filter_state az;
		// gyro filter queues
		filter_state wx;
		filter_state wy;
		filter_state wz;
};
