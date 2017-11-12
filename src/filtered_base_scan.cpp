#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define maxAllowedScans 30

void noisyScanCallback(const sensor_msgs::LaserScan& msg);
void calculateResults();

int currentSample = 0;
std::vector<std::pair<float, float>> samples[maxAllowedScans];
std::pair<float, float> results[maxAllowedScans];

int main(int argc, char **argv) {
	ros::init(argc, argv, "filtered_base_scan");
	ros::NodeHandle nh;
	ros::Subscriber noisy_scan_sub = nh.subscribe("/noisy_base_scan", 100, noisyScanCallback);

	ros::spin();
}

void noisyScanCallback(const sensor_msgs::LaserScan& msg) {
		float start_angle = msg.angle_min;
		float increment = msg.angle_increment;
		for(int i = 0; i < maxAllowedScans; i++) {
			float x = msg.ranges[i] * cos(start_angle);
			float y = msg.ranges[i] * sin(start_angle);
			samples[i].emplace_back(std::make_pair(x, y));
			start_angle += increment;
		}

		currentSample++;
		if(currentSample >= 10) {
			calculateResults();
			exit(-1);
		}
}

void calculateResults() {
	for(int i = 0; i < maxAllowedScans; i++) {
		float xResult = 0;
		float yResult = 0;
		for(auto &sample : samples[i]) {
			xResult += sample.first;
			yResult += sample.second;
		}
		xResult = xResult / samples[i].size();
		yResult = yResult / samples[i].size();
		results[i] = std::make_pair(xResult, yResult);
	}

	for(int i = 0; i < maxAllowedScans; i++) {
		std::cout << "Something at x:" << results[i].first << " y:" << results[i].second << std::endl;
	}
}