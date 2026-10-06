//----------------------------------------------------------------------------------------------------------------------------------
// LightWare SF45B ROS 2 driver (PointCloud2 + LaserScan).
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"

#include <cmath>
#include <string>
#include <vector>
#include <cstring>
#include <memory>
#include <algorithm>
#include <limits>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct lwSf45Params {
	int32_t updateRate;
	int32_t cycleDelay;
	float lowAngleLimit;
	float highAngleLimit;
};

inline float deg2rad(float deg) {
	return deg * static_cast<float>(M_PI) / 180.0f;
}

void validateParams(lwSf45Params* Params) {
	if (Params->updateRate < 1) Params->updateRate = 1;
	else if (Params->updateRate > 12) Params->updateRate = 12;

	if (Params->cycleDelay < 5) Params->cycleDelay = 5;
	else if (Params->cycleDelay > 2000) Params->cycleDelay = 2000;

	if (Params->lowAngleLimit < -160.0f) Params->lowAngleLimit = -160.0f;
	else if (Params->lowAngleLimit > -10.0f) Params->lowAngleLimit = -10.0f;

	if (Params->highAngleLimit < 10.0f) Params->highAngleLimit = 10.0f;
	else if (Params->highAngleLimit > 160.0f) Params->highAngleLimit = 160.0f;
}

int driverStart(lwSerialPort** Serial, const char* PortName, int32_t BaudRate) {
	platformInit();

	lwSerialPort* serial = platformCreateSerialPort();
	*Serial = serial;
	if (!serial->connect(PortName, BaudRate)) {
		RCLCPP_ERROR(rclcpp::get_logger("sf45b"), "Could not establish serial connection on %s at %d baud", PortName, BaudRate);
		return 1;
	}

	// Disable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(serial, 30, 0)) { return 1; }

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName)) { return 1; }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) { return 1; }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;	
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) { return 1; }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber)) { return 1; }

	RCLCPP_INFO(rclcpp::get_logger("sf45b"), "Model: %.16s", modelName);
	RCLCPP_INFO(rclcpp::get_logger("sf45b"), "Hardware: %u", hardwareVersion);
	RCLCPP_INFO(rclcpp::get_logger("sf45b"), "Firmware: %.16s (%u)", firmwareVersionStr, firmwareVersion);
	RCLCPP_INFO(rclcpp::get_logger("sf45b"), "Serial: %.16s", serialNumber);

	return 0;
}

int driverScanStart(lwSerialPort* Serial, lwSf45Params* Params) {
	// Configure distance output for first return and angle. (Command 27: Distance output)
	if (!lwnxCmdWriteUInt32(Serial, 27, 0x101)) { return 1; }

	// (Command 66: Update rate)
	if (!lwnxCmdWriteUInt8(Serial, 66, Params->updateRate)) { return 1; }

	// (Command 85: Scan speed)
	if (!lwnxCmdWriteUInt16(Serial, 85, Params->cycleDelay)) { return 1; }

	// (Command 98: Scan low angle)
	if (!lwnxCmdWriteFloat(Serial, 98, Params->lowAngleLimit)) { return 1; }

	// (Command 99: Scan high angle)
	if (!lwnxCmdWriteFloat(Serial, 99, Params->highAngleLimit)) { return 1; }

	// (Command 96: Scan enable) Ensure scanner motor is actively scanning
	if (!lwnxCmdWriteUInt8(Serial, 96, 1)) { return 1; }

	// Enable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(Serial, 30, 5)) { return 1; }

	return 0;
}

void driverScanStop(lwSerialPort* Serial) {
	if (Serial) {
		// 1. Disable streaming of point data (Command 30: Stream = 0)
		uint32_t streamDisable = 0;
		lwnxSendPacketBytes(Serial, 30, 1, reinterpret_cast<uint8_t*>(&streamDisable), sizeof(streamDisable));
		lwnxCmdWriteUInt32(Serial, 30, 0);

		// 2. Disable scanning motor (Command 96: Scan enable = 0)
		uint8_t scanDisable = 0;
		lwnxSendPacketBytes(Serial, 96, 1, &scanDisable, sizeof(scanDisable));
		lwnxCmdWriteUInt8(Serial, 96, 0);
	}
}

static lwSerialPort* g_serial = nullptr;

void sigHandler(int sig) {
	(void)sig;
	if (g_serial) {
		driverScanStop(g_serial);
	}
	rclcpp::shutdown();
}

struct lwDistanceResult {
	float x;
	float y;
	float z;
};

struct rawDistanceResult {
	float distance;
	float angle_rad;
};

int driverScan(lwSerialPort* Serial, lwDistanceResult* DistanceResult, rawDistanceResult* RawDistanceResult) {
	// The incoming point data packet is Command 44: Distance data in cm.
	lwResponsePacket response;

	if (lwnxRecvPacket(Serial, 44, &response, 1000)) {
		int16_t distanceCm = (response.data[5] << 8) | response.data[4];
		int16_t angleHundredths = (response.data[7] << 8) | response.data[6];

		float distance = distanceCm / 100.0f;
		float angle = angleHundredths / 100.0f; // degrees (0 = front, <0 left, >0 right)

		// ROS REP 103 Standard Body / Sensor Frame:
		// +X: Forward, +Y: Left, +Z: Up.
		// Hardware positive angle is CW (Right), ROS positive angle is CCW (Left) -> angle_ros = -deg2rad(angle)
		float angle_ros = -deg2rad(angle);

		DistanceResult->x = distance * cosf(angle_ros);
		DistanceResult->y = distance * sinf(angle_ros);
		DistanceResult->z = 0.0f;

		RawDistanceResult->distance = distance;
		RawDistanceResult->angle_rad = angle_ros;

		return 1;
	}

	return 0;
}

sensor_msgs::msg::LaserScan buildLaserScanMsg(
	int pointCount,
	std::vector<rawDistanceResult>& rawDistances,
	const lwSf45Params& params,
	double scanDuration)
{
	// Sort by angle so laser scan rays are in monotonic angular order (from right/negative to left/positive)
	std::sort(rawDistances.begin(), rawDistances.begin() + pointCount,
		[](const rawDistanceResult& a, const rawDistanceResult& b) {
			return a.angle_rad < b.angle_rad;
		});

	sensor_msgs::msg::LaserScan scanMsg;
	// Hardware highAngleLimit (+deg) is right side in ROS (-rad)
	// Hardware lowAngleLimit (-deg) is left side in ROS (+rad)
	float configuredMinAngle = -deg2rad(params.highAngleLimit);
	float configuredMaxAngle = -deg2rad(params.lowAngleLimit);

	scanMsg.angle_min = std::max(configuredMinAngle, rawDistances[0].angle_rad);
	scanMsg.angle_max = std::min(configuredMaxAngle, rawDistances[pointCount - 1].angle_rad);

	if (pointCount > 1 && scanMsg.angle_max > scanMsg.angle_min) {
		scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / static_cast<float>(pointCount - 1);
	} else {
		scanMsg.angle_increment = 0.0f;
	}

	scanMsg.time_increment = (pointCount > 0) ? static_cast<float>(scanDuration / pointCount) : 0.0f;
	scanMsg.scan_time = static_cast<float>(scanDuration);
	scanMsg.range_min = 0.2f;
	scanMsg.range_max = 50.0f;

	scanMsg.ranges.assign(pointCount, std::numeric_limits<float>::infinity());

	for (int i = 0; i < pointCount; ++i) {
		const auto& cur = rawDistances[i];
		if (cur.distance >= scanMsg.range_min && cur.distance <= scanMsg.range_max) {
			scanMsg.ranges[i] = cur.distance;
		}
	}

	return scanMsg;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("sf45b", "lightwarelidar");

	auto pointCloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
	auto laserScanPub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

	lwSerialPort* serial = nullptr;

	int32_t baudRate = node->declare_parameter<int>("baudrate", 115200);
	std::string portName = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
	std::string frameId = node->declare_parameter<std::string>("frame_id", "laser");
	bool publishLaserScan = node->declare_parameter<bool>("publish_laser_scan", true);

	lwSf45Params params;
	params.updateRate = node->declare_parameter<int>("updateRate", 12);
	params.cycleDelay = node->declare_parameter<int>("cycleDelay", 5);
	params.lowAngleLimit = static_cast<float>(node->declare_parameter<double>("lowAngleLimit", -45.0));
	params.highAngleLimit = static_cast<float>(node->declare_parameter<double>("highAngleLimit", 45.0));
	validateParams(&params);

	int maxPointsPerMsg = node->declare_parameter<int>("maxPoints", 100);
	if (maxPointsPerMsg < 1) maxPointsPerMsg = 1;

	RCLCPP_INFO(node->get_logger(), "Starting SF45B ROS 2 node on port %s (%d baud)", portName.c_str(), baudRate);
	RCLCPP_INFO(node->get_logger(), "Publishing: /pointcloud [PointCloud2]%s",
		publishLaserScan ? " and /scan [LaserScan]" : "");

	if (driverStart(&serial, portName.c_str(), baudRate) != 0) {
		RCLCPP_ERROR(node->get_logger(), "Failed to start SF45B driver on %s", portName.c_str());
		return 1;
	}

	if (driverScanStart(serial, &params) != 0) {
		RCLCPP_ERROR(node->get_logger(), "Failed to start SF45B scan stream");
		return 1;
	}

	g_serial = serial;
	std::signal(SIGINT, sigHandler);
	std::signal(SIGTERM, sigHandler);

	sensor_msgs::msg::PointCloud2 pointCloudMsg;
	pointCloudMsg.header.frame_id = frameId;
	pointCloudMsg.height = 1;
	pointCloudMsg.width = maxPointsPerMsg;

	pointCloudMsg.fields.resize(3);
	pointCloudMsg.fields[0].name = "x";
	pointCloudMsg.fields[0].offset = 0;
	pointCloudMsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pointCloudMsg.fields[0].count = 1;

	pointCloudMsg.fields[1].name = "y";
	pointCloudMsg.fields[1].offset = 4;
	pointCloudMsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pointCloudMsg.fields[1].count = 1;

	pointCloudMsg.fields[2].name = "z";
	pointCloudMsg.fields[2].offset = 8;
	pointCloudMsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pointCloudMsg.fields[2].count = 1;

	pointCloudMsg.is_bigendian = false;
	pointCloudMsg.point_step = 12;
	pointCloudMsg.row_step = 12 * maxPointsPerMsg;
	pointCloudMsg.is_dense = true;

	pointCloudMsg.data = std::vector<uint8_t>(maxPointsPerMsg * 12);

	int currentPoint = 0;
	std::vector<lwDistanceResult> distanceResults(maxPointsPerMsg);
	std::vector<rawDistanceResult> rawDistanceResults(maxPointsPerMsg);

	auto lastPublishTime = node->now();

	RCLCPP_INFO(node->get_logger(), "SF45B driver streaming actively.");

	while (rclcpp::ok()) {
		lwDistanceResult distanceResult;
		rawDistanceResult rawResult;
		int status = driverScan(serial, &distanceResult, &rawResult);

		if (status == 0) {
			rclcpp::spin_some(node);
			continue;
		}

		distanceResults[currentPoint] = distanceResult;
		rawDistanceResults[currentPoint] = rawResult;
		++currentPoint;

		if (currentPoint == maxPointsPerMsg) {
			auto now = node->now();
			double scanDuration = (now - lastPublishTime).seconds();
			lastPublishTime = now;

			// 1. Publish PointCloud2
			std::memcpy(&pointCloudMsg.data[0], &distanceResults[0], maxPointsPerMsg * 12);
			pointCloudMsg.header.stamp = now;
			pointCloudPub->publish(pointCloudMsg);

			// 2. Publish LaserScan (if enabled)
			if (publishLaserScan) {
				auto laserScanMsg = buildLaserScanMsg(maxPointsPerMsg, rawDistanceResults, params, scanDuration);
				laserScanMsg.header.stamp = now;
				laserScanMsg.header.frame_id = frameId;
				laserScanPub->publish(laserScanMsg);
			}

			currentPoint = 0;
			rclcpp::spin_some(node);
		}
	}

	RCLCPP_INFO(node->get_logger(), "Shutting down SF45B: stopping motor and data stream...");
	if (serial) {
		driverScanStop(serial);
		delete serial;
		serial = nullptr;
		g_serial = nullptr;
	}

	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
	return 0;
}
