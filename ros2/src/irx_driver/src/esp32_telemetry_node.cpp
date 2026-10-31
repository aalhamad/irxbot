// Reads "ENV key=value ..." lines from the ESP32-S3 over USB serial and publishes
// /battery (sensor_msgs/BatteryState) and /diagnostics (diagnostic_msgs/DiagnosticArray).
// The port also carries ESP32 log lines, so anything else is ignored.

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

constexpr std::size_t kMaxLineBytes = 512;
constexpr std::size_t kSensorCount = 6;
constexpr std::array<const char *, kSensorCount> kSensorNames = {
    "INA228", "BME280", "SGP30", "SPS30", "VEML7700", "DS18B20"};
constexpr uint8_t kMq2WarmBit = 6;

constexpr float kBatteryCapacityAh = 20.0f;

struct EnvSample
{
    std::map<std::string, double> values;
    std::string raw_values;
    uint8_t ok = 0;
    std::array<unsigned long, kSensorCount> err{};
};

const std::vector<std::string> kNumericKeys = {
    "vbat", "i", "v12", "v5", "v33", "t", "h", "p", "tvoc", "eco2",
    "pm25", "pm10", "lux", "tb", "tr", "tf", "mq2", "st"};

bool parse_double(const std::string &text, double &out)
{
    if (text.empty()) return false;
    char *end = nullptr;
    errno = 0;
    out = std::strtod(text.c_str(), &end);
    return errno == 0 && end == text.c_str() + text.size() && std::isfinite(out);
}

// Accepts a line only when every expected field is present and valid
bool parse_env_line(const std::string &line, EnvSample &sample)
{
    std::istringstream stream(line);
    std::string token;
    if (!(stream >> token) || token != "ENV") return false;

    std::map<std::string, std::string> fields;
    while (stream >> token) {
        const auto eq = token.find('=');
        if (eq == std::string::npos || eq == 0) return false;
        if (!fields.emplace(token.substr(0, eq), token.substr(eq + 1)).second) return false;
    }
    if (fields.size() != kNumericKeys.size() + 2) return false;

    for (const auto &key : kNumericKeys) {
        const auto it = fields.find(key);
        double value = 0.0;
        if (it == fields.end() || !parse_double(it->second, value)) return false;
        sample.values[key] = value;
    }

    const auto ok_it = fields.find("ok");
    if (ok_it == fields.end()) return false;
    unsigned int ok = 0;
    int used = 0;
    if (std::sscanf(ok_it->second.c_str(), "0x%2x%n", &ok, &used) != 1 ||
        static_cast<std::size_t>(used) != ok_it->second.size()) {
        return false;
    }
    sample.ok = static_cast<uint8_t>(ok);

    const auto err_it = fields.find("err");
    if (err_it == fields.end()) return false;
    std::istringstream err_stream(err_it->second);
    std::string part;
    std::size_t index = 0;
    while (std::getline(err_stream, part, ',')) {
        double value = 0.0;
        if (index >= kSensorCount || !parse_double(part, value) || value < 0.0) return false;
        sample.err[index++] = static_cast<unsigned long>(value);
    }
    return index == kSensorCount;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string &key, const std::string &value)
{
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

std::string number(double value, int precision)
{
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", precision, value);
    return text;
}

}  // namespace

class Esp32TelemetryNode : public rclcpp::Node
{
public:
    Esp32TelemetryNode()
    : Node("esp32_telemetry")
    {
        port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM1");
        stale_after_s_ = declare_parameter<double>("stale_after_s", 15.0);

        battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        last_sample_time_ = now();
        read_timer_ = create_wall_timer(100ms, [this]() { read_serial(); });
        stale_timer_ = create_wall_timer(1s, [this]() { check_stale(); });
    }

    ~Esp32TelemetryNode() override
    {
        close_port();
    }

private:
    bool open_port()
    {
        fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            close_port();
            return false;
        }
        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~HUPCL;  // leave the control lines alone when closing
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close_port();
            return false;
        }

        RCLCPP_INFO(get_logger(), "Reading ESP32 telemetry from %s", port_.c_str());
        return true;
    }

    void close_port()
    {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
        line_.clear();
    }

    void read_serial()
    {
        if (fd_ < 0 && !open_port()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open %s: %s",
                                 port_.c_str(), std::strerror(errno));
            return;
        }

        char buffer[256];
        while (true) {
            const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
            if (count > 0) {
                handle_bytes(buffer, static_cast<std::size_t>(count));
                continue;
            }
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
            // A device that has gone away reads as EOF or as an I/O error
            RCLCPP_WARN(get_logger(), "Lost %s, reconnecting", port_.c_str());
            close_port();
            return;
        }
    }

    void handle_bytes(const char *data, std::size_t size)
    {
        for (std::size_t i = 0; i < size; ++i) {
            const char c = data[i];
            if (c == '\n') {
                handle_line(line_);
                line_.clear();
                overflow_ = false;
            } else if (c != '\r' && !overflow_) {
                if (line_.size() >= kMaxLineBytes) {
                    line_.clear();
                    overflow_ = true;
                } else {
                    line_.push_back(c);
                }
            }
        }
    }

    void handle_line(const std::string &line)
    {
        if (line.rfind("ENV ", 0) != 0) return;

        EnvSample sample;
        if (!parse_env_line(line, sample)) {
            ++rejected_lines_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Rejected malformed ENV line (%lu so far)", rejected_lines_);
            return;
        }

        last_sample_time_ = now();
        stale_reported_ = false;
        publish_battery(sample);
        publish_diagnostics(sample);
    }

    void publish_battery(const EnvSample &sample)
    {
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = now();
        msg.header.frame_id = "base_link";
        msg.voltage = static_cast<float>(sample.values.at("vbat"));
        msg.current = -static_cast<float>(sample.values.at("i"));  // negative while discharging
        msg.temperature = static_cast<float>(sample.values.at("tb"));
        msg.charge = std::numeric_limits<float>::quiet_NaN();
        msg.capacity = kBatteryCapacityAh;
        msg.design_capacity = kBatteryCapacityAh;
        msg.percentage = std::numeric_limits<float>::quiet_NaN();  // no charge table for this pack yet
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
        msg.present = true;
        battery_pub_->publish(msg);
    }

    void publish_diagnostics(const EnvSample &sample)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();

        for (std::size_t i = 0; i < kSensorCount; ++i) {
            const bool good = (sample.ok & (1u << i)) != 0;
            DiagnosticStatus status = make_status(
                std::string("esp32/") + kSensorNames[i],
                good ? DiagnosticStatus::OK : DiagnosticStatus::ERROR,
                good ? "OK" : "Not reading");
            status.values.push_back(key_value("failed_reads", std::to_string(sample.err[i])));
            add_sensor_values(i, sample, status);
            array.status.push_back(status);
        }

        array.status.push_back(make_gas_status(sample));

        DiagnosticStatus rails = make_status("esp32/rails", DiagnosticStatus::OK, "OK");
        rails.values.push_back(key_value("12v", number(sample.values.at("v12"), 2)));
        rails.values.push_back(key_value("5v", number(sample.values.at("v5"), 2)));
        rails.values.push_back(key_value("3v3", number(sample.values.at("v33"), 2)));
        array.status.push_back(rails);

        diagnostics_pub_->publish(array);
    }

    DiagnosticStatus make_gas_status(const EnvSample &sample) const
    {
        const bool warm = (sample.ok & (1u << kMq2WarmBit)) != 0;
        const int state = static_cast<int>(sample.values.at("st"));

        DiagnosticStatus status;
        if (!warm) {
            status = make_status("esp32/MQ2", DiagnosticStatus::WARN, "Warming up");
        } else if (state >= 2) {
            status = make_status("esp32/MQ2", DiagnosticStatus::ERROR, "Gas alarm");
        } else if (state == 1) {
            status = make_status("esp32/MQ2", DiagnosticStatus::WARN, "Gas elevated");
        } else {
            status = make_status("esp32/MQ2", DiagnosticStatus::OK, "OK");
        }
        status.values.push_back(key_value("volts", number(sample.values.at("mq2"), 3)));
        return status;
    }

    static void add_sensor_values(std::size_t index, const EnvSample &sample, DiagnosticStatus &status)
    {
        const auto &v = sample.values;
        switch (index) {
            case 0:
                status.values.push_back(key_value("voltage_v", number(v.at("vbat"), 2)));
                status.values.push_back(key_value("current_a", number(v.at("i"), 3)));
                break;
            case 1:
                status.values.push_back(key_value("temperature_c", number(v.at("t"), 1)));
                status.values.push_back(key_value("humidity_pct", number(v.at("h"), 0)));
                status.values.push_back(key_value("pressure_hpa", number(v.at("p"), 1)));
                break;
            case 2:
                status.values.push_back(key_value("tvoc_ppb", number(v.at("tvoc"), 0)));
                status.values.push_back(key_value("eco2_ppm", number(v.at("eco2"), 0)));
                break;
            case 3:
                status.values.push_back(key_value("pm2_5", number(v.at("pm25"), 1)));
                status.values.push_back(key_value("pm10", number(v.at("pm10"), 1)));
                break;
            case 4:
                status.values.push_back(key_value("lux", number(v.at("lux"), 0)));
                break;
            case 5:
                status.values.push_back(key_value("battery_c", number(v.at("tb"), 1)));
                status.values.push_back(key_value("rear_c", number(v.at("tr"), 1)));
                status.values.push_back(key_value("front_c", number(v.at("tf"), 1)));
                break;
            default:
                break;
        }
    }

    static DiagnosticStatus make_status(const std::string &name, uint8_t level, const std::string &message)
    {
        DiagnosticStatus status;
        status.name = name;
        status.hardware_id = "esp32-s3";
        status.level = level;
        status.message = message;
        return status;
    }

    // Without this, telemetry that stops would look the same as healthy silence
    void check_stale()
    {
        if (stale_reported_ || (now() - last_sample_time_).seconds() < stale_after_s_) return;
        stale_reported_ = true;
        RCLCPP_WARN(get_logger(), "No valid ENV line for %.0f s", stale_after_s_);

        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        array.status.push_back(make_status("esp32/link", DiagnosticStatus::STALE, "No telemetry"));
        diagnostics_pub_->publish(array);
    }

    std::string port_;
    double stale_after_s_ = 15.0;
    int fd_ = -1;
    std::string line_;
    bool overflow_ = false;
    unsigned long rejected_lines_ = 0;
    bool stale_reported_ = false;
    rclcpp::Time last_sample_time_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr stale_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Esp32TelemetryNode>());
    rclcpp::shutdown();
    return 0;
}
