#include <rclcpp/rclcpp.hpp>
#include <cpu_monitor/msg/x86_cpu_temperature.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <set>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <map>
#include <cmath>

namespace fs = std::filesystem;

// CPU 统计数据结构
struct CpuStats {
    uint64_t user;
    uint64_t nice;
    uint64_t system;
    uint64_t idle;
    uint64_t iowait;
    uint64_t irq;
    uint64_t softirq;
};

class CpuMonitorNode : public rclcpp::Node {
public:
    CpuMonitorNode() : Node("cpu_monitor"),
                       cpu_usage_threshold_(5.0),  // 5% 使用率阈值
                       last_cpu_stats_(),
                       prev_cpu_stats_(),
                       allowed_cpus_() {
        // 创建发布者
        publisher_ = this->create_publisher<cpu_monitor::msg::X86CpuTemperature>("x86_temperature", 10);
        
        // 初始化 CPU 统计信息（用于计算使用率）
        read_proc_stat();
        prev_cpu_stats_ = last_cpu_stats_;
        
        // 读取 CPU 亲和性列表
        allowed_cpus_ = get_allowed_cpus();
        RCLCPP_INFO(this->get_logger(), "CPU affinity: %zu cores allowed", allowed_cpus_.size());
        
        // 创建定时器，每1秒调用一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CpuMonitorNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), 
                   "CPU Monitor Node initialized (usage threshold: %.1f%%)", 
                   cpu_usage_threshold_);
    }

private:
    // 从 /proc/stat 读取 CPU 统计信息
    void read_proc_stat() {
        std::ifstream stat_file("/proc/stat");
        std::string line;
        
        last_cpu_stats_.clear();
        
        try {
            while (std::getline(stat_file, line)) {
                if (line.find("cpu") != 0) break;
                
                std::string cpu_name;
                uint64_t user, nice, system, idle, iowait, irq, softirq;
                
                std::istringstream iss(line);
                iss >> cpu_name >> user >> nice >> system >> idle >> iowait >> irq >> softirq;
                
                // 解析 CPU 编号（cpu, cpu0, cpu1, ...）
                if (cpu_name == "cpu") {
                    continue;  // 跳过总体 CPU 行
                }
                
                CpuStats stats;
                stats.user = user;
                stats.nice = nice;
                stats.system = system;
                stats.idle = idle;
                stats.iowait = iowait;
                stats.irq = irq;
                stats.softirq = softirq;
                
                last_cpu_stats_[cpu_name] = stats;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading /proc/stat: %s", e.what());
        }
    }
    
    // 计算单个 CPU 核心的使用率
    double calculate_cpu_usage(const std::string& cpu_name) {
        if (last_cpu_stats_.find(cpu_name) == last_cpu_stats_.end() ||
            prev_cpu_stats_.find(cpu_name) == prev_cpu_stats_.end()) {
            return 0.0;
        }
        
        const CpuStats& curr = last_cpu_stats_[cpu_name];
        const CpuStats& prev = prev_cpu_stats_[cpu_name];
        
        uint64_t curr_idle = curr.idle + curr.iowait;
        uint64_t prev_idle = prev.idle + prev.iowait;
        
        uint64_t curr_non_idle = curr.user + curr.nice + curr.system + curr.irq + curr.softirq;
        uint64_t prev_non_idle = prev.user + prev.nice + prev.system + prev.irq + prev.softirq;
        
        uint64_t total_diff = (curr_idle + curr_non_idle) - (prev_idle + prev_non_idle);
        uint64_t idle_diff = curr_idle - prev_idle;
        
        if (total_diff == 0) return 0.0;
        
        double usage = 100.0 * static_cast<double>(total_diff - idle_diff) / static_cast<double>(total_diff);
        return std::max(0.0, std::min(100.0, usage));
    }
    
    // 解析 CPU 亲和性列表（从 /proc/self/status 的 Cpus_allowed_list）
    std::set<int> get_allowed_cpus() {
        std::set<int> allowed_cpus;
        std::ifstream status_file("/proc/self/status");
        std::string line;
        
        try {
            while (std::getline(status_file, line)) {
                if (line.find("Cpus_allowed_list") != std::string::npos) {
                    size_t pos = line.find(":");
                    if (pos != std::string::npos) {
                        std::string cpus_str = line.substr(pos + 1);
                        // 去除前后空格
                        cpus_str.erase(0, cpus_str.find_first_not_of(" \t"));
                        cpus_str.erase(cpus_str.find_last_not_of(" \t") + 1);
                        
                        // 解析 CPU 列表（格式: "0-3,5,7-8" 或 "0-15"）
                        parse_cpu_list(cpus_str, allowed_cpus);
                    }
                    break;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading CPU affinity: %s", e.what());
        }
        
        // 如果无法读取亲和性列表，则允许所有 CPU
        if (allowed_cpus.empty()) {
            for (const auto& entry : last_cpu_stats_) {
                try {
                    int cpu_num = std::stoi(entry.first.substr(3));
                    allowed_cpus.insert(cpu_num);
                } catch (...) {
                    // 忽略解析错误
                }
            }
        }
        
        return allowed_cpus;
    }
    
    // 辅助函数：解析 CPU 列表字符串（如 "0-3,5,7-8"）
    void parse_cpu_list(const std::string& cpu_str, std::set<int>& cpus) {
        try {
            std::string s = cpu_str;
            size_t pos = 0;
            
            while (pos < s.length()) {
                size_t comma_pos = s.find(',', pos);
                if (comma_pos == std::string::npos) {
                    comma_pos = s.length();
                }
                
                std::string range = s.substr(pos, comma_pos - pos);
                size_t dash_pos = range.find('-');
                
                if (dash_pos != std::string::npos) {
                    // 处理范围 "0-3"
                    int start = std::stoi(range.substr(0, dash_pos));
                    int end = std::stoi(range.substr(dash_pos + 1));
                    for (int i = start; i <= end; ++i) {
                        cpus.insert(i);
                    }
                } else {
                    // 处理单个 CPU "5"
                    cpus.insert(std::stoi(range));
                }
                
                pos = comma_pos + 1;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing CPU list: %s", e.what());
        }
    }
    
    // 检查 CPU 是否在允许列表中
    bool is_cpu_allowed(int cpu_num) const {
        return allowed_cpus_.find(cpu_num) != allowed_cpus_.end();
    }
    
    // 从 /sys/class/hwmon 读取 coretemp 驱动的 CPU 核心温度
    std::map<std::string, float> get_core_temperatures() {
        std::map<std::string, float> core_temps;
        std::string hwmon_base = "/sys/class/hwmon";
        
        try {
            if (fs::exists(hwmon_base)) {
                // 遍历所有 hwmon 目录
                for (const auto& hwmon_entry : fs::directory_iterator(hwmon_base)) {
                    if (!hwmon_entry.is_directory()) continue;
                    
                    std::string hwmon_dir = hwmon_entry.path().string();
                    std::string name_file = hwmon_dir + "/name";
                    
                    // 检查驱动名称是否为 coretemp
                    if (fs::exists(name_file)) {
                        std::ifstream name_stream(name_file);
                        std::string driver_name;
                        
                        if (std::getline(name_stream, driver_name)) {
                            // 去除末尾的换行符
                            driver_name.erase(driver_name.find_last_not_of(" \n\r\t") + 1);
                            
                            if (driver_name == "coretemp") {
                                // 找该目录下所有的温度标签文件
                                for (const auto& temp_entry : fs::directory_iterator(hwmon_dir)) {
                                    std::string entry_name = temp_entry.path().filename().string();
                                    
                                    // 查找 temp*_label 文件
                                    if (entry_name.find("temp") == 0 && 
                                        entry_name.find("_label") != std::string::npos) {
                                        
                                        std::string label_file = temp_entry.path().string();
                                        
                                        // 读取标签名
                                        std::ifstream label_stream(label_file);
                                        std::string label;
                                        
                                        if (std::getline(label_stream, label)) {
                                            // 去除末尾的换行符
                                            label.erase(label.find_last_not_of(" \n\r\t") + 1);
                                            
                                            // 构造对应的 _input 文件名
                                            std::string input_file = label_file;
                                            size_t label_pos = input_file.find("_label");
                                            if (label_pos != std::string::npos) {
                                                input_file.replace(label_pos, 6, "_input");
                                                
                                                // 读取温度值
                                                if (fs::exists(input_file)) {
                                                    std::ifstream input_stream(input_file);
                                                    int64_t raw_temp = 0;
                                                    
                                                    if (input_stream >> raw_temp) {
                                                        // 温度以毫摄氏度为单位，转换为摄氏度
                                                        float temp_c = raw_temp / 1000.0f;
                                                        core_temps[label] = temp_c;
                                                        
                                                        RCLCPP_DEBUG(this->get_logger(),
                                                                   "CPU %s : %.1f °C",
                                                                   label.c_str(), temp_c);
                                                    }
                                                }
                                            }
                                        }
                                        label_stream.close();
                                    }
                                }
                            }
                        }
                        name_stream.close();
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading hwmon coretemp data: %s", e.what());
        }
        
        return core_temps;
    }
    
    // 获取运行中的CPU核心的平均温度（排除低使用率和离线的核心）
    float get_average_cpu_temperature() {
        std::vector<float> valid_temperatures;
        
        // 首先尝试从 coretemp 驱动读取
        std::map<std::string, float> core_temps = get_core_temperatures();
        
        if (!core_temps.empty()) {
            // 从核心标签中提取 CPU 编号，进行过滤
            for (const auto& [label, temp] : core_temps) {
                // 解析标签，如 "Core 0", "Core 1", 等
                bool should_include = true;
                
                // 如果标签包含 "Core"，则为核心级温度，进行亲和性和使用率检查
                if (label.find("Core") != std::string::npos) {
                    try {
                        // 从标签中提取核心编号（如 "Core 0" -> 0）
                        size_t space_pos = label.rfind(' ');
                        if (space_pos != std::string::npos) {
                            int core_num = std::stoi(label.substr(space_pos + 1));
                            std::string cpu_name = "cpu" + std::to_string(core_num);
                            
                            // 检查是否在亲和性列表中
                            if (!is_cpu_allowed(core_num)) {
                                should_include = false;
                                RCLCPP_DEBUG(this->get_logger(),
                                           "Skipping CPU not in affinity list: %s", cpu_name.c_str());
                            } else {
                                // 检查 CPU 使用率
                                double usage = calculate_cpu_usage(cpu_name);
                                if (usage < cpu_usage_threshold_) {
                                    should_include = false;
                                    RCLCPP_DEBUG(this->get_logger(),
                                               "Skipping low-usage CPU %s (%.1f%%)",
                                               cpu_name.c_str(), usage);
                                }
                            }
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_DEBUG(this->get_logger(), "Error parsing core label '%s': %s",
                                    label.c_str(), e.what());
                    }
                } else if (label.find("Package") != std::string::npos) {
                    // Package 温度通常是平台级的，不需要过滤
                    should_include = true;
                }
                
                if (should_include) {
                    valid_temperatures.push_back(temp);
                }
            }
        }
        
        // 如果没有找到有效温度，尝试备选方案
        if (valid_temperatures.empty()) {
            valid_temperatures = get_cpu_temperature_from_cpuinfo();
        }
        
        // 计算平均温度
        if (!valid_temperatures.empty()) {
            float sum = std::accumulate(valid_temperatures.begin(), valid_temperatures.end(), 0.0f);
            float avg_temp = sum / static_cast<float>(valid_temperatures.size());
            
            RCLCPP_DEBUG(this->get_logger(),
                       "Calculated average temperature from %zu cores: %.1f°C",
                       valid_temperatures.size(), avg_temp);
            
            return avg_temp;
        }
        
        return 0;
    }
    
    // 备选方案：从 /proc/cpuinfo 读取 CPU 频率信息
    std::vector<float> get_cpu_temperature_from_cpuinfo() {
        std::vector<float> cpu_temps;
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        int cpu_count = 0;
        
        try {
            while (std::getline(cpuinfo, line)) {
                if (line.find("processor") != std::string::npos) {
                    cpu_count++;
                }
                
                if (line.find("cpu MHz") != std::string::npos) {
                    size_t pos = line.find(":");
                    if (pos != std::string::npos) {
                        try {
                            double mhz = std::stod(line.substr(pos + 1));
                            
                            // 检查该 CPU 核心是否应该被计入
                            int cpu_num = cpu_count - 1;
                            std::string cpu_name = "cpu" + std::to_string(cpu_num);
                            
                            bool should_include = true;
                            
                            // 检查亲和性
                            if (!is_cpu_allowed(cpu_num)) {
                                should_include = false;
                            } else {
                                // 检查使用率
                                double usage = calculate_cpu_usage(cpu_name);
                                if (usage < cpu_usage_threshold_) {
                                    should_include = false;
                                }
                            }
                            
                            if (should_include) {
                                // 将频率转换为温度估计
                                int32_t temp = static_cast<int32_t>(30 + (mhz - 800) / 100);
                                cpu_temps.push_back(temp);
                            }
                        } catch (...) {
                            // 忽略转换错误
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading cpuinfo: %s", e.what());
        }
        
        return cpu_temps;
    }
    
    void timer_callback() {
        // 更新 CPU 统计信息
        prev_cpu_stats_ = last_cpu_stats_;
        read_proc_stat();
        
        // 获取平均温度
        float avg_temp = get_average_cpu_temperature();
        
        // 创建并发布消息
        auto message = cpu_monitor::msg::X86CpuTemperature();
        message.x86_temperature = avg_temp;
        
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published CPU temperature: %.1f°C", avg_temp);
    }
    
    rclcpp::Publisher<cpu_monitor::msg::X86CpuTemperature>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // CPU 使用率阈值（百分比）
    double cpu_usage_threshold_;
    
    // 保存 CPU 统计信息
    std::map<std::string, CpuStats> last_cpu_stats_;   // 当前统计
    std::map<std::string, CpuStats> prev_cpu_stats_;   // 前一次统计
    
    // CPU 亲和性列表（允许使用的 CPU 核心编号）
    std::set<int> allowed_cpus_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CpuMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
