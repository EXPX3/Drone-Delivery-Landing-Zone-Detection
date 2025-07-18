#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <thread>
#include <chrono>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <functional>
#include <algorithm>
#include <sys/resource.h>
#include "architecture.h"

// Typedefs
using PointPcl = pcl::PointXYZI;
using PointCloudPcl = pcl::PointCloud<PointPcl>::Ptr;

// Struct to hold resource usage metrics
struct ResourceMetrics {
    double user_time;
    double system_time;
    long max_rss;
};

// Struct to hold parameters for a landing zone
struct LandingZoneParams {
    double radius;
    double slope_degrees;
    double point_density;
    double relief;
    double roughness;
};

// Function to load a point cloud from a .pcd file, handling both PointXYZI and PointXYZ
PointCloudPcl loadPointCloudFromPCD(const std::string& file_path) {
    PointCloudPcl cloud(new pcl::PointCloud<PointPcl>);

    // First, try loading as PointXYZI
    if (pcl::io::loadPCDFile<PointPcl>(file_path, *cloud) == 0) {
        std::cout << "Loaded PointXYZI point cloud from " << file_path << " with " << cloud->points.size() << " points" << std::endl;
        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    // If PointXYZI fails, try loading as PointXYZ and convert to PointXYZI
    std::cerr << "Failed to load as PointXYZI, attempting to load as PointXYZ" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *temp_cloud) == -1) {
        std::cerr << "Error: Could not load PCD file at " << file_path << " as PointXYZ either" << std::endl;
        return nullptr;
    }

    // Convert PointXYZ to PointXYZI
    cloud->points.resize(temp_cloud->points.size());
    cloud->width = temp_cloud->points.size();
    cloud->height = 1;
    for (size_t i = 0; i < temp_cloud->points.size(); ++i) {
        cloud->points[i].x = temp_cloud->points[i].x;
        cloud->points[i].y = temp_cloud->points[i].y;
        cloud->points[i].z = temp_cloud->points[i].z;
        cloud->points[i].intensity = 0.0f; // Default intensity
    }
    std::cout << "Loaded and converted PointXYZ point cloud from " << file_path << " with " << cloud->points.size() << " points" << std::endl;
    return cloud;
}

// PointCloudGenerator class
class PointCloudGenerator {
private:
    std::mt19937 rng;
    double total_size;
    double max_spike_spacing;
    double spike_base_radius;
    double min_spike_height;
    double max_spike_height;

public:
    PointCloudGenerator(unsigned int seed, double terrain_size = 20.0)
        : rng(seed), total_size(terrain_size), max_spike_spacing(0.2),
          spike_base_radius(8), min_spike_height(6.0), max_spike_height(7.0) {}

    PointCloudPcl generatePointCloud(const LandingZoneParams& params) {
        PointCloudPcl cloud(new pcl::PointCloud<PointPcl>);
        double inclined_half = params.radius;
        double total_half = total_size / 2.0;
        double point_spacing = std::sqrt(1.0 / params.point_density);
        double max_unevenness = 4.5;
        double terrain_noise = 0.5;
        const double max_terrain_height = 5.0;

        // Create grid
        std::vector<double> grid;
        for (double pos = -total_half; pos <= total_half + point_spacing; pos += point_spacing) {
            grid.push_back(pos);
        }

        // Normal for inclined plane
        double theta = params.slope_degrees * M_PI / 180.0;
        double tan_theta = std::tan(theta);
        double normal_x = tan_theta;
        double normal_y = 0.0;
        double normal_z = -1.0;
        double normal_magnitude = std::sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);
        normal_x /= normal_magnitude;
        normal_y /= normal_magnitude;
        normal_z /= normal_magnitude;

        // Ensure normal faces upward
        if (normal_z < 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
            normal_z = -normal_z;
            tan_theta = -tan_theta;
            std::cout << "Flipped normal to face upward: (" << normal_x << ", " << normal_y << ", " << normal_z << ")" << std::endl;
        }

        // Noise distributions
        std::normal_distribution<double> plane_noise(0.0, params.roughness);
        std::normal_distribution<double> terrain_noise_dist(0.0, terrain_noise);
        std::normal_distribution<double> unevenness_dist(0.0, max_unevenness / 3.0);

        // Generate points
        std::vector<std::tuple<double, double, double>> inclined_points;
        for (double x_val : grid) {
            for (double y_val : grid) {
                double z_val = 0.0;
                float r = 0.5, g = 0.5, b = 0.5;
                bool in_inclined = (std::abs(x_val) <= inclined_half && std::abs(y_val) <= inclined_half);

                if (in_inclined) {
                    double z_base = x_val * tan_theta;
                    double noise = plane_noise(rng);
                    double dx = noise * normal_x;
                    double dy = noise * normal_y;
                    double dz = noise * normal_z;
                    x_val += dx;
                    y_val += dy;
                    z_val = z_base + dz;
                    r = 0.0; g = 1.0; b = 0.0;
                    inclined_points.emplace_back(x_val, y_val, z_val - z_base);
                } else {
                    z_val = 0.0;
                    z_val += unevenness_dist(rng);
                    z_val += terrain_noise_dist(rng);
                    z_val = std::max(-max_terrain_height, std::min(max_terrain_height, z_val));
                    r = 1.0; g = 0.0; b = 0.0;
                }

                uint32_t rgb = ((uint32_t)(r * 255) << 16) | ((uint32_t)(g * 255) << 8) | (uint32_t)(b * 255);
                float intensity;
                std::memcpy(&intensity, &rgb, sizeof(float));
                cloud->points.emplace_back(static_cast<float>(x_val), static_cast<float>(y_val), static_cast<float>(z_val), intensity);
            }
        }

        // Compute relief for inclined plane
        double d_min = std::numeric_limits<double>::max();
        double d_max = std::numeric_limits<double>::lowest();
        for (const auto& [x, y, z_pert] : inclined_points) {
            double distance = z_pert * normal_z;
            d_min = std::min(d_min, distance);
            d_max = std::max(d_max, distance);
        }
        double R_current = d_max - d_min;

        // Add two points if relief is too small
        if (R_current < params.relief && !inclined_points.empty()) {
            double x_center = 0.0, y_center = 0.0;
            double z_base = x_center * tan_theta;
            double distance_along_normal = params.relief / 2.0;

            double noise1 = distance_along_normal;
            double x1 = x_center + noise1 * normal_x;
            double y1 = y_center + noise1 * normal_y;
            double z1 = z_base + noise1 * normal_z;
            float r = 0.0, g = 1.0, b = 0.0;
            uint32_t rgb1 = ((uint32_t)(r * 255) << 16) | ((uint32_t)(g * 255) << 8) | (uint32_t)(b * 255);
            float intensity1;
            std::memcpy(&intensity1, &rgb1, sizeof(float));
            cloud->points.emplace_back(static_cast<float>(x1), static_cast<float>(y1), static_cast<float>(z1), intensity1);

            double noise2 = -distance_along_normal;
            double x2 = x_center + noise2 * normal_x;
            double y2 = y_center + noise2 * normal_y;
            double z2 = z_base + noise2 * normal_z;
            uint32_t rgb2 = ((uint32_t)(r * 255) << 16) | ((uint32_t)(g * 255) << 8) | (uint32_t)(b * 255);
            float intensity2;
            std::memcpy(&intensity2, &rgb2, sizeof(float));
            cloud->points.emplace_back(static_cast<float>(x2), static_cast<float>(y2), static_cast<float>(z2), intensity2);
        }

        // Final diagnostics
        d_min = std::numeric_limits<double>::max();
        d_max = std::numeric_limits<double>::lowest();
        std::vector<double> distances;
        for (const auto& pt : cloud->points) {
            if (std::abs(pt.x) <= static_cast<float>(inclined_half) && std::abs(pt.y) <= static_cast<float>(inclined_half)) {
                double x = static_cast<double>(pt.x);
                double z_base = x * tan_theta;
                double z_pert = static_cast<double>(pt.z) - z_base;
                double distance = z_pert * normal_z;
                d_min = std::min(d_min, distance);
                d_max = std::max(d_max, distance);
                distances.push_back(distance);
            }
        }
        double R_final = d_max - d_min;
        double sigma_d_final = 0.0;
        if (!distances.empty()) {
            double mean = 0.0;
            for (double d : distances) {
                mean += d;
            }
            mean /= distances.size();
            double variance = 0.0;
            for (double d : distances) {
                variance += (d - mean) * (d - mean);
            }
            variance /= distances.size();
            sigma_d_final = std::sqrt(variance);
        }
        std::cout << "Generated point cloud: Relief = " << R_final << " (target: " << params.relief
                  << "), Roughness = " << sigma_d_final << " (target: " << params.roughness
                  << "), Normal Z = " << normal_z << std::endl;

        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }
};

// ParameterVariationManager class
class ParameterVariationManager {
private:
    std::mt19937 rng;
    const double min_radius = 2.5;
    const double max_radius = 4.0;
    const double min_slope = 0.0;
    const double max_slope = 25.0;
    const double min_density = 30.0;
    const double max_density = 70.0;
    const double min_relief = 0.0;
    const double max_relief = 0.25;
    const double min_roughness = 0.0;
    const double max_roughness = 0.02;
    const std::vector<double> radius_values = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    const std::vector<double> slope_values = {10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};
    const std::vector<double> density_values = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0};
    const std::vector<double> relief_values = {0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40};
    const std::vector<double> roughness_values = {0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035};

public:
    ParameterVariationManager(unsigned int seed) : rng(seed) {}

    LandingZoneParams generateParams(const std::string& selected_param, double selected_value) {
        LandingZoneParams params;
        std::uniform_real_distribution<double> radius_dist(min_radius, max_radius);
        std::uniform_real_distribution<double> slope_dist(min_slope, max_slope);
        std::uniform_real_distribution<double> density_dist(min_density, max_density);
        std::uniform_real_distribution<double> relief_dist(min_relief, max_relief);
        std::uniform_real_distribution<double> roughness_dist(min_roughness, max_roughness);
        if (selected_param == "radius") {
            params.radius = selected_value;
        } else {
            params.radius = radius_dist(rng);
        }
        if (selected_param == "slope") {
            params.slope_degrees = selected_value;
        } else {
            params.slope_degrees = slope_dist(rng);
        }
        if (selected_param == "density") {
            params.point_density = selected_value;
        } else {
            params.point_density = density_dist(rng);
        }
        if (selected_param == "relief") {
            params.relief = selected_value;
        } else {
            params.relief = relief_dist(rng);
        }
        if (selected_param == "roughness") {
            params.roughness = selected_value;
        } else {
            params.roughness = roughness_dist(rng);
        }
        return params;
    }

    const std::vector<double>& getVariationValues(const std::string& param) {
        if (param == "radius") return radius_values;
        if (param == "slope") return slope_values;
        if (param == "density") return density_values;
        if (param == "relief") return relief_values;
        if (param == "roughness") return roughness_values;
        static std::vector<double> empty;
        return empty;
    }
};

// SimulationRunner class
class SimulationRunner {
private:
    PointCloudGenerator& generator;
    ParameterVariationManager& param_manager;
    std::function<bool(const PointCloudPcl&)> lzd_algorithm;
    bool enable_visualization;
    bool use_pcd_file;
    std::string pcd_file_path;

    void visualizePointCloud(const PointCloudPcl& cloud, const LandingZoneParams& params, const std::string& param, double value, int terrain_size) {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud - " + param + ": " + std::to_string(value)));
        viewer->setBackgroundColor(1, 1, 1);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->points.resize(cloud->size());
        colored_cloud->width = cloud->size();
        colored_cloud->height = 1;
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->points[i];
            colored_cloud->points[i].x = pt.x;
            colored_cloud->points[i].y = pt.y;
            colored_cloud->points[i].z = pt.z;
            uint32_t rgb;
            std::memcpy(&rgb, &pt.intensity, sizeof(uint32_t));
            colored_cloud->points[i].r = (rgb >> 16) & 0xFF;
            colored_cloud->points[i].g = (rgb >> 8) & 0xFF;
            colored_cloud->points[i].b = rgb & 0xFF;
        }
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "colored_cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->addText("Terrain Area: " + std::to_string(terrain_size) + "mX"+ std::to_string(terrain_size) + "m", 10, 130, 1.0, 1.0, 1.0, "area_text");
        viewer->addText("Radius: " + std::to_string(params.radius) + " m", 10, 110, 1.0, 1.0, 1.0, "radius_text");
        viewer->addText("Slope: " + std::to_string(params.slope_degrees) + " deg", 10, 90, 1.0, 1.0, 1.0, "slope_text");
        viewer->addText("Density: " + std::to_string(params.point_density) + " pts/m^2", 10, 70, 1.0, 1.0, 1.0, "density_text");
        viewer->addText("Relief: " + std::to_string(params.relief) + " m", 10, 50, 1.0, 1.0, 1.0, "relief_text");
        viewer->addText("Roughness: " + std::to_string(params.roughness) + " m", 10, 30, 1.0, 1.0, 1.0, "roughness_text");
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        viewer->close();
    }

    ResourceMetrics measureResources(const std::function<void()>& task) {
        struct rusage usage_start, usage_end;
        ResourceMetrics metrics = {0.0, 0.0, 0};
        if (getrusage(RUSAGE_SELF, &usage_start) != 0) {
            std::cerr << "Error getting resource usage (start)" << std::endl;
            return metrics;
        }
        task();
        if (getrusage(RUSAGE_SELF, &usage_end) != 0) {
            std::cerr << "Error getting resource usage (end)" << std::endl;
            return metrics;
        }
        metrics.user_time = (usage_end.ru_utime.tv_sec - usage_start.ru_utime.tv_sec) +
                            (usage_end.ru_utime.tv_usec - usage_start.ru_utime.tv_usec) / 1e6;
        metrics.system_time = (usage_end.ru_stime.tv_sec - usage_start.ru_stime.tv_sec) +
                              (usage_end.ru_stime.tv_usec - usage_start.ru_stime.tv_usec) / 1e6;
        metrics.max_rss = usage_end.ru_maxrss;
        return metrics;
    }

public:
    SimulationRunner(PointCloudGenerator& gen, ParameterVariationManager& pm, std::function<bool(const PointCloudPcl&)> lzd,
                     bool enable_viz, bool use_pcd, const std::string& pcd_path)
        : generator(gen), param_manager(pm), lzd_algorithm(lzd), enable_visualization(enable_viz),
          use_pcd_file(use_pcd), pcd_file_path(pcd_path) {}

    void runSimulations(const std::string& param, std::vector<double>& success_rates,
                        std::vector<double>& avg_times, std::vector<double>& avg_memories,
                        int num_simulations, int terrain_size) {
        const auto& values = param_manager.getVariationValues(param);
        success_rates.resize(values.size(), 0.0);
        avg_times.resize(values.size(), 0.0);
        avg_memories.resize(values.size(), 0.0);
        std::vector<int> counts(values.size(), 0);

        // Load PCD file once if use_pcd_file is true
        PointCloudPcl pcd_cloud = nullptr;
        if (use_pcd_file) {
            pcd_cloud = loadPointCloudFromPCD(pcd_file_path);
            if (!pcd_cloud) {
                std::cerr << "Failed to load PCD file. Exiting simulations." << std::endl;
                return;
            }
        }

        for (size_t i = 0; i < values.size(); ++i) {
            double value = values[i];
            int successes = 0;
            double total_time = 0.0;
            double total_memory = 0.0;
            for (int j = 0; j < num_simulations; ++j) {
                PointCloudPcl cloud;
                LandingZoneParams params;
                if (use_pcd_file) {
                    cloud = pcd_cloud;
                    params = param_manager.generateParams(param, value);
                } else {
                    params = param_manager.generateParams(param, value);
                    cloud = generator.generatePointCloud(params);
                }

                if (!cloud) {
                    std::cerr << "Error: Null point cloud. Skipping simulation." << std::endl;
                    continue;
                }

                if (enable_visualization) {
                    visualizePointCloud(cloud, params, param, value, terrain_size);
                }

                bool success = false;
                ResourceMetrics metrics = measureResources([&]() {
                    success = lzd_algorithm(cloud);
                });

                if (success) {
                    successes++;
                }
                total_time += metrics.user_time + metrics.system_time;
                total_memory += metrics.max_rss / 1024.0;
                counts[i]++;
            }
            success_rates[i] = counts[i] > 0 ? static_cast<double>(successes) / num_simulations : 0.0;
            avg_times[i] = counts[i] > 0 ? total_time / num_simulations : 0.0;
            avg_memories[i] = counts[i] > 0 ? total_memory / num_simulations : 0.0;
            std::cout << "Parameter: " << param << ", Value: " << value
                      << ", Success Rate: " << success_rates[i]
                      << ", Avg Time: " << avg_times[i] << " s"
                      << ", Avg Memory: " << avg_memories[i] << " MB" << std::endl;
        }
    }
};

// ResultProcessor class
class ResultProcessor {
public:
    void saveResults(const std::string& algo, const std::string& param, const std::vector<double>& values,
                     const std::vector<double>& success_rates, const std::vector<double>& avg_times,
                     const std::vector<double>& avg_memories, const std::string& output_dir, int num_simulations) {
        std::string filename = output_dir + "/results_" + algo + "_" + param + ".csv";
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
            return;
        }
        out << "Algorithm,Parameter,Value,SuccessRate,AvgTime,AvgMemory,NumSimulations\n";
        for (size_t i = 0; i < values.size(); ++i) {
            out << algo << "," << param << "," << values[i] << ","
                << success_rates[i] << "," << avg_times[i] << "," << avg_memories[i] << ","
                << num_simulations << "\n";
        }
        out.close();
        std::cout << "Saved results to " << filename << std::endl;
    }

    void generatePlotScript(const std::string& algo, const std::string& param, const std::string& output_dir,
                           double x_min, double x_max, int num_simulations) {
        if (param == "radius") {
            x_min = 1.0; x_max = 4.0;
        } else if (param == "slope") {
            x_min = 10.0; x_max = 40.0;
        } else if (param == "density") {
            x_min = 10.0; x_max = 70.0;
        } else if (param == "relief") {
            x_min = 0.0; x_max = 0.2;
        } else if (param == "roughness") {
            x_min = 0.0; x_max = 0.04;
        } else {
            x_min = 0.0; x_max = 1.0;
        }
        std::string csv_filename = "results_" + algo + "_" + param + ".csv";
        std::string py_filename = output_dir + "/plot_" + algo + "_" + param + ".py";
        std::string png_filename = "plot_" + algo + "_" + param + ".png";
        std::ofstream script(py_filename);
        if (!script.is_open()) {
            std::cerr << "Error: Could not open file " << py_filename << " for writing" << std::endl;
            return;
        }
        script << "import pandas as pd\n"
               << "import matplotlib.pyplot as plt\n"
               << "data = pd.read_csv('" << csv_filename << "')\n"
               << "fig, ax1 = plt.subplots()\n"
               << "ax1.plot(data['Value'].to_numpy(), data['SuccessRate'].to_numpy(), marker='o', label='Success Rate', color='b')\n"
               << "ax1.set_xlabel('" << param << "')\n"
               << "ax1.set_ylabel('Success Rate', color='b')\n"
               << "ax1.tick_params(axis='y', labelcolor='b')\n"
               << "ax1.set_xlim(" << x_min << ", " << x_max << ")\n"
               << "ax1.set_ylim(0, 1)\n"
               << "ax1.grid(True)\n"
               << "ax2 = ax1.twinx()\n"
               << "ax2.plot(data['Value'].to_numpy(), data['AvgTime'].to_numpy(), marker='s', label='Avg Time (s)', color='r')\n"
               << "ax2.set_ylabel('Average Time (s)', color='r')\n"
               << "ax2.tick_params(axis='y', labelcolor='r')\n"
               << "ax3 = ax1.twinx()\n"
               << "ax3.spines['right'].set_position(('outward', 60))\n"
               << "ax3.plot(data['Value'].to_numpy(), data['AvgMemory'].to_numpy(), marker='^', label='Avg Memory (MB)', color='g')\n"
               << "ax3.set_ylabel('Average Memory (MB)', color='g')\n"
               << "ax3.tick_params(axis='y', labelcolor='g')\n"
               << "plt.title('LZD Algorithm (" << algo << ") vs " << param << "')\n"
               << "lines1, labels1 = ax1.get_legend_handles_labels()\n"
               << "lines2, labels2 = ax2.get_legend_handles_labels()\n"
               << "lines3, labels3 = ax3.get_legend_handles_labels()\n"
               << "ax1.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, loc='best', title='Simulations per value: " << num_simulations << "')\n"
               << "plt.savefig('" << png_filename << "', dpi=300, bbox_inches='tight')\n"
               << "plt.show()\n";
        script.close();
        std::cout << "Generated plot script: " << py_filename << std::endl;
    }
};

// Main function
int main() {
    std::string config_path = "/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/lib/config/monte_carlo_benchmarking_config.yaml";
    //std::string config_path = "/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/lib/config/algo_testing_config_local.yaml";

    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file '" << config_path << "': " << e.what() << std::endl;
        return 1;
    }

    // Read new parameters
    bool use_pcd_file = config["use_pcd_file"].as<bool>(false);
    std::string pcd_file_path = config["pcd_file_path"].as<std::string>("");

    // Validate pcd_file_path if use_pcd_file is true
    if (use_pcd_file && pcd_file_path.empty()) {
        std::cerr << "Error: use_pcd_file is true, but pcd_file_path is empty" << std::endl;
        return 1;
    }
    if (use_pcd_file && !std::filesystem::exists(pcd_file_path)) {
        std::cerr << "Error: PCD file does not exist at " << pcd_file_path << std::endl;
        return 1;
    }

    int num_simulations = config["num_simulations"].as<int>(100);
    int num_hazard_metrics = 5; 
    int num_algorithms = 5; 
    int num_of_discrete_values_per_hazard_metric = 7;
    int total_num_simulations = num_simulations * num_hazard_metrics * num_algorithms * num_of_discrete_values_per_hazard_metric;
    bool enable_visualization = config["enable_visualization"].as<bool>(false);

    double min_radius_threshold = config["min_radius_threshold"].as<double>(2.5);
    double max_slope_threshold = config["max_slope_threshold"].as<double>(25.0);
    double min_point_density_threshold = config["min_point_density_threshold"].as<double>(30.0);
    double max_relief_threshold = config["max_relief_threshold"].as<double>(0.1);
    double max_roughness_threshold = config["max_roughness_threshold"].as<double>(0.02);

    int max_landingZones = config["max_landingZones"].as<int>(5);
    int max_Attempts = config["max_Attempts"].as<int>(1000);
    double step_size = config["step_size"].as<double>(1.0);

    double terrain_size = config["terrain_size"].as<double>(20.0);
    int terrain_size_int = static_cast<int>(terrain_size);

    // Create timestamped directory
    std::time_t now = std::time(nullptr);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string base_dir = "/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/results";
    std::string output_dir = base_dir + "/results_" + timestamp + "_" + std::to_string(terrain_size_int) + "mX" + std::to_string(terrain_size_int) + "m_totalSimNum" + std::to_string(total_num_simulations);

    if (!std::filesystem::exists(base_dir)) {
        if (!std::filesystem::create_directories(base_dir)) {
            std::cerr << "Error: Could not create base directory " << base_dir << std::endl;
            return 1;
        }
    }
    if (!std::filesystem::create_directories(output_dir)) {
        std::cerr << "Error: Could not create timestamped directory " << output_dir << std::endl;
        return 1;
    }
    std::cout << "Created output directory: " << output_dir << std::endl;

    unsigned int seed = static_cast<unsigned int>(std::time(nullptr));
    PointCloudGenerator generator(seed, terrain_size);
    ParameterVariationManager param_manager(seed);

    std::vector<std::pair<std::string, std::function<bool(const PointCloudPcl&)>>> algorithms;
    for (const auto& algo_config : config["algorithms"]) {
        std::string algo_name = algo_config["name"].as<std::string>();
        if (algo_name == "Region_Growing_Segmentation") {
            float curvature_threshold = algo_config["curvature_threshold"].as<float>(0.005);
            int min_cluster_size = algo_config["min_cluster_size"].as<int>(50);
            float alpha = algo_config["alpha"].as<float>(0.1);
            double cluster_tolerance = algo_config["cluster_tolerance"].as<double>(0.1);
            algorithms.emplace_back(algo_name, [=](const PointCloudPcl& cloud) {
                auto flat_regions = segmentPointCloud(cloud, curvature_threshold, max_slope_threshold, min_cluster_size);
                CircleFitResult circle_result = circleFitting(flat_regions.inlier_cloud, min_radius_threshold, alpha, max_landingZones,
                    max_slope_threshold, cluster_tolerance, enable_visualization);
                CircleFitResult ranked_result = checkVerticalCollisionAndHazardMetrics(cloud, circle_result, step_size, min_point_density_threshold,
                                                        max_relief_threshold, max_roughness_threshold, enable_visualization);
                if (enable_visualization) {
                    visualizeRankedCandidates(cloud, ranked_result, flat_regions.cluster_indices);
                }
                return ranked_result.centers.size() > 0;
            });
        } else if (algo_name == "seq_overlap") {
            float cell_size = algo_config["cell_size"].as<float>(1.5);
            float alpha = algo_config["alpha"].as<float>(0.1);
            float cluster_tolerance = algo_config["cluster_tolerance"].as<float>(1.0);
            algorithms.emplace_back(algo_name, [=](const PointCloudPcl& cloud) {
                auto seqoverlapResult = sequentialOverlappingApproach(cloud, cell_size, max_slope_threshold);
                CircleFitResult circle_result = circleFitting(seqoverlapResult.inlier_cloud, min_radius_threshold, alpha,
                                                             max_landingZones, max_slope_threshold, cluster_tolerance, enable_visualization);
                CircleFitResult ranked_result = checkVerticalCollisionAndHazardMetrics(cloud, circle_result, step_size, min_point_density_threshold,
                    max_relief_threshold, max_roughness_threshold, enable_visualization);
                if (enable_visualization) {
                    visualizePointCloud(cloud, seqoverlapResult);
                    visualizeRankedCandidates(cloud, ranked_result, seqoverlapResult.cluster_indices);
                }
                return ranked_result.centers.size() > 0;
            });
        } else if (algo_name == "sequentialApproach") {
            float cell_size = algo_config["cell_size"].as<float>(1.5);
            float alpha = algo_config["alpha"].as<float>(0.1);
            float cluster_tolerance = algo_config["cluster_tolerance"].as<float>(1.0);
            algorithms.emplace_back(algo_name, [=](const PointCloudPcl& cloud) {
                auto seqoverlapResult = sequentialApproach(cloud, cell_size, max_slope_threshold);
                CircleFitResult circle_result = circleFitting(seqoverlapResult.inlier_cloud, min_radius_threshold, alpha,
                                                             max_landingZones, max_slope_threshold, cluster_tolerance, enable_visualization);
                CircleFitResult ranked_result = checkVerticalCollisionAndHazardMetrics(cloud, circle_result, step_size, min_point_density_threshold,
                    max_relief_threshold, max_roughness_threshold, enable_visualization);
                if (enable_visualization) {
                    visualizePointCloud(cloud, seqoverlapResult);
                    visualizeRankedCandidates(cloud, ranked_result, seqoverlapResult.cluster_indices);
                }
                return ranked_result.centers.size() > 0;
            });
        } else if (algo_name == "sequentialApproachKdtree") {
            float cell_size = algo_config["cell_size"].as<float>(1.5);
            float alpha = algo_config["alpha"].as<float>(0.1);
            float cluster_tolerance = algo_config["cluster_tolerance"].as<float>(1.0);
            algorithms.emplace_back(algo_name, [=](const PointCloudPcl& cloud) {
                auto sequentialApproachKdtree_result = sequentialApproachKdtree(cloud, cell_size, max_slope_threshold);
                CircleFitResult circle_result = circleFitting(sequentialApproachKdtree_result.inlier_cloud, min_radius_threshold, alpha,
                                                             max_landingZones, max_slope_threshold, cluster_tolerance, enable_visualization);
                CircleFitResult ranked_result = checkVerticalCollisionAndHazardMetrics(cloud, circle_result, step_size, min_point_density_threshold,
                    max_relief_threshold, max_roughness_threshold, enable_visualization);
                if (enable_visualization) {
                    visualizePointCloud(cloud, sequentialApproachKdtree_result);
                    visualizeRankedCandidates(cloud, ranked_result, sequentialApproachKdtree_result.cluster_indices);
                }
                return ranked_result.centers.size() > 0;
            });
        } else if (algo_name == "kdtree_InflatingCircles") {
            int k = algo_config["k"].as<int>(30);
            float radiusIncrement = algo_config["radiusIncrement"].as<float>(0.1);
            algorithms.emplace_back(algo_name, [=](const PointCloudPcl& cloud) {
                auto candidatePoints = kdtreeNeighbourhoodPCAFilterOMP(cloud, min_radius_threshold, k, max_slope_threshold,
                                                                      max_landingZones, max_Attempts,
                                                                      radiusIncrement);
                CircleFitResult ranked_result = checkVerticalCollisionAndHazardMetrics(cloud, candidatePoints, step_size, min_point_density_threshold,
                    max_relief_threshold, max_roughness_threshold, enable_visualization);
                if (enable_visualization) {
                    std::vector<pcl::PointIndices> cluster_indices;
                    visualizeRankedCandidates(cloud, ranked_result, cluster_indices);
                }
                return ranked_result.centers.size() > 0;
            });
        }
    }

    std::vector<std::string> parameters = {"radius", "slope", "density", "relief", "roughness"};
    ResultProcessor processor;
    for (const auto& [algo_name, lzd_algo] : algorithms) {
        SimulationRunner runner(generator, param_manager, lzd_algo, enable_visualization, use_pcd_file, pcd_file_path);
        for (const auto& param : parameters) {
            std::vector<double> success_rates, avg_times, avg_memories;
            runner.runSimulations(param, success_rates, avg_times, avg_memories, num_simulations, terrain_size_int);
            processor.saveResults(algo_name, param, param_manager.getVariationValues(param),
                                 success_rates, avg_times, avg_memories, output_dir, num_simulations);
            processor.generatePlotScript(algo_name, param, output_dir, 0.0, 1.0, num_simulations);
        }
    }

    std::cout << "Simulations complete. Run the generated Python scripts in " << output_dir << " to visualize results." << std::endl;
    return 0;
}