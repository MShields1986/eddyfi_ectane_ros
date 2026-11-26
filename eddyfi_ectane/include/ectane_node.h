#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "eddyfi_ectane/msg/channel.hpp"
#include "eddyfi_ectane/msg/observation.hpp"
#include "eddyfi_ectane/srv/balance_device.hpp"
#include "eddyfi_ectane/srv/stream_data.hpp"

#include "c_sdk_data_structures.h"
#include "channel_naming.h"
#include "ectane_handler.h"


namespace ectane_namespace {

struct ChannelConfig {
    std::string name;
    int time_slot;
    int input;
    int frequency_index;     // 1-based
    float x_position;
    float y_position;
};

class EctaneNode : public rclcpp::Node
{
public:
    explicit EctaneNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~EctaneNode() override;

private:
    Configurations                       configurationsFromString(const std::string& str);

    void                                 getChannelNames();
    void                                 getChannelConfigurations();
    void                                 validateFrequencies();

    void                                 initHardware();
    void                                 balanceDevice(const eddyfi_ectane::srv::BalanceDevice::Request& request,
                                                       eddyfi_ectane::srv::BalanceDevice::Response& response);
    void                                 streamData(const eddyfi_ectane::srv::StreamData::Request& request,
                                                    eddyfi_ectane::srv::StreamData::Response& response);
    void                                 updateStreaming();
    void                                 acquisitionLoop();
    bool                                 populateEcData(eddyfi_ectane::msg::Observation& observation);
    void                                 shutdown();

    std::vector<std::string>                                 channel_names_;
    std::vector<ChannelConfig>                               channel_configs_;     // In C SDK data stream order
    int                                                      num_frequencies_;

    std::string                                              ectane_address_;
    std::string                                              configuration_str_;
    int32_t                                                  acquisition_rate_;
    int32_t                                                  gain_;
    std::vector<int32_t>                                     frequencies_;
    std::vector<float>                                       voltages_;

    rclcpp::Publisher<eddyfi_ectane::msg::Observation>::SharedPtr    publisher_;
    rclcpp::Service<eddyfi_ectane::srv::BalanceDevice>::SharedPtr    balance_service_;
    rclcpp::Service<eddyfi_ectane::srv::StreamData>::SharedPtr       stream_service_;

    std::unique_ptr<EctaneHandler>                           ectane_handler_;
    std::vector<int>                                         ec_data_raw_;
    eddyfi_ectane::msg::Observation                          observation_template_;   // Channels, copied into each observation

    // Reading the C SDK data stream blocks, so it runs on its own thread rather than holding up an executor
    std::atomic<bool>                                        streaming_;
    std::atomic<bool>                                        stopping_;
    std::thread                                              acquisition_thread_;

    std::once_flag                                           shutdown_once_;
    rclcpp::PreShutdownCallbackHandle                        pre_shutdown_handle_;
};

} // namespace ectane_namespace
