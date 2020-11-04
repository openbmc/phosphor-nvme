#pragma once

#include <boost/asio.hpp>
#include <cstring>
#include <fstream>
#include <gpiod.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <vector>

constexpr auto nvmePowerService = "xyz.openbmc_project.Control.Nvme.Power";
constexpr auto nvmePowerIface = "xyz.openbmc_project.Control.Nvme.Power";
constexpr auto nvmePowerPath = "/xyz/openbmc_project/control/nvme/";

namespace nvme_power_control
{
struct GpioConfig
{
    std::string linename;
    uint8_t defaultGpioValue;
};

std::vector<struct GpioConfig> loadGpiosConfig(const std::string& path);

class NvmePowerCtrl
{
  public:
    friend class TestNvmePowerCtrl;

    NvmePowerCtrl() = delete;
    ~NvmePowerCtrl() = default;

    /** @brief Constructs NvmePowerCtrl
     *
     * @param[in] ioc - boost IO service object.
     * @param[in] conn - sdbusplus bus client connection.
     * @param[in] objServer - sdbusplus object server.
     * @param[in] configs - struct GpioConfig configs.
     */
    NvmePowerCtrl(boost::asio::io_context& ioc,
                  sdbusplus::asio::connection& conn,
                  sdbusplus::asio::object_server& objServer,
                  std::vector<struct GpioConfig> configs) :
        ioc_(ioc),
        conn_(conn), objServer_(objServer), configs_(configs)
    {
        createNvmePowerCtrl(configs_);
    }

  private:
    /** @brief boost IO service object. */
    boost::asio::io_context& ioc_;
    /** @brief sdbusplus bus client connection. */
    sdbusplus::asio::connection& conn_;
    /** @brief sdbusplus object server. */
    sdbusplus::asio::object_server& objServer_;
    /** @brief D-bus interfaces. */
    std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>> interfaces;
    /** @brief struct GpioConfig configs. */
    std::vector<struct GpioConfig> configs_;

    /**
     *  @brief Create NvmePowerCtrl D-bus Objects.
     *  @param[in] configs - struct GpioConfig configs.
     */
    void createNvmePowerCtrl(std::vector<struct GpioConfig> configs);

    /**
     *  @brief Set GPIO output to the specified value.
     *  @param[in] name - GPIO line name.
     *  @param[in] value - GPIO signal value.
     *  @param[in] gpioLine - libgpiod GPIO line.
     */
    bool setGPIOOutput(const std::string& name, const int value,
                       gpiod::line& gpioLine);
};
} // namespace nvme_power_control
