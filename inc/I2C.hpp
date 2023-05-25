#pragma once
#ifndef __cpp_exceptions
#error I2C class can only be used when __cpp_exceptions is enabled. Enable CONFIG_COMPILER_CXX_EXCEPTIONS in Kconfig
#endif

#include <memory>
#include <chrono>
#include <vector>
#include <list>
#include <future>
#include "esp_timer.h"
#include "Exceptions.hpp"
#include "Gpio.hpp"
#ifndef millis
#define millis() (esp_timer_get_time() / 1000UL)
#endif
using namespace System;
namespace Components
{

    esp_err_t checkI2CAddress(uint32_t addr) noexcept;

    struct I2CException : public ESPException
    {
        I2CException(esp_err_t error);
    };

    struct I2CTransferException : public I2CException
    {
        I2CTransferException(esp_err_t error);
    };

    class SDA_type;
    using SDA_GPIO = GPIONumBase<class SDA_type>;
    class SCL_type;
    using SCL_GPIO = GPIONumBase<class SCL_type>;

    class I2CNumber : public StrongValueComparable<uint32_t>
    {
        constexpr explicit I2CNumber(uint32_t number) : StrongValueComparable<uint32_t>(number) {}

    public:
        constexpr static I2CNumber I2C0()
        {
            return I2CNumber(0);
        }

#if CONFIG_SOC_I2C_NUM == 2
        constexpr static I2CNumber I2C1()
        {
            return I2CNumber(1);
        }
#endif
    };

    class I2CAddress : public StrongValueComparable<uint8_t>
    {
    public:
        explicit I2CAddress(uint8_t addr);
    };

    class I2CCommandLink
    {
    public:
        I2CCommandLink();
        ~I2CCommandLink();

        I2CCommandLink(const I2CCommandLink &) = delete;
        I2CCommandLink operator=(const I2CCommandLink &) = delete;

        void start();
        void write(const std::vector<uint8_t> &bytes, bool expect_ack = true);
        void writeByte(uint8_t byte, bool expect_ack = true);
        void read(std::vector<uint8_t> &bytes);
        void stop();
        void executeTransfer(I2CNumber i2c_num, std::chrono::milliseconds driver_timeout);

    private:
        void *handle;
    };
    template <typename TReturn>
    class I2CTransfer
    {
    public:
        typedef TReturn TransferReturnT;

        I2CTransfer(std::chrono::milliseconds driver_timeout_arg = std::chrono::milliseconds(1000));

        virtual ~I2CTransfer() {}
        TReturn doTransfer(I2CNumber i2c_num, I2CAddress i2c_addr);

    protected:
        virtual void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) = 0;
        virtual TReturn processResult() = 0;
        std::chrono::milliseconds driver_timeout;
    };

    class I2CBus
    {
    public:
        explicit I2CBus(I2CNumber i2c_number);
        virtual ~I2CBus();
        const I2CNumber i2c_num;
    };

    class I2CMaster : public I2CBus
    {
    public:
        explicit I2CMaster(I2CNumber i2c_number,
                           SCL_GPIO scl_gpio,
                           SDA_GPIO sda_gpio,
                           Frequency clock_speed,
                           bool scl_pullup = true,
                           bool sda_pullup = true);

        virtual ~I2CMaster();

        template <typename TransferT>
        std::future<typename TransferT::TransferReturnT> transfer(I2CAddress i2c_addr, std::shared_ptr<TransferT> xfer);
        void syncWrite(I2CAddress i2c_addr, const std::vector<uint8_t> &data);
        std::vector<uint8_t> syncRead(I2CAddress i2c_addr, size_t n_bytes);
        std::vector<uint8_t> scan();
        std::vector<uint8_t> syncTransfer(I2CAddress i2c_addr,
                                          const std::vector<uint8_t> &write_data,
                                          size_t read_n_bytes);
    };

#if CONFIG_SOC_I2C_SUPPORT_SLAVE
    class I2CSlave : public I2CBus
    {
    public:
        I2CSlave(I2CNumber i2c_number,
                 SCL_GPIO scl_gpio,
                 SDA_GPIO sda_gpio,
                 I2CAddress slave_addr,
                 size_t rx_buf_len,
                 size_t tx_buf_len,
                 bool scl_pullup = true,
                 bool sda_pullup = true);

        virtual ~I2CSlave();
        virtual int writeRaw(const uint8_t *data, size_t data_len, std::chrono::milliseconds timeout);
        virtual int readRaw(uint8_t *buffer, size_t buffer_len, std::chrono::milliseconds timeout);
    };
#endif // CONFIG_SOC_I2C_SUPPORT_SLAVE

    class I2CWrite : public I2CTransfer<void>
    {
    public:
        I2CWrite(const std::vector<uint8_t> &bytes, std::chrono::milliseconds driver_timeout = std::chrono::milliseconds(1000));

    protected:
        void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) override;
        void processResult() override;

    private:
        std::vector<uint8_t> bytes;
    };
    class I2CRead : public I2CTransfer<std::vector<uint8_t>>
    {
    public:
        I2CRead(size_t size, std::chrono::milliseconds driver_timeout = std::chrono::milliseconds(1000));

    protected:
        void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) override;
        std::vector<uint8_t> processResult() override;

    private:
        std::vector<uint8_t> bytes;
    };

    class I2CComposed : public I2CTransfer<std::vector<std::vector<uint8_t>>>
    {
    public:
        I2CComposed(std::chrono::milliseconds driver_timeout = std::chrono::milliseconds(1000));
        void addRead(size_t size);
        void addWrite(std::vector<uint8_t> bytes);

    protected:
        void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) override;
        std::vector<std::vector<uint8_t>> processResult() override;

    private:
        class CompTransferNode
        {
        public:
            virtual ~CompTransferNode() {}
            virtual void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) = 0;
            virtual void processResult(std::vector<std::vector<uint8_t>> &read_results) {}
        };

        class CompTransferNodeRead : public CompTransferNode
        {
        public:
            CompTransferNodeRead(size_t size) : bytes(size) {}
            void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) override;

            void processResult(std::vector<std::vector<uint8_t>> &read_results) override;

        private:
            std::vector<uint8_t> bytes;
        };

        class CompTransferNodeWrite : public CompTransferNode
        {
        public:
            CompTransferNodeWrite(std::vector<uint8_t> bytes) : bytes(bytes) {}
            void queueCmd(I2CCommandLink &handle, I2CAddress i2c_addr) override;

        private:
            std::vector<uint8_t> bytes;
        };
        std::list<std::shared_ptr<CompTransferNode>> transfer_list;
    };

    template <typename TReturn>
    I2CTransfer<TReturn>::I2CTransfer(std::chrono::milliseconds driver_timeout_arg)
        : driver_timeout(driver_timeout_arg) {}

    template <typename TReturn>
    TReturn I2CTransfer<TReturn>::doTransfer(I2CNumber i2c_num, I2CAddress i2c_addr)
    {
        I2CCommandLink cmd_link;

        queueCmd(cmd_link, i2c_addr);

        cmd_link.stop();

        cmd_link.executeTransfer(i2c_num, driver_timeout);

        return processResult();
    }

    template <typename TransferT>
    std::future<typename TransferT::TransferReturnT> I2CMaster::transfer(I2CAddress i2c_addr, std::shared_ptr<TransferT> xfer)
    {
        if (!xfer)
            throw I2CException(ESP_ERR_INVALID_ARG);

        return std::async(
            std::launch::async, [this](std::shared_ptr<TransferT> xfer, I2CAddress i2c_addr)
            { return xfer->doTransfer(i2c_num, i2c_addr); },
            xfer, i2c_addr);
    }

}
