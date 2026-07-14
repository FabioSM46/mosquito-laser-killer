#include "hal/spi_impl.h"
#include "core/error.h"
#include "core/print.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

SpiImpl::SpiImpl(const std::string& device, uint32_t speed_hz)
    : device_(device), speed_hz_(speed_hz) {
}

SpiImpl::~SpiImpl() {
    cleanup();
}

SpiImpl::SpiImpl(SpiImpl&& other) noexcept
    : device_(std::move(other.device_)), speed_hz_(other.speed_hz_), fd_(other.fd_) {
    other.fd_ = -1;
}

auto SpiImpl::operator=(SpiImpl&& other) noexcept -> SpiImpl& {
    if (this != &other) {
        cleanup();
        device_ = std::move(other.device_);
        speed_hz_ = other.speed_hz_;
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

auto SpiImpl::cleanup() -> void {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

auto SpiImpl::transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx)
    -> std::expected<void, HardwareError> {
    if (fd_ < 0) {
        fd_ = open(device_.c_str(), O_RDWR);
        if (fd_ < 0) {
            println(stderr, "[SPI] Failed to open {}: {}", device_, strerror(errno));
            return std::unexpected(HardwareError::SpiOpenFailed);
        }

        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
            println(stderr, "[SPI] ioctl config failed: {}", strerror(errno));
            cleanup();
            return std::unexpected(HardwareError::SpiOpenFailed);
        }

        println("[SPI] Opened {} at {} Hz, Mode 0", device_, speed_hz_);
    }

    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = std::min(tx.size(), rx.size());
    tr.speed_hz = speed_hz_;
    tr.bits_per_word = 8;

    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
        println(stderr, "[SPI] Transfer failed: {}", strerror(errno));
        return std::unexpected(HardwareError::SpiTransferFailed);
    }

    return {};
}

auto SpiImpl::write16(uint16_t value) -> std::expected<void, HardwareError> {
    uint8_t tx[2] = {
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };
    uint8_t rx[2]{};
    return transfer(std::span(tx), std::span(rx));
}

auto SpiImpl::is_open() const -> bool {
    return fd_ >= 0;
}
