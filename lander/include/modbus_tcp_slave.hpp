#ifndef KAANH_PROTOCOL_MODBUS_TCP_SLAVE_HPP_
#define KAANH_PROTOCOL_MODBUS_TCP_SLAVE_HPP_

#include <string>
#include <memory>

#include "kaanh/general/macro.hpp"
#include "kaanh_lib_export.h"

namespace kaanh::protocol {

class KAANH_API ModbusTcpSlave {
public:
    auto ip() const noexcept->const std::string&;
    auto setIp(const std::string &ip)->void;
    auto port() const noexcept->int;
    auto setPort(int port)->void;
    auto setNb(int nb_bits, int nb_input_bits, int nb_regs, int nb_input_regs)->void;
    auto setStart(int start_bits, int start_input_bits, int start_regs, int start_input_regs)->void;
    auto start()->void;
    auto stop()->void;
    auto isServing() const noexcept->bool;

    auto nbBits() const noexcept->int;
    auto nbInputBits() const noexcept->int;
    auto nbRegs() const noexcept->int;
    auto nbInputRegs() const noexcept->int;

    auto startBits() const noexcept->int;
    auto startInputBits() const noexcept->int;
    auto startRegs() const noexcept->int;
    auto startInputRegs() const noexcept->int;

    auto ptrBits() const noexcept->uint8_t*;
    auto ptrInputBits() const noexcept->uint8_t*;
    auto ptrRegs() const noexcept->uint16_t*;
    auto ptrInputRegs() const noexcept->uint16_t*;

    static auto setFloat(float val, uint16_t *des)->void;
    static auto getFloat(uint16_t *src)->float;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;

public:
    ModbusTcpSlave(const std::string &ip="0.0.0.0", int port=502);
    virtual ~ModbusTcpSlave();
    KAANH_DELETE_BIG_FOUR(ModbusTcpSlave)
};

}   // namespace kaanh::protocol

#endif  // KAANH_PROTOCOL_MODBUS_TCP_SLAVE_HPP_