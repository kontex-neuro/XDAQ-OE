#pragma once
#include <vector>

#include "intan_chip.h"

class Ports
{
public:
    const int num_of_ports;
    const int spi_per_port;
    const int chips_per_spi;
    const bool ddr;
    const int num_of_spi;
    const int max_streams_per_chip;
    const int max_streams;
    const int max_non_ddr_streams;
    const int max_non_ddr_streams_per_spi;
    const int max_chips;
    const int max_streams_per_port;
    const int max_non_ddr_streams_per_port;
    const int max_chips_per_port;
    const int num_of_adc;
    const int num_of_dac;
    const int num_of_dig_in;
    const int num_of_dig_out;

    std::vector<IntanChip::Chip> chips;

    Ports(int num_of_ports, int spi_per_port, int chips_per_spi, bool ddr, int num_of_adc,
          int num_of_dac, int num_of_dig_in, int num_of_dig_out)
        : num_of_ports(num_of_ports),
          spi_per_port(spi_per_port),
          chips_per_spi(chips_per_spi),
          ddr(ddr),
          num_of_spi(num_of_ports * spi_per_port),
          max_streams_per_chip(ddr ? 2 : 1),
          max_streams(num_of_ports * spi_per_port * chips_per_spi * (ddr ? 2 : 1)),
          max_non_ddr_streams(num_of_ports * spi_per_port * chips_per_spi),
          max_non_ddr_streams_per_spi(chips_per_spi),
          max_chips(num_of_ports * spi_per_port * chips_per_spi),
          max_streams_per_port(spi_per_port * chips_per_spi * (ddr ? 2 : 1)),
          max_non_ddr_streams_per_port(spi_per_port * chips_per_spi),
          max_chips_per_port(spi_per_port * chips_per_spi),
          num_of_adc(num_of_adc),
          num_of_dac(num_of_dac),
          num_of_dig_in(num_of_dig_in),
          num_of_dig_out(num_of_dig_out),
          chips(num_of_ports * spi_per_port * chips_per_spi)
    {
    }
    int port_from_chip(int chip) const { return chip / max_chips_per_port; }
    bool is_ddr(int stream) const { return ddr && (stream % 2 == 1); }
};

class XDAQPortRHD : public Ports
{
public:
    XDAQPortRHD() : Ports(4, 2, 2, true, 8, 8, 32, 32) {}
};