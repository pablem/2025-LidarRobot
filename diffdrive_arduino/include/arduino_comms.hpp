#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  // Devuelve false si la respuesta no es exactamente "<int> <int>\r\n" (timeout →
  // vacía, línea cortada o mezclada con otra respuesta). En ese caso val_1/val_2
  // no se tocan: antes atoi("") daba 0 y la odometría veía un salto enorme.
  bool read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    const char *start = response.c_str();
    char *end_1 = nullptr;
    errno = 0;
    long v1 = std::strtol(start, &end_1, 10);
    if (end_1 == start || *end_1 != ' ' || errno == ERANGE) return false;

    char *end_2 = nullptr;
    long v2 = std::strtol(end_1, &end_2, 10);
    if (end_2 == end_1 || errno == ERANGE) return false;

    for (const char *c = end_2; *c != '\0'; ++c)
    {
      if (*c != '\r' && *c != '\n') return false;
    }
    if (v1 < INT_MIN || v1 > INT_MAX || v2 < INT_MIN || v2 > INT_MAX) return false;

    val_1 = static_cast<int>(v1);
    val_2 = static_cast<int>(v2);
    return true;
  }
  float read_battery_voltage()
  {
    std::string response = send_msg("b\r");
    return std::atof(response.c_str());
  }

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP