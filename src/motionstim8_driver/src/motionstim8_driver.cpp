#include "motionstim8_driver/motionstim8_driver.hpp"
#include "rclcpp/rclcpp.hpp"


namespace motionstim8_driver
{

    MotionStim8Driver::MotionStim8Driver(){}
    bool MotionStim8Driver::connect()
    {
        // Procura dispositivos série compatíveis com o MotionStim8
        stim_.FindDevices();

        // Obtém a lista de portas encontradas
        const auto devices = stim_.GetDevicesList();

        // Percorre todas as portas e tenta estabelecer comunicação
        for (const auto & serial_port : devices)
        {
            int result = stim_.Open_serial(const_cast<char *>(serial_port.c_str()));

            // Se não conseguir abrir a porta, tenta a seguinte
            if (result != 0)
            {
                continue;
            }

            // Configura a comunicação série (baudrate, bits de dados, stop bits, etc.)
            result = stim_.Setup_serial();

            // Se ocorrer erro na configuração, tenta a porta seguinte
            if (result != 0)
            {
                continue;
            }

            // Comunicação estabelecida com sucesso
            connected_ = true;
            return true;
        }

        // Não foi possível estabelecer comunicação com nenhum dispositivo
        connected_ = false;
        return false;
    }

    bool MotionStim8Driver::initialize(const std::vector<double>& channels, double main_time, double group_time, double n_factor){

        double low_freq[1] = {0};

        unsigned int num_channels = static_cast<unsigned int>(channels.size());

        if (!connected_)
        {
            return false;
        }

        double frequency = 1000.0 / (1.0 + 0.5 * main_time);

        if (frequency > 500.0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Driver"),"Invalid stimulation frequency (%.1f Hz). Maximum allowed is 500 Hz.",frequency);

            return false;
        }

        if (group_time > main_time)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Driver"),"Group time cannot be greater than main time.");
            
            return false;
        }

        if (group_time < num_channels * 1.5)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Driver"),"Group time is too small for the selected number of channels.");

            return false;
        }

        int result = stim_.Send_Init_Param(
            num_channels,
            const_cast<double*>(channels.data()),
            0, //n_lf
            low_freq,
            main_time,
            group_time,
            n_factor);

        if (result != 0)
        {
            disconnect();
            return false;
        }

        return true;
    }

    bool MotionStim8Driver::sendUpdate(const std::vector<double>& pulse_width,const std::vector<double>& pulse_current,const std::vector<double>& mode)
    {
        unsigned int num_channels =static_cast<unsigned int>(pulse_width.size());

        if (!connected_)
        {
            return false;
        }

        // Verificação de segurança
        for (size_t i = 0; i < num_channels; ++i)
        {
            if (pulse_width[i] < 0.0 || pulse_width[i] > 500.0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Driver"),"Invalid stimulation parameters. Pulse width must be in [0,500] us.");
                return false;
            }

            if (pulse_current[i] < 0.0 || pulse_current[i] > 127.0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MotionStim8Driver"),"Invalid stimulation parameters. Pulse current in [0,127] mA.");
                return false;
            }
        }

        int result = stim_.Send_Update_Parameter(
            const_cast<double*>(pulse_width.data()),
            const_cast<double*>(pulse_current.data()),
            const_cast<double*>(mode.data()),
            num_channels);

        if (result != 0)
        {
            disconnect();
            return false;
        }

        return true;
    }

    bool MotionStim8Driver::isConnected() const
    {
        return connected_;
    }

    void MotionStim8Driver::disconnect()
    {
        if (connected_)
        {
            stim_.Send_Stop_Signal();
            stim_.Close_serial();
        }

        connected_ = false;
    }


}