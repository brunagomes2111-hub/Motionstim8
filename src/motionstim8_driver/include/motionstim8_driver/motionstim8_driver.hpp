#pragma once 

#include <string>
#include <vector>

#include "motionstim8.h"


namespace motionstim8_driver{
    class MotionStim8Driver{

        public:

            MotionStim8Driver();
            bool connect();
            bool initialize(const std::vector<double>& channels, double main_time, double group_time, double n_factor);
            bool sendUpdate(const std::vector<double>& pulse_width, const std::vector<double>& pa, const std::vector<double>& mode);
            void disconnect();
            bool isConnected() const;

        private:

            // Biblioteca fornecida pelo fabricante do MotionStim8
            WRF::motionstim8 stim_;

            bool connected_{false};
    };
}