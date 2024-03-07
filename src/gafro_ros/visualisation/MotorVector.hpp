/*
    Copyright (c) Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro_ros.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <gafro/algebra/Motor.hpp>
#include <sackmesser_ros/Publisher.hpp>
#include <tuple>
#include <variant>

namespace gafro_ros::visualisation
{
    class MotorVector : public sackmesser_ros::Publisher<visualization_msgs::MarkerArray, std::vector<gafro::Motor<double>>>
    {
      public:
        MotorVector(const std::string &name, sackmesser_ros::Interface *interface);

        ~MotorVector();

        visualization_msgs::MarkerArray createMessage(const std::vector<gafro::Motor<double>> &motors) const;

      protected:
      private:
        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            double scale = 0.0;
            double opacity = 0.0;
            std::string frame;
        };
        Configuration config_;
    };

}  // namespace gafro_ros::visualisation