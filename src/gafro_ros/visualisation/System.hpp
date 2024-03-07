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

#include <Eigen/Geometry>
#include <gafro/algebra/Point.hpp>
#include <gafro/algebra/PointPair.hpp>
#include <gafro/robot.hpp>
#include <gafro_ros/conversion/Point.hpp>
#include <sackmesser_ros/Publisher.hpp>

namespace gafro_ros::visualisation
{
    class System : public sackmesser_ros::Publisher<visualization_msgs::MarkerArray, gafro::System<double>, Eigen::MatrixXd, gafro::Motor<double>>
    {
      public:
        System(const std::string &name, sackmesser_ros::Interface *interface);

        ~System();

        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            std::string frame;

            double color_r = 1.0;
            double color_g = 1.0;
            double color_b = 1.0;
            double color_a = 1.0;
        };

        visualization_msgs::MarkerArray createMessage(const gafro::System<double> &system, const Eigen::MatrixXd &position,
                                                      const gafro::Motor<double> &base = gafro::Motor<double>()) const;

      protected:
      private:
        Configuration config_;
    };

}  // namespace gafro_ros::visualisation