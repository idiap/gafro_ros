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

#include <visualization_msgs/Marker.h>

#include <gafro/algebra/Point.hpp>
#include <sackmesser_ros/Publisher.hpp>

namespace gafro_ros::visualisation
{
    class Point : public sackmesser_ros::Publisher<visualization_msgs::Marker, gafro::Point<double>>
    {
      public:
        Point(const std::string &name, sackmesser_ros::Interface *interface);

        ~Point();

        visualization_msgs::Marker createMessage(const gafro::Point<double> &point) const;

      protected:
      private:
        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            std::string frame;
            double radius = 0.0;
            double color_r = 0.0;
            double color_g = 0.0;
            double color_b = 0.0;
            double color_a = 0.0;
        };

        Configuration config_;
    };

}  // namespace gafro_ros::visualisation