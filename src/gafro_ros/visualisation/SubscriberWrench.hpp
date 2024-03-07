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

#include <geometry_msgs/WrenchStamped.h>
//
#include <gafro/physics/Wrench.hxx>
//
#include <sackmesser_ros/Subscriber.hpp>

namespace gafro_ros
{
    class SubscriberWrench : public sackmesser_ros::Subscriber<geometry_msgs::WrenchStamped, gafro::Wrench<double>>
    {
      public:
        SubscriberWrench(const std::string &name, sackmesser_ros::Interface *interface);

      private:
        gafro::Wrench<double> convert(const geometry_msgs::WrenchStamped::ConstPtr &message) const;
    };
}  // namespace gafro_ros