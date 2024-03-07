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

#include <Eigen/Geometry>
//
#include <gafro/algebra.hpp>
#include <gafro/physics/Twist.hxx>
#include <gafro_ros/visualisation/Twist.hpp>

namespace gafro_ros::visualisation
{
    Twist::Twist(const std::string &name, sackmesser_ros::Interface *interface)  //
      : sackmesser_ros::Publisher<geometry_msgs::TwistStamped, gafro::Twist<double>>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Twist::~Twist() {}

    bool Twist::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame);
    }

    geometry_msgs::TwistStamped Twist::createMessage(const gafro::Twist<double> &twist) const
    {
        geometry_msgs::TwistStamped wrench_msg;

        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = config_.frame;

        wrench_msg.twist.linear.x = twist.multivector().get<gafro::blades::e1i>();
        wrench_msg.twist.linear.y = twist.multivector().get<gafro::blades::e2i>();
        wrench_msg.twist.linear.z = twist.multivector().get<gafro::blades::e3i>();

        wrench_msg.twist.angular.x = twist.multivector().get<gafro::blades::e23>();
        wrench_msg.twist.angular.y = -twist.multivector().get<gafro::blades::e13>();
        wrench_msg.twist.angular.z = twist.multivector().get<gafro::blades::e12>();

        return wrench_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Twist, "gafro_twist");

}  // namespace gafro_ros::visualisation