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

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
//
#include <gafro_ros/visualisation/Ellipse.hpp>

namespace gafro_ros::visualisation
{
    Ellipse::Ellipse(const std::string &name, sackmesser_ros::Interface *interface)
      : sackmesser_ros::Publisher<visualization_msgs::Marker, Eigen::Vector3d, Eigen::Matrix3d>(name, interface)
    {
        config_ = interface->getConfigurations()->load<Configuration>("publisher/" + name + "/");
    }

    Ellipse::~Ellipse() {}

    bool Ellipse::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "scale", &scale, true) &&      //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::Marker Ellipse::createMessage(const Eigen::Vector3d &center, const Eigen::Matrix3d &ellipse) const
    {
        visualization_msgs::Marker ellipse_msg;

        ellipse_msg.header.frame_id = config_.frame;
        ellipse_msg.header.stamp = ros::Time::now();

        ellipse_msg.pose.position.x = center.x();
        ellipse_msg.pose.position.y = center.y();
        ellipse_msg.pose.position.z = center.z();

        Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(ellipse);

        Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues().real();
        Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors().real();

        Eigen::Quaterniond q(eigenvectors);

        ellipse_msg.pose.orientation.x = q.x();
        ellipse_msg.pose.orientation.y = q.y();
        ellipse_msg.pose.orientation.z = q.z();
        ellipse_msg.pose.orientation.w = q.w();

        ellipse_msg.type = visualization_msgs::Marker::SPHERE;
        ellipse_msg.scale.x = config_.scale * std::sqrt(eigenvalues.x());
        ellipse_msg.scale.y = config_.scale * std::sqrt(eigenvalues.y());
        ellipse_msg.scale.z = config_.scale * std::sqrt(eigenvalues.z());
        ellipse_msg.color.r = static_cast<float>(config_.color_r);
        ellipse_msg.color.g = static_cast<float>(config_.color_g);
        ellipse_msg.color.b = static_cast<float>(config_.color_b);
        ellipse_msg.color.a = static_cast<float>(config_.color_a);

        return ellipse_msg;
    }

    REGISTER_CLASS(sackmesser_ros::PublisherFactory, Ellipse, "gafro_ellipse");

}  // namespace gafro_ros::visualisation