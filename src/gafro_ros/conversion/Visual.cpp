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

#include <gafro_robot_descriptions/serialization/FilePath.hpp>
#include <gafro_ros/conversion/Visual.hpp>

namespace gafro_ros
{

    visualization_msgs::Marker convertToMarker(const gafro::Visual *visual)
    {
        visualization_msgs::Marker marker;

        switch (visual->getType())
        {
        case gafro::Visual::Type::SPHERE: {
            const gafro::visual::Sphere *sphere = static_cast<const gafro::visual::Sphere *>(visual);

            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = sphere->getRadius();
            marker.scale.y = sphere->getRadius();
            marker.scale.z = sphere->getRadius();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        case gafro::Visual::Type::MESH: {
            const gafro::visual::Mesh *mesh = static_cast<const gafro::visual::Mesh *>(visual);

            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "file://" + mesh->getFilename();
            marker.scale.x = mesh->getScaleX();
            marker.scale.y = mesh->getScaleY();
            marker.scale.z = mesh->getScaleZ();
            marker.mesh_use_embedded_materials = true;

            break;
        }
        case gafro::Visual::Type::CYLINDER: {
            const gafro::visual::Cylinder *cylinder = static_cast<const gafro::visual::Cylinder *>(visual);

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = 2.0 * cylinder->getRadius();
            marker.scale.y = 2.0 * cylinder->getRadius();
            marker.scale.z = cylinder->getLength();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        case gafro::Visual::Type::BOX: {
            const gafro::visual::Box *box = static_cast<const gafro::visual::Box *>(visual);

            marker.type = visualization_msgs::Marker::CUBE;

            marker.scale.x = box->getDimX();
            marker.scale.y = box->getDimY();
            marker.scale.z = box->getDimZ();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        }

        return marker;
    }

}  // namespace gafro_ros