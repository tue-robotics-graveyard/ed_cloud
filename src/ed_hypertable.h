#ifndef ED_HYPERTABLE_H
#define ED_HYPERTABLE_H

#include <string>

namespace ed_hypertable {

    const std::string ENTITY_TABLE_NAME = "entity_delta";
    const std::string MEASUREMENT_TABLE_NAME = "entity_delta";
    const std::string DELETED_CELL = "AA_deleted";
    const std::string CONVEX_HULL_CELL = "convex_hull";
    const std::string SHAPE_CELL = "shape";
    const std::string TYPE_CELL = "type";
    const std::string DATA_CELL = "data";
    const std::string POSE_CELL = "pose";
    const std::string MEASUREMENT_CELL = "measurement";

    const int MAX_VERSIONS = 1;

}

#endif // ED_HYPERTABLE_H
