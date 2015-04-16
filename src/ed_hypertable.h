#ifndef ED_HYPERTABLE_H
#define ED_HYPERTABLE_H

#include <string>

namespace ed_hypertable {

    const std::string TABLE_NAME = "entity_delta";

    const std::string DELETED_CELL = "deleted";
    const std::string CONVEX_HULL_CELL = "hull_convex";
    const std::string SHAPE_CELL = "shape";
    const std::string TYPE_CELL = "type";
    const std::string DATA_CELL = "entity_data";
    const std::string POSE_CELL = "pose";

    const int MAX_VERSIONS = 1;

}

#endif // ED_HYPERTABLE_H
