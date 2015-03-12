#ifndef WORLD_WRITER_H
#define WORLD_WRITER_H

#include "ed/types.h"

namespace ed_cloud {

    void world_write(const ed::WorldModel& world,  int rev_number, std::ostream& output);

}

#endif // WORLD_WRITER_H
