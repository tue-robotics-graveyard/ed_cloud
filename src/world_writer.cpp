#include <ed/world_model.h>
#include <ed/entity.h>
#include "world_writer.h"


void ed_cloud::world_write(const ed::WorldModel &world, int rev_number, std::ostream &output)
{
    output << "{";
    output << "\"revision\":\"" << rev_number << "\",";
    output << "[";


    for (std::vector<ed::EntityConstPtr>::const_iterator it = world.entities().begin();
         it != world.entities().end(); it ++) {

        output << "{";
        output << "\"id\":\"" << it->id() << "\"" << ",";
        output << "\"type\":\"" << it->type() << "\"" << ",";
        output << "}";
    }

    output << "]";
}
