/**
 * Created: Jan 22, 2026
 * Author(s): Aly Ashour
 */

#ifndef AP1_SIM_ENTITY
#define AP1_SIM_ENTITY

namespace ap1::sim {
    class Entity {
    public:
        float x, y, z; // position
        float gamma; // rotation (along +z axis)
    };
} // namespace ap1::sim

#endif // AP1_SIM_ENTITY
