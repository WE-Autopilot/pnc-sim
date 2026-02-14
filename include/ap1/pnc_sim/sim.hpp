/**
 * Created: Jan 22, 2026
 * Author(s): Aly Ashour
 */

#ifndef AP1_SIM_HPP
#define AP1_SIM_HPP

#include "ap1/pnc_sim/entity.hpp"
#include <vector>

namespace ap1::sim
{
class Sim
{
  public:
    Sim() {}
    void update(const double DT) {}

  private:
    std::vector<Entity> entities;
};
} // namespace ap1::sim

#endif // AP1_SIM_HPP
