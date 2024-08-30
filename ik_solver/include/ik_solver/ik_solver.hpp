#ifndef IK_SOLVER__IK_SOLVER
#define IK_SOLVER__IK_SOLVER

#include "ik_solver_core/ik_solver_base_class.h"

namespace ik_solver
{
class IkSolver : public IkSolverBase
{
public:
  IkSolver() : IkSolverBase() {}
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;
};

} // namespace ik_solver

#endif // IK_SOLVER__IK_SOLVER
