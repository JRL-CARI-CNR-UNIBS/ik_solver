#include <iomanip>

#include <ik_solver_core/types.h>


namespace ik_solver
{

std::ostream& operator<<(std::ostream& stream, const Range& r) 
{
    stream << std::fixed << std::setprecision(3) << "[" << r.min() << " " << r.max()<<"]";
    return stream;
 }


std::ostream& operator<<(std::ostream& stream, const JointBoundaries& rr) 
{
  stream << "lb: " << rr.lb() << ", ub: " << rr.ub() << ", ranges: [";
  for(size_t i=0; i<rr.size(); i++)
  {
    stream << std::fixed << std::setprecision(3) << rr.at(i) << (i == rr.size()-1 ? "" : ", ");
  }
  stream << "]";
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const NamedJointBoundaries& rr) 
{
  stream << "name: " << rr.first << ", lb: " << rr.second.lb() << ", ub: " << rr.second.ub() << ", ranges: [";
  for(size_t i=0; i<rr.second.size(); i++)
  {
    stream << std::fixed << std::setprecision(3) << rr.second.at(i) << (i == rr.second.size()-1 ? "" : ", ");
  }
  stream << "]";
  return stream;
}


std::ostream& operator<<(std::ostream& stream, const JointsBoundaries& rr) 
{
  stream << "lb: " << rr.lb().transpose() << ", ub: " << rr.ub().transpose() << " joints: {\n";
  for(size_t i =0; i<rr.size(); i++)
  {
    stream << std::fixed << std::setprecision(3) << rr.at(i).first +": { ax_index: " <<  i << ", boundaries: " << rr.at(i).second << "},\n";
  }
  stream << "}";
  return stream;
}
}  // namespace ik_solver
