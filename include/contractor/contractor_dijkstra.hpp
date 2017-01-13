#ifndef OSRM_CONTRACTOR_DIJKSTRA_HPP
#define OSRM_CONTRACTOR_DIJKSTRA_HPP

#include "contractor/contractor_graph.hpp"
#include "contractor/contractor_heap.hpp"

#include <cstddef>

namespace osrm
{
namespace contractor
{

// allow access to the heap itself, add Dijkstra functionality on top
class ContractorDijkstra : public ContractorHeap
{
    using Base = ContractorHeap;
  public:
    ContractorDijkstra(std::size_t heap_size);

    // search the graph up
    void Run(const unsigned number_of_targets,
             const int node_limit,
             const int weight_limit,
             const NodeID forbidden_node,
             const ContractorGraph &graph);

  private:
    void RelaxNode(const NodeID node,
                   const int node_weight,
                   const NodeID forbidden_node,
                   const ContractorGraph &graph);
};

} // namespace contractor
} // namespace osrm

#endif // OSRM_CONTRACTOR_DIJKSTRA_HPP
