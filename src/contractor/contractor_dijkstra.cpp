#include "contractor/contractor_dijkstra.hpp"

namespace osrm
{
namespace contractor
{

ContractorDijkstra::ContractorDijkstra(const std::size_t heap_size) : Base(heap_size) {}

void ContractorDijkstra::Run(const unsigned number_of_targets,
                             const int node_limit,
                             const int weight_limit,
                             const NodeID forbidden_node,
                             const ContractorGraph &graph)
{
    int nodes = 0;
    unsigned number_of_targets_found = 0;
    while (!Empty())
    {
        const NodeID node = DeleteMin();
        const auto node_weight = GetKey(node);
        if (++nodes > node_limit)
        {
            return;
        }
        if (node_weight > weight_limit)
        {
            return;
        }

        // Destination settled?
        if (GetData(node).target)
        {
            ++number_of_targets_found;
            if (number_of_targets_found >= number_of_targets)
            {
                return;
            }
        }

        RelaxNode(node, node_weight, forbidden_node, graph);
    }
}

void ContractorDijkstra::RelaxNode(const NodeID node,
                                   const int node_weight,
                                   const NodeID forbidden_node,
                                   const ContractorGraph &graph)
{
    const short current_hop = GetData(node).hop + 1;
    for (auto edge : graph.GetAdjacentEdgeRange(node))
    {
        const ContractorEdgeData &data = graph.GetEdgeData(edge);
        if (!data.forward)
        {
            continue;
        }
        const NodeID to = graph.GetTarget(edge);
        if (forbidden_node == to)
        {
            continue;
        }
        const int to_weight = node_weight + data.weight;

        // New Node discovered -> Add to Heap + Node Info Storage
        if (!WasInserted(to))
        {
            Insert(to, to_weight, ContractorHeapData{current_hop, false});
        }
        // Found a shorter Path -> Update weight
        else if (to_weight < GetKey(to))
        {
            DecreaseKey(to, to_weight);
            GetData(to).hop = current_hop;
        }
    }
}

} // namespace contractor
} // namespace osrm
