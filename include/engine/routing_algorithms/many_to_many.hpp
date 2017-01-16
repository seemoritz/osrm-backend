#ifndef MANY_TO_MANY_ROUTING_HPP
#define MANY_TO_MANY_ROUTING_HPP

#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"
#include "util/typedefs.hpp"

#include "tbb/enumerable_thread_specific.h"
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

template <class DataFacadeT>
class ManyToManyRouting final
    : public BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>
{
    using super = BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>;
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;

    struct NodeBucket
    {
        unsigned target_id; // essentially a row in the weight matrix
        EdgeWeight weight;
        NodeBucket(const unsigned target_id, const EdgeWeight weight)
            : target_id(target_id), weight(weight)
        {
        }
    };

    // FIXME This should be replaced by an std::unordered_multimap, though this needs benchmarking
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;

  public:
    ManyToManyRouting(SearchEngineData &engine_working_data)
        : engine_working_data(engine_working_data)
    {
    }

    std::vector<EdgeWeight> operator()(const DataFacadeT &facade,
                                       const std::vector<PhantomNode> &phantom_nodes,
                                       const std::vector<std::size_t> &source_indices,
                                       const std::vector<std::size_t> &target_indices) const
    {
        const auto number_of_sources =
            source_indices.empty() ? phantom_nodes.size() : source_indices.size();
        const auto number_of_targets =
            target_indices.empty() ? phantom_nodes.size() : target_indices.size();
        const auto number_of_entries = number_of_sources * number_of_targets;
        std::vector<EdgeWeight> result_table(number_of_entries,
                                             std::numeric_limits<EdgeWeight>::max());

        using ThreadedQueryHeap = tbb::enumerable_thread_specific<std::unique_ptr<QueryHeap>>;
        ThreadedQueryHeap heaps;

        using ThreadedBuckets =
            tbb::enumerable_thread_specific<std::unique_ptr<SearchSpaceWithBuckets>>;
        ThreadedBuckets buckets;

        const auto search_target_phantom = [this, &heaps, &buckets, &facade](
            const PhantomNode &phantom, unsigned column_idx) {
            auto &heap_pointer = heaps.local();
            if (heap_pointer == nullptr)
            {
                heap_pointer.reset(new QueryHeap(facade.GetNumberOfNodes()));
            }
            auto &query_heap = *heap_pointer;

            auto &bucket_pointer = buckets.local();
            if (bucket_pointer == nullptr)
            {
                bucket_pointer.reset(new SearchSpaceWithBuckets);
            }
            auto &search_space_with_buckets = *bucket_pointer;

            query_heap.Clear();
            // insert target(s) at weight 0

            if (phantom.forward_segment_id.enabled)
            {
                query_heap.Insert(phantom.forward_segment_id.id,
                                  phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_segment_id.id);
            }
            if (phantom.reverse_segment_id.enabled)
            {
                query_heap.Insert(phantom.reverse_segment_id.id,
                                  phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_segment_id.id);
            }

            // explore search space
            while (!query_heap.Empty())
            {
                BackwardRoutingStep(facade, column_idx, query_heap, search_space_with_buckets);
            }
        };

        SearchSpaceWithBuckets search_space_with_buckets;

        // for each source do forward search
        const auto search_source_phantom = [this,
                                            &heaps,
                                            &facade,
                                            &search_space_with_buckets,
                                            &result_table,
                                            number_of_targets](const PhantomNode &phantom,
                                                               unsigned row_idx) {
            auto &heap_pointer = heaps.local();
            if (heap_pointer == nullptr)
            {
                heap_pointer.reset(new QueryHeap(facade.GetNumberOfNodes()));
            }
            auto &query_heap = *heap_pointer;
            query_heap.Clear();
            // insert target(s) at weight 0

            if (phantom.forward_segment_id.enabled)
            {
                query_heap.Insert(phantom.forward_segment_id.id,
                                  -phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_segment_id.id);
            }
            if (phantom.reverse_segment_id.enabled)
            {
                query_heap.Insert(phantom.reverse_segment_id.id,
                                  -phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_segment_id.id);
            }

            // explore search space
            while (!query_heap.Empty())
            {
                ForwardRoutingStep(facade,
                                   row_idx,
                                   number_of_targets,
                                   query_heap,
                                   search_space_with_buckets,
                                   result_table);
            }
        };

        if (target_indices.empty())
        {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, phantom_nodes.size()),
                              [&search_target_phantom, &phantom_nodes](const tbb::blocked_range<std::size_t> &rg) {
                                  for (auto column = rg.begin(); column < rg.end(); column++)
                                  {
                                      search_target_phantom(phantom_nodes[column], column);
                                  }
                              });
        }
        else
        {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, target_indices.size()),
                              [&search_target_phantom, &phantom_nodes, &target_indices](
                                  const tbb::blocked_range<std::size_t> &rg) {
                                  for (auto column = rg.begin(); column < rg.end(); column++)
                                  {
                                      const auto &phantom = phantom_nodes[target_indices[column]];
                                      search_target_phantom(phantom, column);
                                  }
                              });
        }

        for (const auto &bucket : buckets)
        {
            for (const auto &id_and_bucket : *bucket)
            {
                auto &resulting_bucket = search_space_with_buckets[id_and_bucket.first];
                resulting_bucket.insert(resulting_bucket.end(), id_and_bucket.second.begin(), id_and_bucket.second.end());
            }
        }
        buckets.clear();

        if (source_indices.empty())
        {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, phantom_nodes.size()),
                              [&search_source_phantom, &phantom_nodes](const tbb::blocked_range<std::size_t> &rg) {
                                  for (auto row = rg.begin(); row < rg.end(); row++)
                                  {
                                      search_source_phantom(phantom_nodes[row], row);
                                  }
                              });
        }
        else
        {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, source_indices.size()),
                              [&search_source_phantom, &phantom_nodes, &source_indices](
                                  const tbb::blocked_range<std::size_t> &rg) {
                                  for (auto row = rg.begin(); row < rg.end(); row++)
                                  {
                                      const auto &phantom = phantom_nodes[source_indices[row]];
                                      search_source_phantom(phantom, row);
                                  }
                              });
        }

        return result_table;
    }

    void ForwardRoutingStep(const DataFacadeT &facade,
                            const unsigned row_idx,
                            const unsigned number_of_targets,
                            QueryHeap &query_heap,
                            const SearchSpaceWithBuckets &search_space_with_buckets,
                            std::vector<EdgeWeight> &result_table) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int source_weight = query_heap.GetKey(node);

        // check if each encountered node has an entry
        const auto bucket_iterator = search_space_with_buckets.find(node);
        // iterate bucket if there exists one
        if (bucket_iterator != search_space_with_buckets.end())
        {
            const std::vector<NodeBucket> &bucket_list = bucket_iterator->second;
            for (const NodeBucket &current_bucket : bucket_list)
            {
                // get target id from bucket entry
                const unsigned column_idx = current_bucket.target_id;
                const int target_weight = current_bucket.weight;
                auto &current_weight = result_table[row_idx * number_of_targets + column_idx];
                // check if new weight is better
                const EdgeWeight new_weight = source_weight + target_weight;
                if (new_weight < 0)
                {
                    const EdgeWeight loop_weight = super::GetLoopWeight(facade, node);
                    const int new_weight_with_loop = new_weight + loop_weight;
                    if (loop_weight != INVALID_EDGE_WEIGHT && new_weight_with_loop >= 0)
                    {
                        current_weight = std::min(current_weight, new_weight_with_loop);
                    }
                }
                else if (new_weight < current_weight)
                {
                    result_table[row_idx * number_of_targets + column_idx] = new_weight;
                }
            }
        }
        if (StallAtNode<true>(facade, node, source_weight, query_heap))
        {
            return;
        }
        RelaxOutgoingEdges<true>(facade, node, source_weight, query_heap);
    }

    void BackwardRoutingStep(const DataFacadeT &facade,
                             const unsigned column_idx,
                             QueryHeap &query_heap,
                             SearchSpaceWithBuckets &search_space_with_buckets) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int target_weight = query_heap.GetKey(node);

        // store settled nodes in search space bucket
        search_space_with_buckets[node].emplace_back(column_idx, target_weight);

        if (StallAtNode<false>(facade, node, target_weight, query_heap))
        {
            return;
        }

        RelaxOutgoingEdges<false>(facade, node, target_weight, query_heap);
    }

    template <bool forward_direction>
    inline void RelaxOutgoingEdges(const DataFacadeT &facade,
                                   const NodeID node,
                                   const EdgeWeight weight,
                                   QueryHeap &query_heap) const
    {
        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const bool direction_flag = (forward_direction ? data.forward : data.backward);
            if (direction_flag)
            {
                const NodeID to = facade.GetTarget(edge);
                const int edge_weight = data.weight;

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const int to_weight = weight + edge_weight;

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_weight, node);
                }
                // Found a shorter Path -> Update weight
                else if (to_weight < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to).parent = node;
                    query_heap.DecreaseKey(to, to_weight);
                }
            }
        }
    }

    // Stalling
    template <bool forward_direction>
    inline bool StallAtNode(const DataFacadeT &facade,
                            const NodeID node,
                            const EdgeWeight weight,
                            QueryHeap &query_heap) const
    {
        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const bool reverse_flag = ((!forward_direction) ? data.forward : data.backward);
            if (reverse_flag)
            {
                const NodeID to = facade.GetTarget(edge);
                const int edge_weight = data.weight;
                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                if (query_heap.WasInserted(to))
                {
                    if (query_heap.GetKey(to) + edge_weight < weight)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
};
}
}
}

#endif
