#pragma once

#include <anton/array.hpp>
#include <anton/math/math.hpp>
#include <anton/optional.hpp>
#include <anton/pair.hpp>
#include <anton/slice.hpp>
#include <build_config.hpp>
#include <intersections.hpp>
#include <scene.hpp>

namespace raytracing {
    struct KD_Tree {
    private:
        // Bounding volumes of the primitives in the scene.
        Array<Extent3> primitive_bv;
        Array<i64> primitive_indices;

        struct Node {
            // initialize_leaf
            //
            void initialize_leaf(i64 primitives, i64 primitices_indices_offset);

            // initialize_interior
            //
            // Parameters:
            //               axis - 0 (x), 1 (y) or 2 (z) depending on which axis the node has been split.
            //     split_position - position of the split along axis in the world space.
            // second_child_index - index of the second child in the nodes array of the KD_Tree.
            //
            void initialize_interior(i32 axis, f32 split_position, i64 second_child_index);

            // is_leaf
            //
            [[nodiscard]] bool is_leaf() const;

            // axis
            //
            [[nodiscard]] i32 axis() const;

            union {
                i64 primitives_indices_offset;
                struct {
                    f32 split_position;
                    i32 _padding;
                };
            };
            u64 flags : 2;
            union {
                u64 primitives : 62;
                // Index of the second child. The index of the first child is <index of the node> + 1.
                u64 second_child_index : 62;
            };
        };

        Array<Node> nodes;

        struct Search_Node {
            Node* node;
            // Parametric minimum along the ray of the intersection with the bounding volume of the node.
            f32 min;
            // Parametric maximum along the ray of the intersection with the bounding volume of the node.
            f32 max;
        };

        Array<Search_Node> node_queue;
        Extent3 root_bounds;

        struct Edge {
            i64 primitive_index;
            // Position of the split on an axis.
            f32 position;
            // Whether the edge is the minimum edge or the maximum edge.
            bool min;
        };

        struct Construct_Parameters {
            // Working memory for finding the best split.
            Slice<Slice<Edge> const> edges;
            Slice<i64> primitive_indices;
            i64* primitive_indices_reusable;
            i64* primitive_indices_nonreusable;
            Extent3 bounds;
            i64 intersect_cost = 0;
            i64 traverse_cost = 0;
            // Max primitives in a node.
            i64 max_primitives = 0;
            i64 primitives = 0;
            i32 depth = 0;
            i32 bad_refines = 0;
            f32 empty_bonus = 0.0f;
        };

        void construct_node(Construct_Parameters const& parameters);
        Pair<Node*, Node*> order_child_nodes(Node* node, Ray ray);

    public:
        struct Build_Options {
            // Maximum depth of the tree. If max_depth is set to 0, the max depth
            // will be calculated based on the number of primitives in the scene.
            i64 max_depth = 0;
            // Maximum number of primitives in a node.
            i64 max_primitives = 1;
            // The cost to intersect a primitive.
            i64 intersect_cost = 80;
            // The cost to traverse the interior node.
            i64 traverse_cost = 1;
            // The bonus for a node being empty.
            // Must be in range [0, 1].
            f32 empty_bonus = 0.5f;
        };

        void build(Scene const& scene, Build_Options const& options);

        [[nodiscard]] Optional<Surface_Interaction> intersect(Scene const& scene, Ray ray);
    };
} // namespace raytracing
