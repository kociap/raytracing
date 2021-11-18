#include <kd_tree.hpp>

#include <anton/algorithm.hpp>
#include <anton/algorithm/sort.hpp>

namespace raytracing {
    void KD_Tree::Node::initialize_leaf(i64 _primitives, i64 _primitives_indices_offset) {
        primitives = _primitives;
        primitives_indices_offset = _primitives_indices_offset;
        flags = 3;
    }

    void KD_Tree::Node::initialize_interior(i32 _axis, f32 _split_position, i64 _second_child_index) {
        flags = _axis;
        second_child_index = _second_child_index;
        split_position = _split_position;
    }

    bool KD_Tree::Node::is_leaf() const {
        return flags == 3;
    }

    i32 KD_Tree::Node::axis() const {
        return flags;
    }

    [[nodiscard]] static i64 calculate_tree_max_depth(i64 const triangles) {
        // 8 + 1.3 * log2(triangles)
        return 8 + (13 * math::ilog2((u64)triangles)) / 10;
    }

    [[nodiscard]] static Extent3 calculate_triangle_bounds(Triangle const& triangle) {
        return {min(min(triangle.v1, triangle.v2), triangle.v3), max(max(triangle.v1, triangle.v2), triangle.v3)};
    }

    [[nodiscard]] static f32 calculate_surface_area(Extent3 const& extent) {
        Vec3 const diagonal = extent.max - extent.min;
        return 2.0f * (diagonal.x * diagonal.y + diagonal.x * diagonal.z + diagonal.y * diagonal.z);
    }

    [[nodiscard]] static i32 find_maximum_extent_axis(Extent3 const& extent) {
        Vec3 const diagonal = extent.max - extent.min;
        if(diagonal.x > diagonal.y && diagonal.x > diagonal.z) {
            return 0;
        } else if(diagonal.y > diagonal.z) {
            return 1;
        } else {
            return 2;
        }
    }

    void KD_Tree::construct_node(Construct_Parameters const& p) {
        i64 const node_index = nodes.size();
        nodes.push_back(Node{});
        if(p.primitives <= p.max_primitives || p.depth == 0) {
            i64 const offset = primitive_indices.size();
            nodes[node_index].initialize_leaf(p.primitives, offset);
            for(i64 const index: p.primitive_indices) {
                primitive_indices.push_back(index);
            }
            return;
        }

        f32 best_cost = math::infinity;
        i64 best_offset = -1;
        i32 best_axis = -1;
        f32 const inv_area = 1.0 / calculate_surface_area(p.bounds);
        i32 axis = find_maximum_extent_axis(p.bounds);
        Vec3 const bounds_diagonal = p.bounds.max - p.bounds.min;
        for(i32 retries = 0; best_axis == -1 && retries < 3; retries += 1) {
            // Compute edges for all bounding volumes in this node.
            for(i64 i = 0; i < p.primitives; ++i) {
                i64 const primitive_index = p.primitive_indices[i];
                Extent3 const bounds = primitive_bv[primitive_index];
                p.edges[axis][2 * i] = Edge{primitive_index, bounds.min[axis], true};
                p.edges[axis][2 * i + 1] = Edge{primitive_index, bounds.max[axis], false};
            }

            quick_sort(p.edges[axis].data(), p.edges[axis].data() + 2 * p.primitives, [](Edge const& lhs, Edge const& rhs) {
                // The edges are only equal when positions are equal and their types are equal.
                // Otherwise sort by position with secondary sorting on min. min edges come first.
                return lhs.position < rhs.position || lhs.min > rhs.min;
            });

            // Compute all splits for the current axis.
            i64 below = 0;
            i64 above = p.primitives;
            for(i64 i = 0; i < 2 * p.primitives; ++i) {
                Edge const& edge = p.edges[axis][i];
                if(!edge.min) {
                    above -= 1;
                }

                f32 const split_position = edge.position;
                if(split_position > p.bounds.min[axis] && split_position < p.bounds.max[axis]) {
                    // Calculate the cost of this split.
                    i32 const other_axis0 = (axis + 1) % 3;
                    i32 const other_axis1 = (axis + 2) % 2;
                    f32 const below_area = 2.0 * (bounds_diagonal[other_axis0] * bounds_diagonal[other_axis1] +
                                                  (split_position - p.bounds.min[axis]) * (bounds_diagonal[other_axis0] + bounds_diagonal[other_axis1]));
                    f32 const above_area = 2.0 * (bounds_diagonal[other_axis0] * bounds_diagonal[other_axis1] +
                                                  (p.bounds.max[axis] - split_position) * (bounds_diagonal[other_axis0] + bounds_diagonal[other_axis1]));
                    f32 const probability_below = below_area * inv_area;
                    f32 const probability_above = above_area * inv_area;
                    f32 const empty_bonus = (above == 0 || below == 0) ? p.empty_bonus : 0.0f;
                    f32 const cost = p.traverse_cost + p.intersect_cost * (1.0f - empty_bonus) * (probability_below * below + probability_above * above);
                    if(cost < best_cost) {
                        best_cost = cost;
                        best_axis = axis;
                        best_offset = i;
                    }
                }

                if(edge.min) {
                    below += 1;
                }
            }

            axis = (axis + 1) % 3;
        }

        f32 const old_cost = p.intersect_cost * p.primitives;
        i32 bad_refines = p.bad_refines;
        if(best_cost > old_cost) {
            bad_refines += 1;
        }

        if(((best_cost > 4 * old_cost) && p.primitives < 16) || best_axis == -1 || bad_refines == 3) {
            i64 const offset = primitive_indices.size();
            nodes[node_index].initialize_leaf(p.primitives, offset);
            for(i64 const index: p.primitive_indices) {
                primitive_indices.push_back(index);
            }
            return;
        }

        i64 primitives_below = 0;
        i64* const primitives_below_data = p.primitive_indices_reusable;
        for(i64 i = 0; i < best_offset; ++i) {
            Edge const& edge = p.edges[best_axis][i];
            if(edge.min) {
                primitives_below_data[primitives_below] = edge.primitive_index;
                ++primitives_below;
            }
        }

        i64 primitives_above = 0;
        i64* const primitives_above_data = p.primitive_indices_nonreusable;
        for(i64 i = best_offset + 1; i < 2 * p.primitives; ++i) {
            Edge const& edge = p.edges[best_axis][i];
            if(!edge.min) {
                primitives_above_data[primitives_above] = edge.primitive_index;
                ++primitives_above;
            }
        }

        f32 const split_position = p.edges[best_axis][best_offset].position;
        // Construct the 'below' node.
        Construct_Parameters p0 = p;
        p0.bounds.max[best_axis] = split_position;
        p0.depth -= 1;
        p0.bad_refines = bad_refines;
        p0.primitives = primitives_below;
        p0.primitive_indices = Slice{primitives_below_data, primitives_below_data + primitives_below};
        p0.primitive_indices_nonreusable = primitives_above_data + primitives_above;
        construct_node(p0);
        // Initialize our current node as interior.
        i64 const second_child_index = nodes.size();
        nodes[node_index].initialize_interior(best_axis, split_position, second_child_index);
        // Construct the 'above' node.
        Construct_Parameters p1 = p;
        p1.bounds.min[best_axis] = split_position;
        p1.depth -= 1;
        p1.bad_refines = bad_refines;
        p1.primitives = primitives_above;
        p1.primitive_indices = Slice{primitives_above_data, primitives_above_data + primitives_above};
        p1.primitive_indices_nonreusable = primitives_above_data + primitives_above;
        construct_node(p1);
    }

    void KD_Tree::build(Scene const& scene, Build_Options const& options) {
        i64 const primitives = scene.triangles.size();
        primitive_bv.ensure_capacity(primitives);
        for(Triangle const& triangle: scene.triangles) {
            Extent3 const triangle_bounds = calculate_triangle_bounds(triangle);
            primitive_bv.push_back(triangle_bounds);
            root_bounds = math::outer_extent(root_bounds, triangle_bounds);
        }

        i64 const max_depth = (options.max_depth == 0 ? calculate_tree_max_depth(primitives) : options.max_depth);
        node_queue.ensure_capacity(2 * max_depth);

        i64 const edge_count = 2 * primitives;
        // Allocate working memory for all 3 axes for edges (2 * the number of primitives).
        Array<Edge> edges{reserve, 3 * edge_count};
        Slice<Edge> edges_slices[3] = {{edges.data(), edge_count}, {edges.data() + edge_count, edge_count}, {edges.data() + 2 * edge_count, edge_count}};
        // Storage for double the number of primitives due to up to primitives overlaps with both children.
        Array<i64> primitive_indices{reserve, (max_depth + 2) * primitives};
        primitive_indices.force_size(primitives);
        fill_with_consecutive(primitive_indices.begin(), primitive_indices.end(), 0);
        Construct_Parameters parameters;
        parameters.edges = Slice{edges_slices};
        parameters.primitive_indices = primitive_indices;
        parameters.primitive_indices_reusable = primitive_indices.data();
        parameters.primitive_indices_nonreusable = primitive_indices.data() + primitives;
        parameters.bounds = root_bounds;
        parameters.depth = max_depth;
        parameters.max_primitives = options.max_primitives;
        parameters.primitives = primitives;
        parameters.intersect_cost = options.intersect_cost;
        parameters.traverse_cost = options.traverse_cost;
        parameters.bad_refines = 0;
        parameters.empty_bonus = options.empty_bonus;
        construct_node(parameters);
    }

    struct Min_Max_Distance {
        f32 min;
        f32 max;
    };

    [[nodiscard]] static Optional<Min_Max_Distance> intersect_extent(Vec3 const ray_origin, Vec3 const inv_ray_direction, Extent3 const extent) {
        // AABB slab test
        f32 tmin = -math::infinity;
        f32 tmax = math::infinity;
        for(i32 i = 0; i < 3; ++i) {
            f32 tx1 = (extent.min[i] - ray_origin[i]) * inv_ray_direction[i];
            f32 tx2 = (extent.max[i] - ray_origin[i]) * inv_ray_direction[i];
            tmin = math::max(tmin, math::min(tx1, tx2));
            tmax = math::min(tmax, math::max(tx1, tx2));
        }

        if(tmax >= 0 && tmax >= tmin) {
            return Min_Max_Distance{tmin, tmax};
        } else {
            return null_optional;
        }
    }

    Pair<KD_Tree::Node*, KD_Tree::Node*> KD_Tree::order_child_nodes(Node* const node, Ray const ray) {
        f32 const split_position = node->split_position;
        i32 const axis = node->axis();
        bool const below_first = (ray.origin[axis] < split_position) || (ray.origin[axis] == split_position && ray.direction[axis] <= 0.0f);
        if(below_first) {
            return {node + 1, &nodes[node->second_child_index]};
        } else {
            return {&nodes[node->second_child_index], node + 1};
        }
    }

    Optional<Surface_Interaction> KD_Tree::intersect(Scene const& scene, Ray const ray) {
        Vec3 const inv_ray_direction = Vec3{1.0f} / ray.direction;
        Optional<Min_Max_Distance> bounds_result = intersect_extent(ray.origin, inv_ray_direction, root_bounds);
        if(!bounds_result) {
            return null_optional;
        }

        bool hit = false;
        f32 minimal_max = math::infinity;
        Surface_Interaction result;
        node_queue.push_back(Search_Node{&nodes[0], bounds_result->min, bounds_result->max});
        while(node_queue.size() > 0) {
            auto [node, min, max] = node_queue.back();
            if(min > minimal_max) {
                break;
            }

            node_queue.pop_back();
            if(!node->is_leaf()) {
                auto [first, second] = order_child_nodes(node, ray);
                i32 const axis = node->axis();
                f32 const split = (node->split_position - ray.origin[axis]) * inv_ray_direction[axis];
                if(split > max || split <= 0) {
                    node_queue.push_back(Search_Node{first, min, max});
                } else if(split < min) {
                    node_queue.push_back(Search_Node{second, min, max});
                } else {
                    node_queue.push_back(Search_Node{second, split, max});
                    node_queue.push_back(Search_Node{first, min, split});
                }
            } else {
                // Intersect the primitives inside the leaf node.
                i64 const primitives = node->primitives;
                i64 const* const indices = primitive_indices.data() + node->primitives_indices_offset;
                for(i64 i = 0; i < primitives; ++i) {
                    i64 const index = indices[i];
                    Triangle const& triangle = scene.triangles[index];
                    Optional<Surface_Interaction> intersection_result = intersect_triangle(ray, triangle);
                    if(intersection_result && intersection_result->distance < result.distance) {
                        result = intersection_result.value();
                        hit = true;
                        minimal_max = math::min(minimal_max, max);
                    }
                }
            }
        }

        if(hit) {
            return result;
        } else {
            return null_optional;
        }
    }
} // namespace raytracing
