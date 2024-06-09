/* aimod.h */

#ifndef AIMOD_H
#define AIMOD_H

#include "core/object/ref_counted.h"
#include "servers/physics_server_3d.h"
#include "scene/3d/node_3d.h"
#include "core/object/worker_thread_pool.h"


template <typename T>
class TypedArray;

using namespace godot;


class VisionQueryParameters : public RefCounted {
	GDCLASS(VisionQueryParameters, RefCounted);

	Node* query_owner = nullptr;
	Vector3 owner_position;
	Vector3 eye_position;
	Vector3 eye_forward;
	float range;
	float vision_angle_horizontal;
	float vision_angle_vertical;
	float vision_angle_backwards_offset;
	float vision_height_high;
	float vision_height_low;
	int max_collisions = 32;
	uint32_t detection_collision_mask = (1 << 1) | (1 << 4) | (1 << 8);
	uint32_t occlusion_collision_mask = (1) | (1 << 6);

protected:
	static void _bind_methods();

public:

	Node *get_query_owner() const { return query_owner; };
	void set_query_owner(Node *_query_owner) { query_owner = _query_owner; };

	Vector3 get_owner_position() const { return owner_position; };
	void set_owner_position(const Vector3 &_owner_position) { owner_position = _owner_position; };

	Vector3 get_eye_position() const { return eye_position; };
	void set_eye_position(const Vector3& _eye_position) { eye_position = _eye_position;	};

	Vector3 get_eye_forward() const { return eye_forward; };
	void set_eye_forward(const Vector3 &_eye_forward) { eye_forward = _eye_forward; };

	float get_range() const { return range; };
	void set_range(float _range) { range = _range; };

	float get_vision_angle_horizontal() const { return vision_angle_horizontal; };
	void set_vision_angle_horizontal(float _vision_angle_horizontal) { vision_angle_horizontal = _vision_angle_horizontal; };
	float get_vision_angle_vertical() const { return vision_angle_vertical; };
	void set_vision_angle_vertical(float _vision_angle_vertical) { vision_angle_vertical = _vision_angle_vertical; };
	float get_vision_angle_backwards_offset() const { return vision_angle_backwards_offset; };
	void set_vision_angle_backwards_offset(float _vision_angle_backwards_offset) { vision_angle_backwards_offset = _vision_angle_backwards_offset; };

	float get_vision_height_high() const { return vision_height_high; };
	void set_vision_height_high(float _vision_height_high) { vision_height_high = _vision_height_high; };
	float get_vision_height_low() const { return vision_height_low; };
	void set_vision_height_low(float _vision_height_low) { vision_height_low = _vision_height_low; };

	int get_max_collisions() const { return max_collisions; };
	void set_max_collisions(float _max_collisions) { max_collisions = _max_collisions; };

	uint32_t get_detection_collision_mask() const { return detection_collision_mask; };
	void set_detection_collision_mask(uint32_t _detection_collision_mask) { detection_collision_mask = _detection_collision_mask; };

	uint32_t get_occlusion_collision_mask() const { return occlusion_collision_mask; };
	void set_occlusion_collision_mask(uint32_t _occlusion_collision_mask) { occlusion_collision_mask = _occlusion_collision_mask; };

};


class VisibilitySystem : public Object {
	GDCLASS(VisibilitySystem, Object);

	static VisibilitySystem *singleton;

protected:
	static void _bind_methods();

public:

	VisibilitySystem();
	_FORCE_INLINE_ static VisibilitySystem *get_singleton() { return singleton; };
	TypedArray<Node> calculate_vision(const Ref<VisionQueryParameters> &p_query);
	static bool is_within_vision_angle(const Vector3 &point, const Ref<VisionQueryParameters> &p_query);

};

#endif // AIMOD_H
