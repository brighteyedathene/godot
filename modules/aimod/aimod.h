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



struct VisibilityData {
	ObjectID object_id;
	Vector3 position;

	VisibilityData(ObjectID _object_id, Vector3 _position) :
			object_id(_object_id),
			position(_position){};

	VisibilityData(){};
};


class VisibilityComponent : public Node3D {
	GDCLASS(VisibilityComponent, Node3D);

	Node3D *owner = nullptr;
	NodePath owner_path;
	bool is_visibility_enabled = true;

protected:
	static void _bind_methods();

public:
	void _ready();
	void _tree_exiting();
	void blabla();

	NodePath get_gameplay_owner_path() const { return owner_path; };
	void set_gameplay_owner_path(NodePath _path) { owner_path = _path; };

	Node3D *get_gameplay_owner() const { return owner; };
	void set_gameplay_owner(Node3D *_owner) { owner = _owner; };

	bool get_is_visibility_enabled() const { return is_visibility_enabled; };
	void set_is_visibility_enabled(bool _enabled) { is_visibility_enabled = _enabled; };

	VisibilityComponent();
};

class EyeComponent : public Node3D {
	GDCLASS(EyeComponent, Node3D);

public:
	TypedArray<Node> visible_nodes;

protected:
	static void _bind_methods();

public:
	TypedArray<Node> get_visible_nodes() { return visible_nodes; };
};


class VisionQueryParameters : public RefCounted {
	GDCLASS(VisionQueryParameters, RefCounted);

	Object* query_owner = nullptr;
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

protected:
	static void _bind_methods();

public:

	Object *get_query_owner() const { return query_owner; };
	void set_query_owner(Object *_query_owner) { query_owner = _query_owner; };

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
};


struct VisionRayTarget {
	Vector3 position;
	Node3D* node;

	VisionRayTarget() :
			position(Vector3(0,0,0)), node(nullptr){};

	VisionRayTarget(const Vector3 &_position, Node3D* _node) :
			position(_position), node(_node){};
};

enum VisionTaskState {
	Inactive,
	Starting,
	GatheringNearbyNodes,
	GatheringNearbyNodes_Finished,
	GatheringRayTargets,
	GatheringRayTargets_Finished,
	FilteringRayTargets,
	FilteringRayTargets_Finished,
};

struct VisionTask {
	WorkerThreadPool::TaskID task_id = 0;
	Ref<VisionQueryParameters> &query = Ref<VisionQueryParameters>();
	PhysicsDirectSpaceState3D* space = nullptr;
	Mutex mutex;
	VisionTaskState state = VisionTaskState::Inactive;

	// Intermediate result
	TypedArray<Node> nearby_nodes;

	Vector<VisionRayTarget> ray_targets;

	TypedArray<Node> visible_nodes;

	TypedArray<Vector3> debug_filtered_by_angle;
	TypedArray<Vector3> debug_filtered_by_angle_h;
	TypedArray<Vector3> debug_filtered_by_angle_v;
	TypedArray<Vector3> debug_filtered_by_ray;

	class VisibilitySystem *system;

	VisionTask() {};
	VisionTask(Ref<VisionQueryParameters> &_query, PhysicsDirectSpaceState3D *_space) :
		query(_query), space(_space){};
};


struct DebugLine
{
	Vector3 start;
	Vector3 end;
	Color color;
	float duration;

	DebugLine() :
		start(Vector3(0, 0, 0)), end(Vector3(0, 0, 0)), color(Color()), duration(0.0){};
	DebugLine(Vector3 _start, Vector3 _end, Color _color, float _duration) :
		start(_start), end(_end), color(_color), duration(_duration){};
};

class VisibilitySystem : public Node
{
	GDCLASS(VisibilitySystem, Node);

	Vector<VisibilityData> visibility_datas;

	Vector<VisibilityComponent *> visibility_components;

	WorkerThreadPool::TaskID vision_task_id;
	VisionTask vision_task;

	Vector<VisionQueryParameters> pending_queries;

protected:
	static void _bind_methods();

public:

	VisibilitySystem();

	static VisibilitySystem *get_visibility_system(const Node& node);

	void update_vision_system(float delta);

	void request_vision_query(const Ref<VisionQueryParameters> &query);
	void progress_task_on_thread(VisionTask &vision_task);
	static void run_vision_query_task(void *p_arg);

	static void task_gather_nearby_nodes(VisionTask &vision_task);
	static void task_gather_node_targets(VisionTask &vision_task);
	static void task_filter_obstructed_nodes(VisionTask &vision_task);

	static void calculate_vision_asynchronous(
		const Ref<VisionQueryParameters> &p_query,
		PhysicsDirectSpaceState3D *space,
		TypedArray<Node> r_visible_nodes);


	TypedArray<Node> calculate_vision_synchronous(const Ref<VisionQueryParameters> &p_query);
	static bool is_within_vision_angle(const Vector3 &point, const Ref<VisionQueryParameters> &p_query, VisibilitySystem *system = nullptr);


	void update_visibility_data();
	TypedArray<Node> get_nearby_visibility_components(const Vector3 &location, float max_distance_sqr);
	void register_visibility_component(VisibilityComponent* visibility_component);
	void unregister_visibility_component(VisibilityComponent *visibility_component);

	void print_visibility_components();

	Vector<DebugLine> debug_lines;
};

#endif // AIMOD_H
