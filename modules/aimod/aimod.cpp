/* aimod.cpp */

#include "aimod.h"
#include "core/variant/typed_array.h"
#include "servers/physics_server_3d.h"
#include "scene/main/window.h"
#include "core/string/print_string.h"

#include "scene/3d/n_character_3d.h" // need this for vision targets

using namespace godot;


#define MAGENTA Color::hex(0xFF00FFFF)
#define BLUE Color::hex(0x0000FFFF)
#define PURPLE Color::hex(0xA020F0FF)
#define RED Color::hex(0xFF0000FF)
#define ORANGE Color::hex(0xFFA500FF)
#define YELLOW Color::hex(0xFFFF00FF)
#define YELLOW_GREEN Color::hex(0x9ACD32FF)
#define GREEN Color::hex(0x00FF00FF)
#define GREEN_YELLOW Color::hex(0xADFF2FFF)

void VisionQueryParameters::_bind_methods() {

	ClassDB::bind_method(D_METHOD("get_query_owner"), &VisionQueryParameters::get_query_owner);
	ClassDB::bind_method(D_METHOD("set_query_owner", "query_owner"), &VisionQueryParameters::set_query_owner);

	ClassDB::bind_method(D_METHOD("get_owner_position"), &VisionQueryParameters::get_owner_position);
	ClassDB::bind_method(D_METHOD("set_owner_position", "owner_position"), &VisionQueryParameters::set_owner_position);

	ClassDB::bind_method(D_METHOD("get_eye_position"), &VisionQueryParameters::get_eye_position);
	ClassDB::bind_method(D_METHOD("set_eye_position", "eye_position"), &VisionQueryParameters::set_eye_position);
	ClassDB::bind_method(D_METHOD("get_eye_forward"), &VisionQueryParameters::get_eye_forward);
	ClassDB::bind_method(D_METHOD("set_eye_forward", "eye_forward"), &VisionQueryParameters::set_eye_forward);

	ClassDB::bind_method(D_METHOD("get_range"), &VisionQueryParameters::get_range);
	ClassDB::bind_method(D_METHOD("set_range", "range"), &VisionQueryParameters::set_range);

	ClassDB::bind_method(D_METHOD("get_vision_angle_horizontal"), &VisionQueryParameters::get_vision_angle_horizontal);
	ClassDB::bind_method(D_METHOD("set_vision_angle_horizontal", "vision_angle_horizontal"), &VisionQueryParameters::set_vision_angle_horizontal);
	ClassDB::bind_method(D_METHOD("get_vision_angle_vertical"), &VisionQueryParameters::get_vision_angle_vertical);
	ClassDB::bind_method(D_METHOD("set_vision_angle_vertical", "vision_angle_vertical"), &VisionQueryParameters::set_vision_angle_vertical);
	ClassDB::bind_method(D_METHOD("get_vision_angle_backwards_offset"), &VisionQueryParameters::get_vision_angle_backwards_offset);
	ClassDB::bind_method(D_METHOD("set_vision_angle_backwards_offset", "vision_angle_backwards_offset"), &VisionQueryParameters::set_vision_angle_backwards_offset);

	ClassDB::bind_method(D_METHOD("get_vision_height_high"), &VisionQueryParameters::get_vision_height_high);
	ClassDB::bind_method(D_METHOD("set_vision_height_high", "vision_height_high"), &VisionQueryParameters::set_vision_height_high);
	ClassDB::bind_method(D_METHOD("get_vision_height_low"), &VisionQueryParameters::get_vision_height_low);
	ClassDB::bind_method(D_METHOD("set_vision_height_low", "vision_height_low"), &VisionQueryParameters::set_vision_height_low);

	ClassDB::bind_method(D_METHOD("get_max_collisions"), &VisionQueryParameters::get_max_collisions);
	ClassDB::bind_method(D_METHOD("set_max_collisions", "max_collisions"), &VisionQueryParameters::set_max_collisions);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "query_owner"), "set_query_owner", "get_query_owner");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "owner_position"), "set_owner_position", "get_owner_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "eye_position"), "set_eye_position", "get_eye_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "eye_forward"), "set_eye_forward", "get_eye_forward");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "range"), "set_range", "get_range");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vision_angle_horizontal"), "set_vision_angle_horizontal", "get_vision_angle_horizontal");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vision_angle_vertical"), "set_vision_angle_vertical", "get_vision_angle_vertical");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vision_angle_backwards_offset"), "set_vision_angle_backwards_offset", "get_vision_angle_backwards_offset");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vision_height_high"), "set_vision_height_high", "get_vision_height_high");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vision_height_low"), "set_vision_height_low", "get_vision_height_low");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_collisions"), "set_max_collisions", "get_max_collisions");
}

void VisibilityComponent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_gameplay_owner_path"), &VisibilityComponent::get_gameplay_owner_path);
	ClassDB::bind_method(D_METHOD("set_gameplay_owner_path", "path"), &VisibilityComponent::set_gameplay_owner_path);

	ClassDB::bind_method(D_METHOD("get_gameplay_owner"), &VisibilityComponent::get_gameplay_owner);
	ClassDB::bind_method(D_METHOD("set_gameplay_owner", "owner"), &VisibilityComponent::set_gameplay_owner);
	ClassDB::bind_method(D_METHOD("blabla"), &VisibilityComponent::blabla);

	ClassDB::bind_method(D_METHOD("get_is_visibility_enabled"), &VisibilityComponent::get_is_visibility_enabled);
	ClassDB::bind_method(D_METHOD("set_is_visibility_enabled", "is_enabled"), &VisibilityComponent::set_is_visibility_enabled);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "gameplay_owner_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_gameplay_owner_path", "get_gameplay_owner_path");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gameplay_owner", PROPERTY_HINT_NODE_TYPE, "Node3D"), "set_gameplay_owner", "get_gameplay_owner");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_visibility_enabled"), "set_is_visibility_enabled", "get_is_visibility_enabled");
}
void VisibilityComponent::blabla()
{
	print_line("blabla was called!! ");
}

void VisibilityComponent::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	VisibilitySystem *visibility_system = VisibilitySystem::get_visibility_system(*this);
	if (!visibility_system) {
		print_line("Couldn't get visibility system. Exiting VisibilityComponent::_ready ");
		return;
	}

	visibility_system->register_visibility_component(this);

	connect("tree_exiting", callable_mp(this, &VisibilityComponent::_tree_exiting), 0); //CONNECT_DEFERRED);
}

void VisibilityComponent::_tree_exiting() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	print_line("_tree_exiting was called!! ");

	VisibilitySystem *visibility_system = VisibilitySystem::get_visibility_system(*this);
	if (!visibility_system) {
		print_line("Couldn't get visibility system. Exiting VisibilityComponent::_tree_exited");
		return;
	}

	visibility_system->unregister_visibility_component(this);
}

VisibilityComponent::VisibilityComponent() {
	connect("ready", callable_mp(this, &VisibilityComponent::_ready), 0); //CONNECT_DEFERRED);
}

void EyeComponent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_visible_nodes"), &EyeComponent::get_visible_nodes);
}

void VisibilitySystem::_bind_methods() {

	ADD_SIGNAL(MethodInfo("gather_visibility_components", PropertyInfo(Variant::ARRAY, "components", PROPERTY_HINT_NONE, "")));
	ADD_SIGNAL(MethodInfo("vision_query_complete", PropertyInfo(Variant::ARRAY, "visible_nodes", PROPERTY_HINT_NONE, "")));
	ADD_SIGNAL(MethodInfo("vision_query_complete_debug",
		PropertyInfo(Variant::ARRAY, "visible_nodes", PROPERTY_HINT_NONE, ""),
		PropertyInfo(Variant::ARRAY, "nearby_nodes", PROPERTY_HINT_NONE, ""),
		PropertyInfo(Variant::ARRAY, "filtered_by_angle", PROPERTY_HINT_NONE, ""),
		PropertyInfo(Variant::ARRAY, "filtered_by_angle_h", PROPERTY_HINT_NONE, ""),
		PropertyInfo(Variant::ARRAY, "filtered_by_angle_v", PROPERTY_HINT_NONE, ""),
		PropertyInfo(Variant::ARRAY, "filtered_by_ray", PROPERTY_HINT_NONE, "")));

	ADD_SIGNAL(MethodInfo("draw_line",
			PropertyInfo(Variant::VECTOR3, "start", PROPERTY_HINT_NONE, ""),
			PropertyInfo(Variant::VECTOR3, "end", PROPERTY_HINT_NONE, ""),
			PropertyInfo(Variant::COLOR, "color", PROPERTY_HINT_NONE, ""), 
			PropertyInfo(Variant::FLOAT, "duration", PROPERTY_HINT_NONE, "")));

	ClassDB::bind_method(D_METHOD("update_vision_system", "delta"), &VisibilitySystem::update_vision_system);
	ClassDB::bind_method(D_METHOD("request_vision_query", "query"), &VisibilitySystem::request_vision_query);
	ClassDB::bind_method(D_METHOD("update_visibility_data"), &VisibilitySystem::update_visibility_data);
	ClassDB::bind_method(D_METHOD("get_nearby_visibility_components", "location", "max_distance_sqr"), &VisibilitySystem::get_nearby_visibility_components);
	ClassDB::bind_method(D_METHOD("calculate_vision", "query"), &VisibilitySystem::calculate_vision_synchronous);
	ClassDB::bind_method(D_METHOD("register_visibility_component", "visibility_component"), &VisibilitySystem::register_visibility_component);
	ClassDB::bind_method(D_METHOD("unregister_visibility_component", "visibility_component"), &VisibilitySystem::unregister_visibility_component);
	ClassDB::bind_method(D_METHOD("print_visibility_components"), &VisibilitySystem::print_visibility_components);
}


VisibilitySystem::VisibilitySystem() {

	if (!Engine::get_singleton() || Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	//SceneTree *scene_tree = get_tree();
	//if (scene_tree) {
	//	scene_tree->connect("physics_frame", callable_mp(this, &VisibilitySystem::_physics_process), 0); //CONNECT_DEFERRED);
	//}
	
}

VisibilitySystem* VisibilitySystem::get_visibility_system(const Node& node) {
	SceneTree *scene_tree = node.get_tree();
	if (scene_tree == nullptr) {
		print_line("get_tree()  was null. Can't VisibilitySystem::get_visibility_system");
		return nullptr;
	}
	Window *root = scene_tree->get_root();
	if (root == nullptr) {
		print_line("get_tree()->get_root()  was null. Can't VisibilitySystem::get_visibility_system");
		return nullptr;
	}
	VisibilitySystem *visibility_system = Object::cast_to<VisibilitySystem>(root->get_node(NodePath("VisibilitySystemGlobal")));
	if (visibility_system == nullptr) {
		print_line("visibility_system  was null. Can't VisibilitySystem::get_visibility_system");
		return nullptr;
	}

	return visibility_system;
}


void VisibilitySystem::update_vision_system(float delta){

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	VisionTaskState state = VisionTaskState::Inactive;
	if (vision_task.mutex.try_lock()) {
		state = vision_task.state;
		vision_task.mutex.unlock();
	}

	switch (state) {
		case VisionTaskState::GatheringNearbyNodes_Finished:
			task_gather_node_targets(vision_task);
			// that's a synchronous function, so I can progress to the next stage immediately.
			progress_task_on_thread(vision_task);
			break;

		case VisionTaskState::FilteringRayTargets_Finished:
			if (vision_task.mutex.try_lock()) {
				print_line("vision task complete!!!!!");
				emit_signal(SNAME("vision_query_complete"), vision_task.visible_nodes);
				emit_signal(SNAME("vision_query_complete_debug"),
					vision_task.visible_nodes,
					vision_task.nearby_nodes,
					vision_task.debug_filtered_by_angle,
					vision_task.debug_filtered_by_angle_h,
					vision_task.debug_filtered_by_angle_v,
					vision_task.debug_filtered_by_ray);
				vision_task.state = VisionTaskState::Inactive;
				vision_task.mutex.unlock();
			}
			break;
	}

	if (vision_task.mutex.try_lock()) {
		for (int i = 0; i < debug_lines.size(); i++) {
			DebugLine &debug_line = debug_lines.get(i);
			emit_signal(SNAME("draw_line"),
					debug_line.start,
					debug_line.end,
					debug_line.color,
					debug_line.duration);
		}
		debug_lines.clear();
		vision_task.mutex.unlock();
	}
}

void VisibilitySystem::request_vision_query(const Ref<VisionQueryParameters> &query) {
	vision_task.query = query;
	vision_task.space = get_tree()->get_root()->get_world_3d()->get_direct_space_state();
	vision_task.nearby_nodes.clear();
	vision_task.ray_targets.clear();
	vision_task.visible_nodes.clear();

	// debug things, these should be removed, or at least #if debug'd out
	vision_task.debug_filtered_by_angle.clear();
	vision_task.debug_filtered_by_angle_h.clear();
	vision_task.debug_filtered_by_angle_v.clear();
	vision_task.debug_filtered_by_ray.clear();
	vision_task.system = this;

	vision_task.state = VisionTaskState::Starting;

	// probably move this to some queue?
	progress_task_on_thread(vision_task);
}

void VisibilitySystem::progress_task_on_thread(VisionTask &_vision_task) {
	if (_vision_task.mutex.try_lock()) {
		_vision_task.mutex.unlock();
		vision_task_id = WorkerThreadPool::get_singleton()->add_native_task(
				&VisibilitySystem::run_vision_query_task,
				&_vision_task,
				true,
				SNAME("VisionQuery"));
	}
}


void VisibilitySystem::run_vision_query_task(void* p_arg) {
	print_line("VisibilitySystem::run_vision_query_task called");

	VisionTask* vision_task = static_cast<VisionTask*>(p_arg);

	VisionTaskState state = VisionTaskState::Inactive;
	if (vision_task->mutex.try_lock()) {
		state = vision_task->state;
		vision_task->mutex.unlock();
	}

	switch (state) {
		case VisionTaskState::Starting:
			task_gather_nearby_nodes(*vision_task);
			break;
		case VisionTaskState::GatheringRayTargets_Finished:
			task_filter_obstructed_nodes(*vision_task);
			break;

	}


	// old...

	// execute the vision query?
	//calculate_vision_asynchronous(vision_task->query, vision_task->space, vision_task->visible_nodes);

	//for (int i = 0; i < vision_task->visible_nodes.size(); i++) {
	//	print_line(vision_task->visible_nodes[i]);
	//}

	// wtf am I doing this for
	//VisibilitySystem *visibility_system = get_visibility_system(*Object::cast_to<Node>(vision_task->query->get_query_owner()));
	//if (visibility_system) {
	//	visibility_system->emit_signal(SNAME("vision_query_complete"), vision_task->visible_nodes);
	//}
	// why are all these functions static?

}

void VisibilitySystem::update_visibility_data() {
	//print_line("VisibilitySystem::update_visibility_data");

	visibility_datas.resize(visibility_components.size());
	int data_index = 0;
	for (int i = 0; i < visibility_components.size(); i++) {
		if (visibility_components[i]->get_is_visibility_enabled() == false) {
			continue;
		}
		visibility_datas.set(data_index, VisibilityData(
			visibility_components[i]->get_gameplay_owner()->get_instance_id(),
			visibility_components[i]->get_gameplay_owner()->get_global_position()
		));
		// increment this only if we actually add a component to the data array
		data_index++;
	}
	visibility_datas.resize(data_index);
}

TypedArray<Node> VisibilitySystem::get_nearby_visibility_components(const Vector3& location, float max_distance_sqr) {
	TypedArray<Node> nodes;
	for (int i = 0; i < visibility_datas.size(); i++) {
		const float distance_sqr = (visibility_datas[i].position - location).length_squared();
		if (distance_sqr <= max_distance_sqr) {
			// what's the point if i'm just going to do this....
			nodes.push_back(Object::cast_to<Node>(ObjectDB::get_instance(visibility_datas[i].object_id)));
		}		
	}
	return nodes;
}


void VisibilitySystem::task_gather_nearby_nodes(VisionTask &vision_task) {
	if (vision_task.mutex.try_lock() == false) {
		print_line("VisibilitySystem::task_gather_nearby_nodes couldn't lock mutex. Exiting!");
		return;
	}
	if (!vision_task.space) {
		print_line("VisibilitySystem::task_gather_nearby_nodes no space. Exiting!");
		vision_task.state = VisionTaskState::Inactive;
		vision_task.mutex.unlock();
		return;
	}

	vision_task.state = VisionTaskState::GatheringNearbyNodes;

	// make shape
	RID sphere_rid = PhysicsServer3D::get_singleton()->sphere_shape_create();
	PhysicsServer3D::get_singleton()->shape_set_data(sphere_rid, vision_task.query->get_range());

	// make params
	PhysicsDirectSpaceState3D::ShapeParameters parameters;
	parameters.shape_rid = sphere_rid;
	parameters.transform = Transform3D(Basis(), vision_task.query->get_eye_position());
	parameters.collision_mask = (1 << 1) | (1 << 4) | (1 << 8);

	// prepare results array
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(vision_task.query->get_max_collisions());

	// Do the expensive physics query...
	int collision_count = vision_task.space->intersect_shape(parameters, results.ptrw(), vision_task.query->get_max_collisions());
	results.resize(collision_count);

	// Don't forget to free the shape!
	PhysicsServer3D::get_singleton()->free(sphere_rid);

	//for (const PhysicsDirectSpaceState3D::ShapeResult& result : results) {
	for (int i = 0; i < collision_count; i++) {
		const PhysicsDirectSpaceState3D::ShapeResult &result = results[i];

		// ignore myself
		if (result.collider == vision_task.query->get_query_owner()) {
			continue;
		}

		vision_task.nearby_nodes.push_back(result.collider);
	}

	vision_task.state = VisionTaskState::GatheringNearbyNodes_Finished;

	vision_task.mutex.unlock();
}

void VisibilitySystem::task_gather_node_targets(VisionTask &vision_task) {
	if (vision_task.mutex.try_lock() == false) {
		print_line("VisibilitySystem::task_gather_nearby_nodes couldn't lock mutex. Exiting!");
		return;
	}

	vision_task.state = VisionTaskState::GatheringRayTargets;

	for (int i = 0; i < vision_task.nearby_nodes.size(); i++) {
		Node3D *nearby_node = Object::cast_to<Node3D>(vision_task.nearby_nodes[i]);
		if (!nearby_node) {
			continue;
		}

		if (nearby_node->has_method("get_visibility_targets")) {
			TypedArray<Vector3> positions = nearby_node->call("get_visibility_targets");
			for (int j = 0; j < positions.size(); j++) {
				vision_task.ray_targets.push_back(VisionRayTarget(positions[j], nearby_node));
			}	
		} else if (nearby_node->has_method("get_global_position")) {
			Vector3 pos = nearby_node->call("get_global_position");
			vision_task.ray_targets.push_back(VisionRayTarget(pos, nearby_node));
		}
	}

	vision_task.state = VisionTaskState::GatheringRayTargets_Finished;

	vision_task.mutex.unlock();
}

void VisibilitySystem::task_filter_obstructed_nodes(VisionTask &vision_task) {
	if (vision_task.mutex.try_lock() == false) {
		print_line("VisibilitySystem::task_gather_nearby_nodes couldn't lock mutex. Exiting!");
		return;
	}

	vision_task.state = VisionTaskState::FilteringRayTargets;

	// caching this to skip rays on already seen guys.
	Node3D* previously_seen_node = nullptr;

	for (int i = 0; i < vision_task.ray_targets.size(); i++)
	{
		const VisionRayTarget &ray_target = vision_task.ray_targets[i];

		// I should be able to skip if I already saw this node.
		// All VisionRayTarget entries for the node have to be consecutive.
		if (ray_target.node == previously_seen_node) {
			continue;
		}
		// obstructed until proven un-obstructed
		bool obstructed = true;

		if (VisibilitySystem::is_within_vision_angle(ray_target.position, vision_task.query, vision_task.system) == false) {

			if (false)
			{
				Vector3 point = ray_target.position;
				const Vector3 &owner_position = vision_task.query->get_owner_position();
				const Vector3 eye_position = vision_task.query->get_eye_position();
				const Vector3 eye_forward = vision_task.query->get_eye_forward();

				const Vector3 adjusted_eye_position = eye_position - eye_forward * vision_task.query->get_vision_angle_backwards_offset();
				const Vector3 difference = point - adjusted_eye_position;

				const Vector3 horizontal_eye_forward = Vector3(eye_forward.x, 0, eye_forward.z).normalized();
				const Vector3 horizontal_direction = Vector3(difference.x, 0, difference.y).normalized();
				const float horizontal_dp = horizontal_direction.dot(horizontal_eye_forward);
				const float horizontal_angle = acos(horizontal_dp);
				// vertical Plane from eye-right and eye position
				const Plane vertical_plane = Plane(Vector3(0, -1, 0).cross(horizontal_eye_forward), eye_position);
				//const Vector3 vertical_eye_direction = vertical_plane.project(eye_forward).normalized(); // is this step necessary at all?
				const Vector3 vertical_direction = vertical_plane.project(difference).normalized();
				//const float vertical_dp = vertical_direction.dot(vertical_eye_direction);
				const float vertical_dp = vertical_direction.dot(horizontal_eye_forward);
				const float vertical_angle = acos(vertical_dp);

				if (owner_position.y - vision_task.query->get_vision_height_low() > point.y ||
						owner_position.y + vision_task.query->get_vision_height_high() < point.y) {
					// this one seems ok....
				} else if (horizontal_angle > vision_task.query->get_vision_angle_horizontal()) {
					vision_task.debug_filtered_by_angle_h.push_back(point);
				} else if (vertical_angle > vision_task.query->get_vision_angle_vertical()) {
					vision_task.debug_filtered_by_angle_v.push_back(point);
				}
			}


			vision_task.debug_filtered_by_angle.push_back(ray_target.position);
			continue;
		}

		// shoot expensive rays...
		PhysicsDirectSpaceState3D::RayParameters ray_params;
		ray_params.from = vision_task.query->get_eye_position();
		ray_params.to = ray_target.position;
		ray_params.collision_mask = (1) | (1 << 6);
		ray_params.collide_with_areas = true; // for smoke....
		// skipping ignore querier and object, because they shouldn't be on the same collision channel as the ray.

		PhysicsDirectSpaceState3D::RayResult ray_result;
		obstructed &= vision_task.space->intersect_ray(ray_params, ray_result);

		if (obstructed == false) {
			vision_task.visible_nodes.push_back(ray_target.node);
			previously_seen_node = ray_target.node;
			print_line("asynchronously seeing a node (I'm in a thread right now, can't call get_name)");
		} else {
			vision_task.debug_filtered_by_ray.push_back(ray_result.position);
		}
	}

	vision_task.state = VisionTaskState::FilteringRayTargets_Finished;

	vision_task.mutex.unlock();
}

void VisibilitySystem::calculate_vision_asynchronous(
	const Ref<VisionQueryParameters> &p_query,
	PhysicsDirectSpaceState3D *space,
	TypedArray<Node> r_visible_nodes)
{
	if (!space) {
		print_line("VisibilitySystem::calculate_vision no space. Exiting!");
		return;
	}

	// make shape
	RID sphere_rid = PhysicsServer3D::get_singleton()->sphere_shape_create();
	PhysicsServer3D::get_singleton()->shape_set_data(sphere_rid, p_query->get_range());

	// make params
	PhysicsDirectSpaceState3D::ShapeParameters parameters;
	parameters.shape_rid = sphere_rid;
	parameters.transform = Transform3D(Basis(), p_query->get_eye_position());
	parameters.collision_mask = (1 << 1) | (1 << 4) | (1 << 8);

	// prepare results array
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(p_query->get_max_collisions());

	// Do the expensive physics query...
	int collision_count = space->intersect_shape(parameters, results.ptrw(), p_query->get_max_collisions());
	results.resize(collision_count);

	// Don't forget to free the shape!
	PhysicsServer3D::get_singleton()->free(sphere_rid);

	//for (const PhysicsDirectSpaceState3D::ShapeResult& result : results) {
	for (int i = 0; i < collision_count; i++) {
		const PhysicsDirectSpaceState3D::ShapeResult &result = results[i];

		// ignore myself
		if (result.collider == p_query->get_query_owner()) {
			continue;
		}

		// get the individual locations for each point...
		Vector<Vector3> ray_targets;
		if (result.collider->has_method("get_visibility_targets")) {
			ray_targets.append_array(result.collider->call("get_visibility_targets"));
		} else if (result.collider->has_method("get_global_position")) {
			ray_targets.append(result.collider->call("get_global_position"));
		}

		// obstructed until proven un-obstructed
		bool obstructed = true;

		for (const Vector3 &ray_target : ray_targets) {
			if (VisibilitySystem::is_within_vision_angle(ray_target, p_query) == false) {
				continue;
			}

			// shoot expensive rays...
			PhysicsDirectSpaceState3D::RayParameters ray_params;
			ray_params.from = p_query->get_eye_position();
			ray_params.to = ray_target;
			ray_params.collision_mask = (1) | (1 << 6);
			ray_params.collide_with_areas = true; // for smoke....
			// skipping ignore querier and object, because they shouldn't be on the same collision channel as the ray.

			PhysicsDirectSpaceState3D::RayResult ray_result;
			obstructed &= space->intersect_ray(ray_params, ray_result);

			if (obstructed == false) {
				r_visible_nodes.push_back(result.collider);
				print_line("asynchronously seeing ", result.collider->get_class_name());
				break; // out of the ray target loop
			}
		}
	}

	return;
}


TypedArray<Node> VisibilitySystem::calculate_vision_synchronous(const Ref<VisionQueryParameters> &p_query) {

	TypedArray<Node> visible_nodes;
	PhysicsDirectSpaceState3D *space = get_tree()->get_root()->get_world_3d()->get_direct_space_state();
	if (!space) {
		print_line("VisibilitySystem::calculate_vision no space. Exiting!");
		return visible_nodes;
	}

	// make shape
	RID sphere_rid = PhysicsServer3D::get_singleton()->sphere_shape_create();
	PhysicsServer3D::get_singleton()->shape_set_data(sphere_rid, p_query->get_range());

	// make params
	PhysicsDirectSpaceState3D::ShapeParameters parameters;
	parameters.shape_rid = sphere_rid;
	parameters.transform = Transform3D(Basis(), p_query->get_eye_position());
	parameters.collision_mask = (1 << 1) | (1 << 4) | (1 << 8);

	// prepare results array
	Vector<PhysicsDirectSpaceState3D::ShapeResult> results;
	results.resize(p_query->get_max_collisions());

	// Do the expensive physics query...
	int collision_count = space->intersect_shape(parameters, results.ptrw(), p_query->get_max_collisions());
	results.resize(collision_count);

	// Don't forget to free the shape!
	PhysicsServer3D::get_singleton()->free(sphere_rid);

	//for (const PhysicsDirectSpaceState3D::ShapeResult& result : results) {
	for (int i = 0; i < collision_count; i++) {
		const PhysicsDirectSpaceState3D::ShapeResult &result = results[i];

		// ignore myself
		if (result.collider == p_query->get_query_owner()) {
			continue;
		}

		// get the individual locations for each point...
		Vector<Vector3> ray_targets;
		if (result.collider->has_method("get_visibility_targets")) {
			ray_targets.append_array(result.collider->call("get_visibility_targets"));
		} else if (result.collider->has_method("get_global_position")) {
			ray_targets.append(result.collider->call("get_global_position"));
		}

		// obstructed until proven un-obstructed
		bool obstructed = true;

		for (const Vector3& ray_target : ray_targets) {
			if (VisibilitySystem::is_within_vision_angle(ray_target, p_query, this) == false) {
				continue;
			}

			// shoot expensive rays...
			PhysicsDirectSpaceState3D::RayParameters ray_params;
			ray_params.from = p_query->get_eye_position();
			ray_params.to = ray_target;
			ray_params.collision_mask = (1) | (1<<6);
			ray_params.collide_with_areas = true; // for smoke....
			// skipping ignore querier and object, because they shouldn't be on the same collision channel as the ray.

			PhysicsDirectSpaceState3D::RayResult ray_result;
			obstructed &= space->intersect_ray(ray_params, ray_result);

			if (obstructed == false) {
				visible_nodes.push_back(result.collider);
				break; // out of the ray target loop
			}
		}
	}

	return visible_nodes;
}

bool VisibilitySystem::is_within_vision_angle(const Vector3 &point, const Ref<VisionQueryParameters> &p_query, VisibilitySystem *system) {

	const Vector3& owner_position = p_query->get_owner_position();

	if (owner_position.y - p_query->get_vision_height_low() > point.y ||
		owner_position.y + p_query->get_vision_height_high() < point.y) {
		return false;
	}

	const Vector3 eye_position = p_query->get_eye_position();
	const Vector3 eye_forward = p_query->get_eye_forward();

	const Vector3 adjusted_eye_position = eye_position - eye_forward * p_query->get_vision_angle_backwards_offset();
	const Vector3 difference = point - adjusted_eye_position;

	const Vector3 horizontal_eye_forward = Vector3(eye_forward.x, 0, eye_forward.z).normalized();
	const Vector3 horizontal_difference = Vector3(difference.x, 0, difference.z);
	const float horizontal_dp = horizontal_difference.normalized().dot(horizontal_eye_forward);
	const float horizontal_angle = acos(horizontal_dp);
	if (horizontal_angle > p_query->get_vision_angle_horizontal()) {
		if (system) {
			system->debug_lines.push_back(DebugLine(adjusted_eye_position, adjusted_eye_position + horizontal_difference.normalized(), ORANGE, 0.1));
			system->debug_lines.push_back(DebugLine(adjusted_eye_position, adjusted_eye_position + horizontal_eye_forward.normalized(), YELLOW_GREEN, 0.1));
		}
		return false;
	}

	// vertical Plane from eye-right and eye position
	const Plane vertical_plane = Plane(Vector3(0,-1,0).cross(horizontal_eye_forward), eye_position);
	const Vector3 vertical_eye_direction = vertical_plane.project(eye_forward).normalized(); // is this step necessary at all?
	const Vector3 vertical_direction = vertical_plane.project(difference).normalized();
	const float vertical_dp = vertical_direction.dot(vertical_eye_direction);
	//const float vertical_dp = vertical_direction.dot(horizontal_eye_forward);
	const float vertical_angle = acos(vertical_dp);
	if (vertical_angle > p_query->get_vision_angle_vertical()) {
		if (system) {
			//system->debug_lines.push_back(DebugLine(adjusted_eye_position, adjusted_eye_position + vertical_direction, Color::hex(0x00FFFFFF), 0.1));
			system->debug_lines.push_back(DebugLine(adjusted_eye_position, adjusted_eye_position + eye_forward, BLUE, 0.1));
			system->debug_lines.push_back(DebugLine(adjusted_eye_position, adjusted_eye_position + vertical_eye_direction, PURPLE, 0.1));
		}
		return false;
	}

	return true;
}


void VisibilitySystem::register_visibility_component(VisibilityComponent* visibility_component) {
	print_line("VisibilitySystem::register_visibility_component");

	if (!visibility_component) {
		print_line("visibility_component was null...");
		return;
	}
	if (!visibility_component->get_gameplay_owner()) {
		print_line("visibility_component's gameplay owner was null...");
		return;
	}

	visibility_components.push_back(visibility_component);
}

void VisibilitySystem::unregister_visibility_component(VisibilityComponent* visibility_component) {
	print_line("VisibilitySystem::unregister_visibility_component");

	for (int i = 0; i < visibility_components.size(); i++) {
		if (visibility_components[i] == visibility_component) {
			visibility_components.remove_at(i);
			return;
		}
	}

	print_line("VisibilitySystem::unregister_visibility_component didn't find it? huh?");
}

void VisibilitySystem::print_visibility_components() {
	print_line("print_visibility_components: ");
	for (int i = 0; i < visibility_components.size(); i++) {

		VisibilityComponent *visibility_component = Object::cast_to<VisibilityComponent>(visibility_components[i]);
		if (visibility_component) {
			print_line("-- ", visibility_component->get_name());
		}
	}
}



