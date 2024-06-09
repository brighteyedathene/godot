/* aimod.cpp */

#include "aimod.h"
#include "core/variant/typed_array.h"
#include "servers/physics_server_3d.h"
#include "scene/main/window.h"
#include "core/string/print_string.h"

#include "scene/3d/n_character_3d.h" // need this for vision targets

using namespace godot;


VisibilitySystem *VisibilitySystem::singleton = nullptr;


void VisionQueryParameters::_bind_methods() {

	ADD_SIGNAL(MethodInfo("complete", PropertyInfo(Variant::ARRAY, "visible_nodes", PROPERTY_HINT_NONE, "")));

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
	
	ClassDB::bind_method(D_METHOD("get_detection_collision_mask"), &VisionQueryParameters::get_detection_collision_mask);
	ClassDB::bind_method(D_METHOD("set_detection_collision_mask", "detection_collision_mask"), &VisionQueryParameters::set_detection_collision_mask);
		
	ClassDB::bind_method(D_METHOD("get_occlusion_collision_mask"), &VisionQueryParameters::get_occlusion_collision_mask);
	ClassDB::bind_method(D_METHOD("set_occlusion_collision_mask", "occlusion_collision_mask"), &VisionQueryParameters::set_occlusion_collision_mask);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "query_owner", PROPERTY_HINT_RESOURCE_TYPE, "Node"), "set_query_owner", "get_query_owner");
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
	ADD_PROPERTY(PropertyInfo(Variant::INT, "detection_collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_detection_collision_mask", "get_detection_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "occlusion_collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_occlusion_collision_mask", "get_occlusion_collision_mask");
}


void VisibilitySystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("calculate_vision", "query"), &VisibilitySystem::calculate_vision);
}


VisibilitySystem::VisibilitySystem() {
	singleton = this;
}


TypedArray<Node> VisibilitySystem::calculate_vision(const Ref<VisionQueryParameters> &p_query) {

	TypedArray<Node> visible_nodes;
	Node *query_owner = p_query->get_query_owner();
	if (!query_owner) {
		print_line("VisibilitySystem::calculate_vision_synchronous invalid query_owner. Exiting!");
		return visible_nodes;
	}
	PhysicsDirectSpaceState3D *space = query_owner->get_tree()->get_root()->get_world_3d()->get_direct_space_state();
	if (!space) {
		print_line("VisibilitySystem::calculate_vision_synchronous no space. Exiting!");
		return visible_nodes;
	}

	// make shape
	RID sphere_rid = PhysicsServer3D::get_singleton()->sphere_shape_create();
	PhysicsServer3D::get_singleton()->shape_set_data(sphere_rid, p_query->get_range());

	// make params
	PhysicsDirectSpaceState3D::ShapeParameters parameters;
	parameters.shape_rid = sphere_rid;
	parameters.transform = Transform3D(Basis(), p_query->get_eye_position());
	parameters.collision_mask = p_query->get_detection_collision_mask();

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

		Node3D *collider_as_node3d = Object::cast_to<Node3D>(result.collider);

		// get the individual locations for each point...
		Vector<Vector3> ray_targets;
		if (result.collider->has_method("get_visibility_targets")) {
			ray_targets.append_array(result.collider->call("get_visibility_targets"));
		} else{
			ray_targets.append(collider_as_node3d->get_global_position());
		}

		// obstructed until proven un-obstructed
		bool obstructed = true;

		for (const Vector3& ray_target : ray_targets) {
			if (VisibilitySystem::is_within_vision_angle(ray_target, p_query) == false) {
				continue;
			}

			// shoot expensive rays...
			PhysicsDirectSpaceState3D::RayParameters ray_params;
			ray_params.from = p_query->get_eye_position();
			ray_params.to = ray_target;
			ray_params.collision_mask = p_query->get_occlusion_collision_mask();
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

bool VisibilitySystem::is_within_vision_angle(const Vector3 &point, const Ref<VisionQueryParameters> &p_query) {

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
		return false;
	}

	return true;
}
