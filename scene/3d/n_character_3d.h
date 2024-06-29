/**************************************************************************/
/*  n_character_3d.h                                                     */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#ifndef N_CHARACTER_3D_H
#define N_CHARACTER_3D_H

#include "scene/3d/physics/kinematic_collision_3d.h"
#include "scene/3d/physics/physics_body_3d.h"


class NCharacter3D : public PhysicsBody3D {
	GDCLASS(NCharacter3D, PhysicsBody3D);

public:
	enum MotionMode {
		MOTION_MODE_GROUNDED,
		MOTION_MODE_FLOATING,
	};
	enum PlatformOnLeave {
		PLATFORM_ON_LEAVE_ADD_VELOCITY,
		PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY,
		PLATFORM_ON_LEAVE_DO_NOTHING,
	};
	bool move_and_slide();
	void apply_floor_snap();

	const Vector3 &get_velocity() const;
	void set_velocity(const Vector3 &p_velocity);

	bool is_on_floor() const;
	bool is_on_floor_only() const;
	bool is_on_wall() const;
	bool is_on_wall_only() const;
	bool is_on_ceiling() const;
	bool is_on_ceiling_only() const;
	const Vector3 &get_last_motion() const;
	Vector3 get_position_delta() const;
	const Vector3 &get_floor_normal() const;
	const Vector3 &get_wall_normal() const;
	const Vector3 &get_real_velocity() const;
	real_t get_floor_angle(const Vector3 &p_up_direction = Vector3(0.0, 1.0, 0.0)) const;
	const Vector3 &get_platform_velocity() const;
	const Vector3 &get_platform_angular_velocity() const;

	virtual Vector3 get_linear_velocity() const override;

	int get_slide_collision_count() const;
	PhysicsServer3D::MotionResult get_slide_collision(int p_bounce) const;

	void set_safe_margin(real_t p_margin);
	real_t get_safe_margin() const;

	bool is_floor_stop_on_slope_enabled() const;
	void set_floor_stop_on_slope_enabled(bool p_enabled);

	bool is_floor_constant_speed_enabled() const;
	void set_floor_constant_speed_enabled(bool p_enabled);

	bool is_floor_block_on_wall_enabled() const;
	void set_floor_block_on_wall_enabled(bool p_enabled);

	bool is_slide_on_ceiling_enabled() const;
	void set_slide_on_ceiling_enabled(bool p_enabled);

	int get_max_slides() const;
	void set_max_slides(int p_max_slides);

	real_t get_floor_max_angle() const;
	void set_floor_max_angle(real_t p_radians);

	real_t get_floor_snap_length();
	void set_floor_snap_length(real_t p_floor_snap_length);

	real_t get_wall_min_slide_angle() const;
	void set_wall_min_slide_angle(real_t p_radians);

	uint32_t get_platform_floor_layers() const;
	void set_platform_floor_layers(const uint32_t p_exclude_layer);

	uint32_t get_platform_wall_layers() const;
	void set_platform_wall_layers(const uint32_t p_exclude_layer);

	void set_motion_mode(MotionMode p_mode);
	MotionMode get_motion_mode() const;

	void set_platform_on_leave(PlatformOnLeave p_on_leave_velocity);
	PlatformOnLeave get_platform_on_leave() const;

	
	bool get_step_enabled() const { return step_enabled; };
	void set_step_enabled(bool p_enabled) { step_enabled = p_enabled; };

	real_t get_max_step_height() const { return max_step_height; };
	void set_max_step_height(real_t p_meters) { max_step_height = p_meters; };

	real_t get_max_step_lookahead() const { return max_step_lookahead; };
	void set_max_step_lookahead(real_t p_meters) { max_step_lookahead = p_meters; };


	// try to get the base of a capsule collision shape
	Vector3 get_capsule_base();

	NCharacter3D();

private:
	real_t margin = 0.001;
	MotionMode motion_mode = MOTION_MODE_GROUNDED;
	PlatformOnLeave platform_on_leave = PLATFORM_ON_LEAVE_ADD_VELOCITY;
	union CollisionState {
		uint32_t state = 0;
		struct {
			bool floor;
			bool wall;
			bool ceiling;
		};

		CollisionState() {
		}

		CollisionState(bool p_floor, bool p_wall, bool p_ceiling) {
			floor = p_floor;
			wall = p_wall;
			ceiling = p_ceiling;
		}
	};

	CollisionState collision_state;
	bool floor_constant_speed = false;
	bool floor_stop_on_slope = true;
	bool floor_block_on_wall = true;
	bool slide_on_ceiling = true;
	int max_slides = 6;
	int platform_layer = 0;
	RID platform_rid;
	ObjectID platform_object_id;
	uint32_t platform_floor_layers = UINT32_MAX;
	uint32_t platform_wall_layers = 0;
	real_t floor_snap_length = 0.1;
	real_t floor_max_angle = Math::deg_to_rad((real_t)45.0);
	real_t wall_min_slide_angle = Math::deg_to_rad((real_t)15.0);
	Vector3 up_direction = Vector3(0.0, 1.0, 0.0);
	Vector3 velocity;
	Vector3 floor_normal;
	Vector3 wall_normal;
	Vector3 ceiling_normal;
	Vector3 last_motion;
	Vector3 platform_velocity;
	Vector3 platform_angular_velocity;
	Vector3 platform_ceiling_velocity;
	Vector3 previous_position;
	Vector3 real_velocity;

	bool step_enabled = false;
	real_t max_step_height = 0.0;
	real_t max_step_lookahead = 0.2;

	Vector3 last_step_ray_start = Vector3(0, 0, 0);
	Vector3 last_step_ray_end = Vector3(0, 0, 0);


	Vector<PhysicsServer3D::MotionResult> motion_results;
	Vector<Ref<KinematicCollision3D>> slide_colliders;

	void _move_and_slide_floating(double p_delta);
	void _move_and_slide_floating_motion(Vector3 motion, int p_max_slides);
	void _move_and_slide_grounded(double p_delta, bool p_was_on_floor);

	bool is_walkable_ground(PhysicsServer3D::MotionResult p_result) const;

	void _move_and_slide_grounded_motion(Vector3 motion, bool p_was_on_floor);

	bool move_and_collide_with_step_up(const PhysicsServer3D::MotionParameters &p_parameters, PhysicsServer3D::MotionResult &r_result, bool p_test_only = false, bool p_cancel_sliding = true);

	Ref<KinematicCollision3D> _get_slide_collision(int p_bounce);
	Ref<KinematicCollision3D> _get_last_slide_collision();
	const Vector3 &get_up_direction() const;
	bool _on_floor_if_snapped(bool p_was_on_floor, bool p_vel_dir_facing_up);
	void set_up_direction(const Vector3 &p_up_direction);
	void _set_collision_direction(const PhysicsServer3D::MotionResult &p_result, CollisionState &r_state, CollisionState p_apply_state = CollisionState(true, true, true));
	void _set_platform_data(const PhysicsServer3D::MotionCollision &p_collision);
	void _snap_on_floor(bool p_was_on_floor, bool p_vel_dir_facing_up);

protected:
	void _notification(int p_what);
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	struct StateCache {
		Transform3D transform;
		Vector3 velocity = Vector3();
		Vector3 last_motion = Vector3();
		CollisionState collision_state = CollisionState();
		RID platform_rid = RID();
		ObjectID platform_object_id = ObjectID();
		Vector3 platform_velocity = Vector3();
		Vector3 platform_angular_velocity = Vector3();
		Vector3 platform_ceiling_velocity = Vector3();
		Vector3 floor_normal = Vector3();
		Vector3 wall_normal = Vector3();
		Vector3 ceiling_normal = Vector3();
	};

	void create_cache(StateCache &cache) {
		cache.transform = get_global_transform();
		cache.velocity = velocity;
		cache.last_motion = last_motion;
		cache.collision_state = collision_state;
		cache.platform_rid = platform_rid;
		cache.platform_object_id = platform_object_id;
		cache.platform_velocity = platform_velocity;
		cache.platform_angular_velocity = platform_angular_velocity;
		cache.platform_ceiling_velocity = platform_ceiling_velocity;
		cache.floor_normal = floor_normal;
		cache.wall_normal = wall_normal;
		cache.ceiling_normal = ceiling_normal;
	};

	void load_cache(StateCache &cache) {
		set_global_transform(cache.transform);
		velocity = cache.velocity;
		last_motion = cache.last_motion;
		collision_state = cache.collision_state;
		platform_rid = cache.platform_rid;
		platform_object_id = cache.platform_object_id;
		platform_velocity = cache.platform_velocity;
		platform_angular_velocity = cache.platform_angular_velocity;
		platform_ceiling_velocity = cache.platform_ceiling_velocity;
		floor_normal = cache.floor_normal;
		wall_normal = cache.wall_normal;
		ceiling_normal = cache.ceiling_normal;
	};

	bool stepping_up = false;

};

VARIANT_ENUM_CAST(NCharacter3D::MotionMode);
VARIANT_ENUM_CAST(NCharacter3D::PlatformOnLeave);

#endif // N_CHARACTER_3D_H
