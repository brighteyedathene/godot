/**************************************************************************/
/*  n_character_3d.cpp                                                   */
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

#include "n_character_3d.h"

//so, if you pass 45 as limit, avoid numerical precision errors when angle is 45.
#define FLOOR_ANGLE_THRESHOLD 0.01

bool NCharacter3D::move_and_slide() {
	// Hack in order to work with calling from _process as well as from _physics_process; calling from thread is risky
	double delta = Engine::get_singleton()->is_in_physics_frame() ? get_physics_process_delta_time() : get_process_delta_time();

	for (int i = 0; i < 3; i++) {
		if (locked_axis & (1 << i)) {
			velocity[i] = 0.0;
		}
	}

	Transform3D gt = get_global_transform();
	previous_position = gt.origin;

	Vector3 current_platform_velocity = platform_velocity;

	if ((collision_state.floor || collision_state.wall) && platform_rid.is_valid()) {
		bool excluded = false;
		if (collision_state.floor) {
			excluded = (platform_floor_layers & platform_layer) == 0;
		} else if (collision_state.wall) {
			excluded = (platform_wall_layers & platform_layer) == 0;
		}
		if (!excluded) {
			PhysicsDirectBodyState3D *bs = nullptr;

			// We need to check the platform_rid object still exists before accessing.
			// A valid RID is no guarantee that the object has not been deleted.
			if (ObjectDB::get_instance(platform_object_id)) {
				//this approach makes sure there is less delay between the actual body velocity and the one we saved
				bs = PhysicsServer3D::get_singleton()->body_get_direct_state(platform_rid);
			}

			if (bs) {
				Vector3 local_position = gt.origin - bs->get_transform().origin;
				current_platform_velocity = bs->get_velocity_at_local_position(local_position);
			} else {
				// Body is removed or destroyed, invalidate floor.
				current_platform_velocity = Vector3();
				platform_rid = RID();
			}
		} else {
			current_platform_velocity = Vector3();
		}
	}

	motion_results.clear();

	bool was_on_floor = collision_state.floor;
	collision_state.state = 0;

	last_motion = Vector3();

	if (!current_platform_velocity.is_zero_approx()) {
		PhysicsServer3D::MotionParameters parameters(get_global_transform(), current_platform_velocity * delta, margin);
		parameters.recovery_as_collision = true; // Also report collisions generated only from recovery.

		parameters.exclude_bodies.insert(platform_rid);
		if (platform_object_id.is_valid()) {
			parameters.exclude_objects.insert(platform_object_id);
		}

		PhysicsServer3D::MotionResult floor_result;
		if (move_and_collide(parameters, floor_result, false, false)) {
			motion_results.push_back(floor_result);

			CollisionState result_state;
			_set_collision_direction(floor_result, result_state);
		}
	}

	if (motion_mode == MOTION_MODE_GROUNDED) {
		_move_and_slide_grounded(delta, was_on_floor);
	} else {
		_move_and_slide_floating(delta);
	}

	// Compute real velocity.
	real_velocity = get_position_delta() / delta;

	if (platform_on_leave != PLATFORM_ON_LEAVE_DO_NOTHING) {
		// Add last platform velocity when just left a moving platform.
		if (!collision_state.floor && !collision_state.wall) {
			if (platform_on_leave == PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY && current_platform_velocity.dot(up_direction) < 0) {
				current_platform_velocity = current_platform_velocity.slide(up_direction);
			}
			velocity += current_platform_velocity;
		}
	}

	return motion_results.size() > 0;
}

void NCharacter3D::_move_and_slide_grounded(double p_delta, bool p_was_on_floor) {
	Vector3 motion = velocity * p_delta;

	if (velocity.is_zero_approx() == false) {
		motion = Vector3();
		motion = velocity * p_delta;
	}

	StateCache starting_move_cache;
	create_cache(starting_move_cache);

	// Try normal move
	_move_and_slide_grounded_motion(motion, p_was_on_floor);

	bool b_should_step = max_step_height > 0 && step_enabled && (p_was_on_floor || stepping_up);
	if (b_should_step == false) {
		stepping_up = false;
		return;
	}

	// Cache normal move results
	StateCache normal_move_cache;
	create_cache(normal_move_cache);

	// Try step move
	load_cache(starting_move_cache);

	// Move up by step height
	PhysicsServer3D::MotionParameters up_parameters(starting_move_cache.transform, Vector3(0, max_step_height, 0), margin);
	up_parameters.max_collisions = 1;
	PhysicsServer3D::MotionResult up_result;
	PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), up_parameters, &up_result);
	set_global_transform(starting_move_cache.transform.translated(up_result.travel));

	// Slide forwards from there
	//_move_and_slide_grounded_motion(motion, false/*p_was_on_floor*/);
	_move_and_slide_floating_motion(motion, 1);

	// Move down by step height
	PhysicsServer3D::MotionParameters down_parameters(get_global_transform(), Vector3(0, -(max_step_height + 0.001), 0), margin);
	down_parameters.max_collisions = 6;
	PhysicsServer3D::MotionResult down_result;
	//PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), down_parameters, &down_result);
	//set_global_transform(get_global_transform().translated(down_result.travel));

	bool b_down_collided = move_and_collide(down_parameters, down_result, false, false);

	if (!b_down_collided) {
		load_cache(normal_move_cache);
		stepping_up = false;
		return;
	}
	_set_collision_direction(down_result, CollisionState());

	// Cache step move results
	StateCache step_move_cache;
	create_cache(step_move_cache);

	// check for walkable ground after stepping
	bool bFoundWalkableGround = false;
	bFoundWalkableGround = is_walkable_ground(down_result);
	if (max_step_lookahead > 0 && bFoundWalkableGround == false) {
		// do another trace further ahead

		// start from just after the up trace
		set_global_transform(starting_move_cache.transform.translated(up_result.travel));
		_move_and_slide_floating_motion(motion.normalized() * max_step_lookahead, 1);

		PhysicsServer3D::MotionParameters lookahead_down_parameters(get_global_transform(), Vector3(0, -(max_step_height + CMP_EPSILON), 0), margin);
		lookahead_down_parameters.max_collisions = 6;
		PhysicsServer3D::MotionResult lookahead_down_result;
		PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), lookahead_down_parameters, &lookahead_down_result);
		if (is_walkable_ground(lookahead_down_result)) {
			bFoundWalkableGround = true;

			//_set_collision_direction(lookahead_down_result, CollisionState());

			//print_line("found walkable groung from lookahead");
		}
		last_step_ray_start = lookahead_down_parameters.from.origin;
		last_step_ray_end = last_step_ray_start + lookahead_down_result.travel;

		// restore values from before this test
		load_cache(step_move_cache);
	}

	// check which move went further
	Vector3 normal_travel = normal_move_cache.transform.origin - starting_move_cache.transform.origin;
	normal_travel.y = 0;
	Vector3 step_travel = step_move_cache.transform.origin - starting_move_cache.transform.origin;
	step_travel.y = 0;

	if (!bFoundWalkableGround || normal_travel.length_squared() > step_travel.length_squared()) {
		// Restore the original move results if the step up wasn't a success
		load_cache(normal_move_cache);
		stepping_up = false;

		//print_line("not stepping up");

	} else {
		// use the normal move's y velocity to prevent launching from step move
		velocity.y = normal_move_cache.velocity.y;
		stepping_up = true;
		//print_line("Stepping up");
	}

	// what is going on...
	//print_line_rich("Stepup up collisions (%d) ", up_result.collision_count);
	//for (int i = 0; i < up_result.collision_count; i++) {
	//	const PhysicsServer3D::MotionCollision &collision = up_result.collisions[i];
	//
	//	Object *obj = ObjectDB::get_instance(collision.collider_id);
	//	Node *obj_as_node = Object::cast_to<Node>(obj);
	//
	//	StringName node_name = String("not a node");
	//	if (obj_as_node) {
	//		node_name = obj_as_node->get_name();
	//	}
	//
	//	real_t floor_angle = collision.get_angle(up_direction);
	//	print_line_rich("Stepup up collision angle (must less than %f) %s  %f   normal:\n",
	//		floor_max_angle + FLOOR_ANGLE_THRESHOLD, String(node_name).utf8().get_data(), floor_angle, String(collision.normal).utf8().get_data());
	//}

	//print_line("Stepup down collisions (%d) ", down_result.collision_count);
	//for (int i = 0; i < down_result.collision_count; i++) {
	//	const PhysicsServer3D::MotionCollision &collision = down_result.collisions[i];
	//
	//	Object *obj = ObjectDB::get_instance(collision.collider_id);
	//	Node *obj_as_node = Object::cast_to<Node>(obj);
	//
	//	StringName node_name = String("not a node");
	//	if (obj_as_node) {
	//		node_name = obj_as_node->get_name();
	//	}
	//
	//	real_t floor_angle = collision.get_angle(up_direction);
	//	print_line("Stepup down collision angle (must less than %f) %s  %f   normal:\n",
	//			floor_max_angle + FLOOR_ANGLE_THRESHOLD, String(node_name).utf8().get_data(), floor_angle, String(collision.normal).utf8().get_data());
	//}
}

bool NCharacter3D::is_walkable_ground(PhysicsServer3D::MotionResult p_result) const {
	for (int i = 0; i < p_result.collision_count; i++) {
		const PhysicsServer3D::MotionCollision &collision = p_result.collisions[i];
		real_t floor_angle = collision.get_angle(up_direction);
		if (floor_angle <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) {
			return true;
		}
	}
	return false;
}

void NCharacter3D::_move_and_slide_grounded_motion(Vector3 motion, bool p_was_on_floor) {
	Vector3 motion_slide_up = motion.slide(up_direction);
	Vector3 prev_floor_normal = floor_normal;

	platform_rid = RID();
	platform_object_id = ObjectID();
	platform_velocity = Vector3();
	platform_angular_velocity = Vector3();
	platform_ceiling_velocity = Vector3();
	floor_normal = Vector3();
	wall_normal = Vector3();
	ceiling_normal = Vector3();

	// No sliding on first attempt to keep floor motion stable when possible,
	// When stop on slope is enabled or when there is no up direction.
	bool sliding_enabled = !floor_stop_on_slope;
	// Constant speed can be applied only the first time sliding is enabled.
	bool can_apply_constant_speed = sliding_enabled;
	// If the platform's ceiling push down the body.
	bool apply_ceiling_velocity = false;
	bool first_slide = true;
	bool vel_dir_facing_up = velocity.dot(up_direction) > 0;
	Vector3 total_travel;

	for (int iteration = 0; iteration < max_slides; ++iteration) {
		PhysicsServer3D::MotionParameters parameters(get_global_transform(), motion, margin);
		parameters.max_collisions = 6; // There can be 4 collisions between 2 walls + 2 more for the floor.
		parameters.recovery_as_collision = true; // Also report collisions generated only from recovery.

		PhysicsServer3D::MotionResult result;
		bool collided = move_and_collide(parameters, result, false, !sliding_enabled);
		//bool collided = move_and_collide_with_step_up(parameters, result, false, !sliding_enabled);

		last_motion = result.travel;

		if (collided) {
			motion_results.push_back(result);

			CollisionState previous_state = collision_state;

			CollisionState result_state;
			_set_collision_direction(result, result_state);

			// If we hit a ceiling platform, we set the vertical velocity to at least the platform one.
			if (collision_state.ceiling && platform_ceiling_velocity != Vector3() && platform_ceiling_velocity.dot(up_direction) < 0) {
				// If ceiling sliding is on, only apply when the ceiling is flat or when the motion is upward.
				if (!slide_on_ceiling || motion.dot(up_direction) < 0 || (ceiling_normal + up_direction).length() < 0.01) {
					apply_ceiling_velocity = true;
					Vector3 ceiling_vertical_velocity = up_direction * up_direction.dot(platform_ceiling_velocity);
					Vector3 motion_vertical_velocity = up_direction * up_direction.dot(velocity);
					if (motion_vertical_velocity.dot(up_direction) > 0 || ceiling_vertical_velocity.length_squared() > motion_vertical_velocity.length_squared()) {
						velocity = ceiling_vertical_velocity + velocity.slide(up_direction);
					}
				}
			}

			if (collision_state.floor && floor_stop_on_slope && (velocity.normalized() + up_direction).length() < 0.01) {
				Transform3D gt = get_global_transform();
				if (result.travel.length() <= margin + CMP_EPSILON) {
					gt.origin -= result.travel;
				}
				set_global_transform(gt);
				velocity = Vector3();
				motion = Vector3();
				last_motion = Vector3();
				break;
			}

			if (result.remainder.is_zero_approx()) {
				motion = Vector3();
				break;
			}

			// Apply regular sliding by default.
			bool apply_default_sliding = true;

			// Wall collision checks.
			if (result_state.wall && (motion_slide_up.dot(wall_normal) <= 0)) {
				// Move on floor only checks.
				if (floor_block_on_wall) {
					// Needs horizontal motion from current motion instead of motion_slide_up
					// to properly test the angle and avoid standing on slopes
					Vector3 horizontal_motion = motion.slide(up_direction);
					Vector3 horizontal_normal = wall_normal.slide(up_direction).normalized();
					real_t motion_angle = Math::abs(Math::acos(-horizontal_normal.dot(horizontal_motion.normalized())));

					// Avoid to move forward on a wall if floor_block_on_wall is true.
					// Applies only when the motion angle is under 90 degrees,
					// in order to avoid blocking lateral motion along a wall.
					if (motion_angle < .5 * Math_PI) {
						apply_default_sliding = false;
						if (p_was_on_floor && !vel_dir_facing_up) {
							// Cancel the motion.
							Transform3D gt = get_global_transform();
							real_t travel_total = result.travel.length();
							real_t cancel_dist_max = MIN(0.1, margin * 20);
							if (travel_total <= margin + CMP_EPSILON) {
								gt.origin -= result.travel;
								result.travel = Vector3(); // Cancel for constant speed computation.
							} else if (travel_total < cancel_dist_max) { // If the movement is large the body can be prevented from reaching the walls.
								gt.origin -= result.travel.slide(up_direction);
								// Keep remaining motion in sync with amount canceled.
								motion = motion.slide(up_direction);
								result.travel = Vector3();
							} else {
								// Travel is too high to be safely canceled, we take it into account.
								result.travel = result.travel.slide(up_direction);
								motion = result.remainder;
							}
							set_global_transform(gt);
							// Determines if you are on the ground, and limits the possibility of climbing on the walls because of the approximations.
							_snap_on_floor(true, false);
						} else {
							// If the movement is not canceled we only keep the remaining.
							motion = result.remainder;
						}

						// Apply slide on forward in order to allow only lateral motion on next step.
						Vector3 forward = wall_normal.slide(up_direction).normalized();
						motion = motion.slide(forward);

						// Scales the horizontal velocity according to the wall slope.
						if (vel_dir_facing_up) {
							Vector3 slide_motion = velocity.slide(result.collisions[0].normal);
							// Keeps the vertical motion from velocity and add the horizontal motion of the projection.
							velocity = up_direction * up_direction.dot(velocity) + slide_motion.slide(up_direction);
						} else {
							velocity = velocity.slide(forward);
						}

						// Allow only lateral motion along previous floor when already on floor.
						// Fixes slowing down when moving in diagonal against an inclined wall.
						if (p_was_on_floor && !vel_dir_facing_up && (motion.dot(up_direction) > 0.0)) {
							// Slide along the corner between the wall and previous floor.
							Vector3 floor_side = prev_floor_normal.cross(wall_normal);
							if (floor_side != Vector3()) {
								motion = floor_side * motion.dot(floor_side);
							}
						}

						// Stop all motion when a second wall is hit (unless sliding down or jumping),
						// in order to avoid jittering in corner cases.
						bool stop_all_motion = previous_state.wall && !vel_dir_facing_up;

						// Allow sliding when the body falls.
						if (!collision_state.floor && motion.dot(up_direction) < 0) {
							Vector3 slide_motion = motion.slide(wall_normal);
							// Test again to allow sliding only if the result goes downwards.
							// Fixes jittering issues at the bottom of inclined walls.
							if (slide_motion.dot(up_direction) < 0) {
								stop_all_motion = false;
								motion = slide_motion;
							}
						}

						if (stop_all_motion) {
							motion = Vector3();
							velocity = Vector3();
						}
					}
				}

				// Stop horizontal motion when under wall slide threshold.
				if (p_was_on_floor && (wall_min_slide_angle > 0.0) && result_state.wall) {
					Vector3 horizontal_normal = wall_normal.slide(up_direction).normalized();
					real_t motion_angle = Math::abs(Math::acos(-horizontal_normal.dot(motion_slide_up.normalized())));
					if (motion_angle < wall_min_slide_angle) {
						motion = up_direction * motion.dot(up_direction);
						velocity = up_direction * velocity.dot(up_direction);

						apply_default_sliding = false;
					}
				}
			}

			if (apply_default_sliding) {
				// Regular sliding, the last part of the test handle the case when you don't want to slide on the ceiling.
				if ((sliding_enabled || !collision_state.floor) && (!collision_state.ceiling || slide_on_ceiling || !vel_dir_facing_up) && !apply_ceiling_velocity) {
					const PhysicsServer3D::MotionCollision &collision = result.collisions[0];

					Vector3 slide_motion = result.remainder.slide(collision.normal);
					if (collision_state.floor && !collision_state.wall && !motion_slide_up.is_zero_approx()) {
						// Slide using the intersection between the motion plane and the floor plane,
						// in order to keep the direction intact.
						real_t motion_length = slide_motion.length();
						slide_motion = up_direction.cross(result.remainder).cross(floor_normal);

						// Keep the length from default slide to change speed in slopes by default,
						// when constant speed is not enabled.
						slide_motion.normalize();
						slide_motion *= motion_length;
					}

					if (slide_motion.dot(velocity) > 0.0) {
						motion = slide_motion;
					} else {
						motion = Vector3();
					}

					if (slide_on_ceiling && result_state.ceiling) {
						// Apply slide only in the direction of the input motion, otherwise just stop to avoid jittering when moving against a wall.
						if (vel_dir_facing_up) {
							velocity = velocity.slide(collision.normal);
						} else {
							// Avoid acceleration in slope when falling.
							velocity = up_direction * up_direction.dot(velocity);
						}
					}
				}
				// No sliding on first attempt to keep floor motion stable when possible.
				else {
					motion = result.remainder;
					if (result_state.ceiling && !slide_on_ceiling && vel_dir_facing_up) {
						velocity = velocity.slide(up_direction);
						motion = motion.slide(up_direction);
					}
				}
			}

			total_travel += result.travel;

			// Apply Constant Speed.
			if (p_was_on_floor && floor_constant_speed && can_apply_constant_speed && collision_state.floor && !motion.is_zero_approx()) {
				Vector3 travel_slide_up = total_travel.slide(up_direction);
				motion = motion.normalized() * MAX(0, (motion_slide_up.length() - travel_slide_up.length()));
			}
		}
		// When you move forward in a downward slope you don’t collide because you will be in the air.
		// This test ensures that constant speed is applied, only if the player is still on the ground after the snap is applied.
		else if (floor_constant_speed && first_slide && _on_floor_if_snapped(p_was_on_floor, vel_dir_facing_up)) {
			can_apply_constant_speed = false;
			sliding_enabled = true;
			Transform3D gt = get_global_transform();
			gt.origin = gt.origin - result.travel;
			set_global_transform(gt);

			// Slide using the intersection between the motion plane and the floor plane,
			// in order to keep the direction intact.
			Vector3 motion_slide_norm = up_direction.cross(motion).cross(prev_floor_normal);
			motion_slide_norm.normalize();

			motion = motion_slide_norm * (motion_slide_up.length());
			collided = true;
		}

		if (!collided || motion.is_zero_approx()) {
			break;
		}

		can_apply_constant_speed = !can_apply_constant_speed && !sliding_enabled;
		sliding_enabled = true;
		first_slide = false;
	}

	_snap_on_floor(p_was_on_floor, vel_dir_facing_up);

	// Reset the gravity accumulation when touching the ground.
	if (collision_state.floor && !vel_dir_facing_up) {
		velocity = velocity.slide(up_direction);
	}
}

bool NCharacter3D::move_and_collide_with_step_up(const PhysicsServer3D::MotionParameters &p_parameters, PhysicsServer3D::MotionResult &r_result, bool p_test_only, bool p_cancel_sliding) {
	/**
	 * It's a copy of PhysicsBody3D::move_and_collide, with an extra check for steps
	 *	If there's a collision going FORWARD, try moving UP, FORWARDS, then DOWN.
	 *	Pick whichever move goes further.
	 */
	bool colliding = PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), p_parameters, &r_result);

	// Try to step up if colliding
	if (colliding) {
		// try going up by step_height
		PhysicsServer3D::MotionParameters up_parameters = p_parameters; // does this get exclude_bodies etc?
		PhysicsServer3D::MotionResult up_result;
		up_parameters.from = p_parameters.from;
		up_parameters.motion = Vector3(0, max_step_height, 0);
		PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), up_parameters, &up_result);

		// now try going forwards by the original motion
		PhysicsServer3D::MotionParameters forward_parameters = p_parameters;
		PhysicsServer3D::MotionResult forward_result;
		forward_parameters.from = up_parameters.from.translated(up_result.travel);
		forward_parameters.motion = p_parameters.motion;
		PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), forward_parameters, &forward_result);

		// now try going back down by step height
		PhysicsServer3D::MotionParameters down_parameters = p_parameters;
		PhysicsServer3D::MotionResult down_result;
		down_parameters.from = forward_parameters.from.translated(forward_result.travel);
		down_parameters.motion = Vector3(0, -(max_step_height + CMP_EPSILON), 0);
		PhysicsServer3D::get_singleton()->body_test_motion(get_rid(), down_parameters, &down_result);

		// I think I need to make sure there is walkable ground below
		bool bFoundWalkableGround = false;
		//for (const PhysicsServer3D::MotionCollision &collision : down_result.collisions) {

		for (int i = 0; i < down_result.collision_count; i++) {
			const PhysicsServer3D::MotionCollision &collision = down_result.collisions[i];
			real_t floor_angle = collision.get_angle(up_direction);
			if (floor_angle <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) {
				bFoundWalkableGround = true;
				break;
			}
		}

		if (bFoundWalkableGround) {
			// now I compare the distance travelled by stepping vs original
			Vector3 step_destination = down_parameters.from.get_origin() + down_result.travel;
			Vector3 step_travel = step_destination - p_parameters.from.get_origin();

			if (step_travel.slide(up_direction).length_squared() > r_result.travel.slide(up_direction).length_squared()) {
				r_result = forward_result;
				r_result.travel = step_travel;

				// i might want to keep the collisions from down, if any
				for (int i = 0; i < down_result.collision_count; i++) {
					if (forward_result.collision_count + 1 < forward_result.MAX_COLLISIONS) {
						forward_result.collisions[forward_result.collision_count] = down_result.collisions[i];
						forward_result.collision_count++;
					} else {
						break;
					}
				}
			}
		}
	}

	// Restore direction of motion to be along original motion,
	// in order to avoid sliding due to recovery,
	// but only if collision depth is low enough to avoid tunneling.
	if (p_cancel_sliding) {
		real_t motion_length = p_parameters.motion.length();
		real_t precision = 0.001;

		if (colliding) {
			// Can't just use margin as a threshold because collision depth is calculated on unsafe motion,
			// so even in normal resting cases the depth can be a bit more than the margin.
			precision += motion_length * (r_result.collision_unsafe_fraction - r_result.collision_safe_fraction);

			if (r_result.collisions[0].depth > p_parameters.margin + precision) {
				p_cancel_sliding = false;
			}
		}

		if (p_cancel_sliding) {
			// When motion is null, recovery is the resulting motion.
			Vector3 motion_normal;
			if (motion_length > CMP_EPSILON) {
				motion_normal = p_parameters.motion / motion_length;
			}

			// Check depth of recovery.
			real_t projected_length = r_result.travel.dot(motion_normal);
			Vector3 recovery = r_result.travel - motion_normal * projected_length;
			real_t recovery_length = recovery.length();
			// Fixes cases where canceling slide causes the motion to go too deep into the ground,
			// because we're only taking rest information into account and not general recovery.
			if (recovery_length < p_parameters.margin + precision) {
				// Apply adjustment to motion.
				r_result.travel = motion_normal * projected_length;
				r_result.remainder = p_parameters.motion - r_result.travel;
			}
		}
	}

	for (int i = 0; i < 3; i++) {
		if (locked_axis & (1 << i)) {
			r_result.travel[i] = 0;
		}
	}

	if (!p_test_only) {
		Transform3D gt = p_parameters.from;
		gt.origin += r_result.travel;
		set_global_transform(gt);
	}

	return colliding;
}

void NCharacter3D::_move_and_slide_floating(double p_delta) {
	Vector3 motion = velocity * p_delta;
	_move_and_slide_floating_motion(motion, max_slides);
}
void NCharacter3D::_move_and_slide_floating_motion(Vector3 motion, int p_max_slides) {
	platform_rid = RID();
	platform_object_id = ObjectID();
	floor_normal = Vector3();
	platform_velocity = Vector3();
	platform_angular_velocity = Vector3();

	bool first_slide = true;
	for (int iteration = 0; iteration < max_slides; ++iteration) {
		PhysicsServer3D::MotionParameters parameters(get_global_transform(), motion, margin);
		parameters.recovery_as_collision = true; // Also report collisions generated only from recovery.

		PhysicsServer3D::MotionResult result;
		bool collided = move_and_collide(parameters, result, false, false);

		last_motion = result.travel;

		if (collided) {
			motion_results.push_back(result);

			CollisionState result_state;
			_set_collision_direction(result, result_state);

			if (result.remainder.is_zero_approx()) {
				motion = Vector3();
				break;
			}

			if (wall_min_slide_angle != 0 && Math::acos(wall_normal.dot(-velocity.normalized())) < wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD) {
				motion = Vector3();
				if (result.travel.length() < margin + CMP_EPSILON) {
					Transform3D gt = get_global_transform();
					gt.origin -= result.travel;
					set_global_transform(gt);
				}
			} else if (first_slide) {
				Vector3 motion_slide_norm = result.remainder.slide(wall_normal).normalized();
				motion = motion_slide_norm * (motion.length() - result.travel.length());
			} else {
				motion = result.remainder.slide(wall_normal);
			}

			if (motion.dot(velocity) <= 0.0) {
				motion = Vector3();
			}
		}

		if (!collided || motion.is_zero_approx()) {
			break;
		}

		first_slide = false;
	}
}

void NCharacter3D::apply_floor_snap() {
	if (collision_state.floor) {
		return;
	}

	// Snap by at least collision margin to keep floor state consistent.
	real_t length = MAX(floor_snap_length, margin);

	PhysicsServer3D::MotionParameters parameters(get_global_transform(), -up_direction * length, margin);
	parameters.max_collisions = 4;
	parameters.recovery_as_collision = true; // Also report collisions generated only from recovery.
	parameters.collide_separation_ray = true;

	PhysicsServer3D::MotionResult result;
	if (move_and_collide(parameters, result, true, false)) {
		CollisionState result_state;
		// Apply direction for floor only.
		_set_collision_direction(result, result_state, CollisionState(true, false, false));

		if (result_state.floor) {
			if (floor_stop_on_slope) {
				// move and collide may stray the object a bit because of pre un-stucking,
				// so only ensure that motion happens on floor direction in this case.
				if (result.travel.length() > margin) {
					result.travel = up_direction * up_direction.dot(result.travel);
				} else {
					result.travel = Vector3();
				}
			}

			parameters.from.origin += result.travel;
			set_global_transform(parameters.from);
		}
	}
}

void NCharacter3D::_snap_on_floor(bool p_was_on_floor, bool p_vel_dir_facing_up) {
	if (collision_state.floor || !p_was_on_floor || p_vel_dir_facing_up) {
		return;
	}

	apply_floor_snap();
}

bool NCharacter3D::_on_floor_if_snapped(bool p_was_on_floor, bool p_vel_dir_facing_up) {
	if (up_direction == Vector3() || collision_state.floor || !p_was_on_floor || p_vel_dir_facing_up) {
		return false;
	}

	// Snap by at least collision margin to keep floor state consistent.
	real_t length = MAX(floor_snap_length, margin);

	PhysicsServer3D::MotionParameters parameters(get_global_transform(), -up_direction * length, margin);
	parameters.max_collisions = 4;
	parameters.recovery_as_collision = true; // Also report collisions generated only from recovery.
	parameters.collide_separation_ray = true;

	PhysicsServer3D::MotionResult result;
	if (move_and_collide(parameters, result, true, false)) {
		CollisionState result_state;
		// Don't apply direction for any type.
		_set_collision_direction(result, result_state, CollisionState());

		return result_state.floor;
	}

	return false;
}

void NCharacter3D::_set_collision_direction(const PhysicsServer3D::MotionResult &p_result, CollisionState &r_state, CollisionState p_apply_state) {
	r_state.state = 0;

	real_t wall_depth = -1.0;
	real_t floor_depth = -1.0;

	bool was_on_wall = collision_state.wall;
	Vector3 prev_wall_normal = wall_normal;
	int wall_collision_count = 0;
	Vector3 combined_wall_normal;
	Vector3 tmp_wall_col; // Avoid duplicate on average calculation.

	for (int i = p_result.collision_count - 1; i >= 0; i--) {
		const PhysicsServer3D::MotionCollision &collision = p_result.collisions[i];

		if (motion_mode == MOTION_MODE_GROUNDED) {
			// Check if any collision is floor.
			// It can't be a character
			if (Object::cast_to<NCharacter3D>(ObjectDB::get_instance(collision.collider_id)) == nullptr) {
				real_t floor_angle = collision.get_angle(up_direction);
				if (floor_angle <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) {
					r_state.floor = true;
					if (p_apply_state.floor && collision.depth > floor_depth) {
						collision_state.floor = true;
						floor_normal = collision.normal;
						floor_depth = collision.depth;
						_set_platform_data(collision);
					}
					continue;
				}
			}

			// Check if any collision is ceiling.
			real_t ceiling_angle = collision.get_angle(-up_direction);
			if (ceiling_angle <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) {
				r_state.ceiling = true;
				if (p_apply_state.ceiling) {
					platform_ceiling_velocity = collision.collider_velocity;
					ceiling_normal = collision.normal;
					collision_state.ceiling = true;
				}
				continue;
			}
		}

		// Collision is wall by default.
		r_state.wall = true;

		if (p_apply_state.wall && collision.depth > wall_depth) {
			collision_state.wall = true;
			wall_depth = collision.depth;
			wall_normal = collision.normal;

			// Don't apply wall velocity when the collider is a NCharacter3D.
			if (Object::cast_to<NCharacter3D>(ObjectDB::get_instance(collision.collider_id)) == nullptr) {
				_set_platform_data(collision);
			}
		}

		// Collect normal for calculating average.
		if (!collision.normal.is_equal_approx(tmp_wall_col)) {
			tmp_wall_col = collision.normal;
			combined_wall_normal += collision.normal;
			wall_collision_count++;
		}
	}

	if (r_state.wall) {
		if (wall_collision_count > 1 && !r_state.floor) {
			// Check if wall normals cancel out to floor support.
			if (!r_state.floor && motion_mode == MOTION_MODE_GROUNDED) {
				combined_wall_normal.normalize();
				real_t floor_angle = Math::acos(combined_wall_normal.dot(up_direction));
				if (floor_angle <= floor_max_angle + FLOOR_ANGLE_THRESHOLD) {
					r_state.floor = true;
					r_state.wall = false;
					if (p_apply_state.floor) {
						collision_state.floor = true;
						floor_normal = combined_wall_normal;
					}
					if (p_apply_state.wall) {
						collision_state.wall = was_on_wall;
						wall_normal = prev_wall_normal;
					}
					return;
				}
			}
		}
	}
}

void NCharacter3D::_set_platform_data(const PhysicsServer3D::MotionCollision &p_collision) {
	platform_rid = p_collision.collider;
	platform_object_id = p_collision.collider_id;
	platform_velocity = p_collision.collider_velocity;
	platform_angular_velocity = p_collision.collider_angular_velocity;
	platform_layer = PhysicsServer3D::get_singleton()->body_get_collision_layer(platform_rid);
}

void NCharacter3D::set_safe_margin(real_t p_margin) {
	margin = p_margin;
}

real_t NCharacter3D::get_safe_margin() const {
	return margin;
}

const Vector3 &NCharacter3D::get_velocity() const {
	return velocity;
}

void NCharacter3D::set_velocity(const Vector3 &p_velocity) {
	velocity = p_velocity;
}

bool NCharacter3D::is_on_floor() const {
	return collision_state.floor;
}

bool NCharacter3D::is_on_floor_only() const {
	return collision_state.floor && !collision_state.wall && !collision_state.ceiling;
}

bool NCharacter3D::is_on_wall() const {
	return collision_state.wall;
}

bool NCharacter3D::is_on_wall_only() const {
	return collision_state.wall && !collision_state.floor && !collision_state.ceiling;
}

bool NCharacter3D::is_on_ceiling() const {
	return collision_state.ceiling;
}

bool NCharacter3D::is_on_ceiling_only() const {
	return collision_state.ceiling && !collision_state.floor && !collision_state.wall;
}

const Vector3 &NCharacter3D::get_floor_normal() const {
	return floor_normal;
}

const Vector3 &NCharacter3D::get_wall_normal() const {
	return wall_normal;
}

const Vector3 &NCharacter3D::get_last_motion() const {
	return last_motion;
}

Vector3 NCharacter3D::get_position_delta() const {
	return get_global_transform().origin - previous_position;
}

const Vector3 &NCharacter3D::get_real_velocity() const {
	return real_velocity;
}

real_t NCharacter3D::get_floor_angle(const Vector3 &p_up_direction) const {
	ERR_FAIL_COND_V(p_up_direction == Vector3(), 0);
	return Math::acos(floor_normal.dot(p_up_direction));
}

const Vector3 &NCharacter3D::get_platform_velocity() const {
	return platform_velocity;
}

const Vector3 &NCharacter3D::get_platform_angular_velocity() const {
	return platform_angular_velocity;
}

Vector3 NCharacter3D::get_linear_velocity() const {
	return get_real_velocity();
}

int NCharacter3D::get_slide_collision_count() const {
	return motion_results.size();
}

PhysicsServer3D::MotionResult NCharacter3D::get_slide_collision(int p_bounce) const {
	ERR_FAIL_INDEX_V(p_bounce, motion_results.size(), PhysicsServer3D::MotionResult());
	return motion_results[p_bounce];
}

Ref<KinematicCollision3D> NCharacter3D::_get_slide_collision(int p_bounce) {
	ERR_FAIL_INDEX_V(p_bounce, motion_results.size(), Ref<KinematicCollision3D>());
	if (p_bounce >= slide_colliders.size()) {
		slide_colliders.resize(p_bounce + 1);
	}

	// Create a new instance when the cached reference is invalid or still in use in script.
	if (slide_colliders[p_bounce].is_null() || slide_colliders[p_bounce]->get_reference_count() > 1) {
		slide_colliders.write[p_bounce].instantiate();
		slide_colliders.write[p_bounce]->owner_id = get_instance_id();
	}

	slide_colliders.write[p_bounce]->result = motion_results[p_bounce];
	return slide_colliders[p_bounce];
}

Ref<KinematicCollision3D> NCharacter3D::_get_last_slide_collision() {
	if (motion_results.size() == 0) {
		return Ref<KinematicCollision3D>();
	}
	return _get_slide_collision(motion_results.size() - 1);
}

bool NCharacter3D::is_floor_stop_on_slope_enabled() const {
	return floor_stop_on_slope;
}

void NCharacter3D::set_floor_stop_on_slope_enabled(bool p_enabled) {
	floor_stop_on_slope = p_enabled;
}

bool NCharacter3D::is_floor_constant_speed_enabled() const {
	return floor_constant_speed;
}

void NCharacter3D::set_floor_constant_speed_enabled(bool p_enabled) {
	floor_constant_speed = p_enabled;
}

bool NCharacter3D::is_floor_block_on_wall_enabled() const {
	return floor_block_on_wall;
}

void NCharacter3D::set_floor_block_on_wall_enabled(bool p_enabled) {
	floor_block_on_wall = p_enabled;
}

bool NCharacter3D::is_slide_on_ceiling_enabled() const {
	return slide_on_ceiling;
}

void NCharacter3D::set_slide_on_ceiling_enabled(bool p_enabled) {
	slide_on_ceiling = p_enabled;
}

uint32_t NCharacter3D::get_platform_floor_layers() const {
	return platform_floor_layers;
}

void NCharacter3D::set_platform_floor_layers(uint32_t p_exclude_layers) {
	platform_floor_layers = p_exclude_layers;
}

uint32_t NCharacter3D::get_platform_wall_layers() const {
	return platform_wall_layers;
}

void NCharacter3D::set_platform_wall_layers(uint32_t p_exclude_layers) {
	platform_wall_layers = p_exclude_layers;
}

void NCharacter3D::set_motion_mode(MotionMode p_mode) {
	motion_mode = p_mode;
}

NCharacter3D::MotionMode NCharacter3D::get_motion_mode() const {
	return motion_mode;
}

void NCharacter3D::set_platform_on_leave(PlatformOnLeave p_on_leave_apply_velocity) {
	platform_on_leave = p_on_leave_apply_velocity;
}

NCharacter3D::PlatformOnLeave NCharacter3D::get_platform_on_leave() const {
	return platform_on_leave;
}

int NCharacter3D::get_max_slides() const {
	return max_slides;
}

void NCharacter3D::set_max_slides(int p_max_slides) {
	ERR_FAIL_COND(p_max_slides < 1);
	max_slides = p_max_slides;
}

real_t NCharacter3D::get_floor_max_angle() const {
	return floor_max_angle;
}

void NCharacter3D::set_floor_max_angle(real_t p_radians) {
	floor_max_angle = p_radians;
}

real_t NCharacter3D::get_floor_snap_length() {
	return floor_snap_length;
}

void NCharacter3D::set_floor_snap_length(real_t p_floor_snap_length) {
	ERR_FAIL_COND(p_floor_snap_length < 0);
	floor_snap_length = p_floor_snap_length;
}

real_t NCharacter3D::get_wall_min_slide_angle() const {
	return wall_min_slide_angle;
}

void NCharacter3D::set_wall_min_slide_angle(real_t p_radians) {
	wall_min_slide_angle = p_radians;
}

const Vector3 &NCharacter3D::get_up_direction() const {
	return up_direction;
}

void NCharacter3D::set_up_direction(const Vector3 &p_up_direction) {
	ERR_FAIL_COND_MSG(p_up_direction == Vector3(), "up_direction can't be equal to Vector3.ZERO, consider using Floating motion mode instead.");
	up_direction = p_up_direction.normalized();
}

Vector3 NCharacter3D::get_capsule_base() {
	// get all shapes
	real_t max_radius = 0.0f;
	List<uint32_t> shape_owners;
	get_shape_owners(&shape_owners);
	for (uint32_t shape_owner : shape_owners) {
		for (int i = 0; i < shape_owner_get_shape_count(shape_owner); i++) {
			Ref<Shape3D> shape_ref = shape_owner_get_shape(shape_owner, i);
			real_t shape_radius = shape_ref.ptr()->get_enclosing_radius();
			max_radius = MAX(shape_radius, max_radius);
		}
	}

	return get_global_transform().origin + Vector3(0, -max_radius, 0);
}

void NCharacter3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			// Reset move_and_slide() data.
			collision_state.state = 0;
			platform_rid = RID();
			platform_object_id = ObjectID();
			motion_results.clear();
			platform_velocity = Vector3();
			platform_angular_velocity = Vector3();
		} break;
	}
}

void NCharacter3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("move_and_slide"), &NCharacter3D::move_and_slide);
	ClassDB::bind_method(D_METHOD("apply_floor_snap"), &NCharacter3D::apply_floor_snap);

	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &NCharacter3D::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &NCharacter3D::get_velocity);

	ClassDB::bind_method(D_METHOD("set_safe_margin", "margin"), &NCharacter3D::set_safe_margin);
	ClassDB::bind_method(D_METHOD("get_safe_margin"), &NCharacter3D::get_safe_margin);
	ClassDB::bind_method(D_METHOD("is_floor_stop_on_slope_enabled"), &NCharacter3D::is_floor_stop_on_slope_enabled);
	ClassDB::bind_method(D_METHOD("set_floor_stop_on_slope_enabled", "enabled"), &NCharacter3D::set_floor_stop_on_slope_enabled);
	ClassDB::bind_method(D_METHOD("set_floor_constant_speed_enabled", "enabled"), &NCharacter3D::set_floor_constant_speed_enabled);
	ClassDB::bind_method(D_METHOD("is_floor_constant_speed_enabled"), &NCharacter3D::is_floor_constant_speed_enabled);
	ClassDB::bind_method(D_METHOD("set_floor_block_on_wall_enabled", "enabled"), &NCharacter3D::set_floor_block_on_wall_enabled);
	ClassDB::bind_method(D_METHOD("is_floor_block_on_wall_enabled"), &NCharacter3D::is_floor_block_on_wall_enabled);
	ClassDB::bind_method(D_METHOD("set_slide_on_ceiling_enabled", "enabled"), &NCharacter3D::set_slide_on_ceiling_enabled);
	ClassDB::bind_method(D_METHOD("is_slide_on_ceiling_enabled"), &NCharacter3D::is_slide_on_ceiling_enabled);

	ClassDB::bind_method(D_METHOD("set_platform_floor_layers", "exclude_layer"), &NCharacter3D::set_platform_floor_layers);
	ClassDB::bind_method(D_METHOD("get_platform_floor_layers"), &NCharacter3D::get_platform_floor_layers);
	ClassDB::bind_method(D_METHOD("set_platform_wall_layers", "exclude_layer"), &NCharacter3D::set_platform_wall_layers);
	ClassDB::bind_method(D_METHOD("get_platform_wall_layers"), &NCharacter3D::get_platform_wall_layers);

	ClassDB::bind_method(D_METHOD("get_max_slides"), &NCharacter3D::get_max_slides);
	ClassDB::bind_method(D_METHOD("set_max_slides", "max_slides"), &NCharacter3D::set_max_slides);
	ClassDB::bind_method(D_METHOD("get_floor_max_angle"), &NCharacter3D::get_floor_max_angle);
	ClassDB::bind_method(D_METHOD("set_floor_max_angle", "radians"), &NCharacter3D::set_floor_max_angle);
	ClassDB::bind_method(D_METHOD("get_floor_snap_length"), &NCharacter3D::get_floor_snap_length);
	ClassDB::bind_method(D_METHOD("set_floor_snap_length", "floor_snap_length"), &NCharacter3D::set_floor_snap_length);

	ClassDB::bind_method(D_METHOD("get_step_enabled"), &NCharacter3D::get_step_enabled);
	ClassDB::bind_method(D_METHOD("set_step_enabled", "step_enabled"), &NCharacter3D::set_step_enabled);
	ClassDB::bind_method(D_METHOD("get_max_step_height"), &NCharacter3D::get_max_step_height);
	ClassDB::bind_method(D_METHOD("set_max_step_height", "max_step_height"), &NCharacter3D::set_max_step_height);
	ClassDB::bind_method(D_METHOD("get_max_step_lookahead"), &NCharacter3D::get_max_step_lookahead);
	ClassDB::bind_method(D_METHOD("set_max_step_lookahead", "max_step_lookahead"), &NCharacter3D::set_max_step_lookahead);

	ClassDB::bind_method(D_METHOD("get_wall_min_slide_angle"), &NCharacter3D::get_wall_min_slide_angle);
	ClassDB::bind_method(D_METHOD("set_wall_min_slide_angle", "radians"), &NCharacter3D::set_wall_min_slide_angle);
	ClassDB::bind_method(D_METHOD("get_up_direction"), &NCharacter3D::get_up_direction);
	ClassDB::bind_method(D_METHOD("set_up_direction", "up_direction"), &NCharacter3D::set_up_direction);
	ClassDB::bind_method(D_METHOD("set_motion_mode", "mode"), &NCharacter3D::set_motion_mode);
	ClassDB::bind_method(D_METHOD("get_motion_mode"), &NCharacter3D::get_motion_mode);
	ClassDB::bind_method(D_METHOD("set_platform_on_leave", "on_leave_apply_velocity"), &NCharacter3D::set_platform_on_leave);
	ClassDB::bind_method(D_METHOD("get_platform_on_leave"), &NCharacter3D::get_platform_on_leave);

	ClassDB::bind_method(D_METHOD("is_on_floor"), &NCharacter3D::is_on_floor);
	ClassDB::bind_method(D_METHOD("is_on_floor_only"), &NCharacter3D::is_on_floor_only);
	ClassDB::bind_method(D_METHOD("is_on_ceiling"), &NCharacter3D::is_on_ceiling);
	ClassDB::bind_method(D_METHOD("is_on_ceiling_only"), &NCharacter3D::is_on_ceiling_only);
	ClassDB::bind_method(D_METHOD("is_on_wall"), &NCharacter3D::is_on_wall);
	ClassDB::bind_method(D_METHOD("is_on_wall_only"), &NCharacter3D::is_on_wall_only);
	ClassDB::bind_method(D_METHOD("get_floor_normal"), &NCharacter3D::get_floor_normal);
	ClassDB::bind_method(D_METHOD("get_wall_normal"), &NCharacter3D::get_wall_normal);
	ClassDB::bind_method(D_METHOD("get_last_motion"), &NCharacter3D::get_last_motion);
	ClassDB::bind_method(D_METHOD("get_position_delta"), &NCharacter3D::get_position_delta);
	ClassDB::bind_method(D_METHOD("get_real_velocity"), &NCharacter3D::get_real_velocity);
	ClassDB::bind_method(D_METHOD("get_floor_angle", "up_direction"), &NCharacter3D::get_floor_angle, DEFVAL(Vector3(0.0, 1.0, 0.0)));
	ClassDB::bind_method(D_METHOD("get_platform_velocity"), &NCharacter3D::get_platform_velocity);
	ClassDB::bind_method(D_METHOD("get_platform_angular_velocity"), &NCharacter3D::get_platform_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_slide_collision_count"), &NCharacter3D::get_slide_collision_count);
	ClassDB::bind_method(D_METHOD("get_slide_collision", "slide_idx"), &NCharacter3D::_get_slide_collision);
	ClassDB::bind_method(D_METHOD("get_last_slide_collision"), &NCharacter3D::_get_last_slide_collision);

	ClassDB::bind_method(D_METHOD("get_capsule_base"), &NCharacter3D::get_capsule_base);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "motion_mode", PROPERTY_HINT_ENUM, "Grounded,Floating", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_UPDATE_ALL_IF_MODIFIED), "set_motion_mode", "get_motion_mode");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "up_direction"), "set_up_direction", "get_up_direction");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "slide_on_ceiling"), "set_slide_on_ceiling_enabled", "is_slide_on_ceiling_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity", PROPERTY_HINT_NONE, "suffix:m/s", PROPERTY_USAGE_NO_EDITOR), "set_velocity", "get_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_slides", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_max_slides", "get_max_slides");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wall_min_slide_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians", PROPERTY_USAGE_DEFAULT), "set_wall_min_slide_angle", "get_wall_min_slide_angle");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "step_enabled"), "set_step_enabled", "get_step_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_step_height", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater,suffix:m"), "set_max_step_height", "get_max_step_height");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_step_lookahead", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater,suffix:m"), "set_max_step_lookahead", "get_max_step_lookahead");

	ADD_GROUP("Floor", "floor_");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_stop_on_slope"), "set_floor_stop_on_slope_enabled", "is_floor_stop_on_slope_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_constant_speed"), "set_floor_constant_speed_enabled", "is_floor_constant_speed_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floor_block_on_wall"), "set_floor_block_on_wall_enabled", "is_floor_block_on_wall_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "floor_max_angle", PROPERTY_HINT_RANGE, "0,180,0.1,radians"), "set_floor_max_angle", "get_floor_max_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "floor_snap_length", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater,suffix:m"), "set_floor_snap_length", "get_floor_snap_length");

	ADD_GROUP("Moving Platform", "platform_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "platform_on_leave", PROPERTY_HINT_ENUM, "Add Velocity,Add Upward Velocity,Do Nothing", PROPERTY_USAGE_DEFAULT), "set_platform_on_leave", "get_platform_on_leave");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "platform_floor_layers", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_platform_floor_layers", "get_platform_floor_layers");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "platform_wall_layers", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_platform_wall_layers", "get_platform_wall_layers");

	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "safe_margin", PROPERTY_HINT_RANGE, "0.001,256,0.001,suffix:m"), "set_safe_margin", "get_safe_margin");

	BIND_ENUM_CONSTANT(MOTION_MODE_GROUNDED);
	BIND_ENUM_CONSTANT(MOTION_MODE_FLOATING);

	BIND_ENUM_CONSTANT(PLATFORM_ON_LEAVE_ADD_VELOCITY);
	BIND_ENUM_CONSTANT(PLATFORM_ON_LEAVE_ADD_UPWARD_VELOCITY);
	BIND_ENUM_CONSTANT(PLATFORM_ON_LEAVE_DO_NOTHING);
}

void NCharacter3D::_validate_property(PropertyInfo &p_property) const {
	if (motion_mode == MOTION_MODE_FLOATING) {
		if (p_property.name.begins_with("floor_") || p_property.name == "up_direction" || p_property.name == "slide_on_ceiling") {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	}
}

NCharacter3D::NCharacter3D() :
		PhysicsBody3D(PhysicsServer3D::BODY_MODE_KINEMATIC) {
}
