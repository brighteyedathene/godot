
#include "n_animation_player.h"

#include "core/config/engine.h"
#include "core/math/geometry_2d.h"

// helper function for advancing animations and getting back progress fraction
float calculate_animation_progress_unclamped(float current_progress, float animation_length, float animation_speed, float delta)
{
	if (Animation::is_less_approx(animation_length, 0.01))
	{
		return 0.0f;
	}
	float current_position = current_progress * animation_length;
	float next_position = current_position + delta * animation_speed;
	return next_position / animation_length;
}

void NAnimationFilter::apply_filter(NAnimationPlayer& nanim_player, float amount, Vector<NAnimationInstance>& source_instances, Vector<NAnimationInstance>& target_instances)
{

	// if there's a filter, i must only weigh the tracks it filters
	// for each track in the filter (whitelist), I multiply weight by inverse ofr blend_amount
	float source_total_weight = 0.f;

	float source_multiplier = 1.0 - amount;

	for (int i = 0; i < source_instances.size(); i++)
	{
		NAnimationInstance& ai = source_instances.ptrw()[i];
		Ref<Animation> animation = nanim_player.get_animation(ai.anim_name);
		if (filter_type == FilterType::NORMALIZE)
		{
			source_total_weight += ai.pi.weight;
			continue; // continue without resizing? i think this is causing a problem...
		}

		// skip track weights if there's no filter because i'm doing something wrong with them
		if (filter_type == FilterType::BLEND && filter_map.is_empty())
		{
			if (filter_type == FilterType::BLEND)
			{
				ai.pi.weight *= source_multiplier;
				if (source_multiplier < 0.5)
				{
					ai.pi.mute_method_tracks = true;
				}
			}
			else if (filter_type == FilterType::ADD)
			{
				// do nothing to source if we're adding
			}
			continue;
		}

		ai.pi.track_weights.resize(animation->get_track_count());
		real_t* blendw = ai.pi.track_weights.ptrw();

		for (int j = 0; j < animation->get_track_count(); j++)
		{
			NodePath track_path = animation->track_get_path(j);
			bool track_filtered = filter_map.has(track_path);

			switch (filter_type)
			{
			case FilterType::BLEND:
				if (track_filtered)
				{
					blendw[j] = (1.0 - amount);
				}
				else
				{
					blendw[j] = 1.0;
				}
				break;

			case FilterType::ADD:
				// for add, source is always max weight?
				blendw[j] = 1.0;
				break;

			case FilterType::NORMALIZE:
				// wtf do i do here? need max weight or somehting...
				break;
			}
		}
	}

	// now I filter the target instances?
	float target_total_weight = 0.f;
	for (int i = 0; i < target_instances.size(); i++)
	{
		NAnimationInstance& ai = target_instances.ptrw()[i];
		Ref<Animation> animation = nanim_player.get_animation(ai.anim_name);
		if (filter_type == FilterType::NORMALIZE)
		{
			target_total_weight += ai.pi.weight;
			continue;
		}

		// skip track weights if there's no filter because i'm doing something wrong with them
		if (filter_map.is_empty())
		{
			if (filter_type == FilterType::BLEND)
			{
				ai.pi.weight *= amount;
				if (amount < 0.5)
				{
					ai.pi.mute_method_tracks = true;
				}
			}
			else if (filter_type == FilterType::ADD)
			{
				// multiply by add amount if adding? makes sense I think...
				ai.pi.weight *= amount;
			}
			continue;
		}

		ai.pi.track_weights.resize(animation->get_track_count());
		real_t* blendw = ai.pi.track_weights.ptrw();

		for (int j = 0; j < animation->get_track_count(); j++)
		{
			NodePath track_path = animation->track_get_path(j);
			bool track_filtered = filter_map.has(track_path);

			switch (filter_type)
			{
			case FilterType::BLEND:
				if (track_filtered)
				{
					blendw[j] = amount;
				}
				else
				{
					blendw[j] = 0.0;
				}
				break;

			case FilterType::ADD:
				if (track_filtered)
				{
					// for add, target is always amount
					blendw[j] = amount;
				}
				else
				{
					// otherwise they get no weight
					blendw[j] = 0.0;
				}
				break;

			case FilterType::NORMALIZE:
				// wtf do i do here? need max weight or somehting...
				break;
			}
		}
	}

	// this is nonsense it'll never work unless i fix it.
	if (filter_type == FilterType::NORMALIZE)
	{

		//if (Math::is_equal_approx(source_total_weight, 1.f) == false)
		//{
		//	print_line("trying to normalize when source weight is not 1.0 i didnt figure out what to do for that yet ");
		//}
		// leave target weight alone.
		// change source weights so that their sum = 1-target_weight
		float multiplier = CLAMP(1.0f - target_total_weight, 0.f, 1.f);

		for (int i = 0; i < source_instances.size(); i++)
		{
			NAnimationInstance& ai = source_instances.ptrw()[i];
			Ref<Animation> animation = nanim_player.get_animation(ai.anim_name);
			ai.pi.weight *= multiplier;
		}

	}

}


void NAnimationAsset_Locomotion::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_animation", "animation"), &NAnimationAsset_Locomotion::set_animation);
	ClassDB::bind_method(D_METHOD("get_animation"), &NAnimationAsset_Locomotion::get_animation);
	ClassDB::bind_method(D_METHOD("set_animation_speed_scale", "animation_speed_scale"), &NAnimationAsset_Locomotion::set_animation_speed_scale);
	ClassDB::bind_method(D_METHOD("get_animation_speed_scale"), &NAnimationAsset_Locomotion::get_animation_speed_scale);
	ClassDB::bind_method(D_METHOD("set_animation_base_speed", "animation_base_speed"), &NAnimationAsset_Locomotion::set_animation_base_speed);
	ClassDB::bind_method(D_METHOD("get_animation_base_speed"), &NAnimationAsset_Locomotion::get_animation_base_speed);

	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "animation", PROPERTY_HINT_NONE, ""), "set_animation", "get_animation");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "animation_speed_scale"), "set_animation_speed_scale", "get_animation_speed_scale");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "animation_base_speed"), "set_animation_base_speed", "get_animation_base_speed");
}

void NAnimationAsset_Locomotion1D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_blend_value", "blend_value"), &NAnimationAsset_Locomotion1D::set_blend_value);
	ClassDB::bind_method(D_METHOD("get_blend_value"), &NAnimationAsset_Locomotion1D::get_blend_value);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_value"), "set_blend_value", "get_blend_value");
}

void NAnimationAsset_Locomotion2D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_blend_position", "blend_position"), &NAnimationAsset_Locomotion2D::set_blend_position);
	ClassDB::bind_method(D_METHOD("get_blend_position"), &NAnimationAsset_Locomotion2D::get_blend_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "blend_position"), "set_blend_position", "get_blend_position");
}


void NAnimationMode::_bind_methods()
{
	
}

void NAnimationMode_OneShot::process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances)
{
	// find out if i just triggered the animation
	Variant triggered_value = nanim_player.parameters.get(triggered_key, triggered);
	if (triggered_value.get_type() == Variant::BOOL)
	{
		triggered = triggered_value;
	}
	else
	{
		print_line("NAnimationMode_Blend::process_animation_mode blend_amount_key wasn't a bool ", triggered_key);
	}

	// i might also want to change the settings on this node.
	// to avoid having to set up keys for every single property, i just set a key for a dictionary, and the rest of the shit can live in there.
	// hate instantiating new dics.
	Dictionary one_shot_params = nanim_player.parameters.get(one_shot_params_key, Dictionary());
	if(one_shot_params.is_empty() == false)
	{
		// incredibly unsafe access. whatever.
		// this should all be set when i trigger the animation, not every update...
		anim_name = one_shot_params.get("anim_name", anim_name);
		blend_in_time = one_shot_params.get("blend_in_time", blend_in_time);
		blend_out_time = one_shot_params.get("blend_out_time", blend_out_time);
		duration = one_shot_params.get("duration", duration);
		looping = one_shot_params.get("looping", looping);
		auto_blend_out = one_shot_params.get("auto_blend_out", auto_blend_out);
		freeze = one_shot_params.get("freeze", freeze);
	}

	// now I act on the trigger
	if (triggered)
	{
		timer = 0.f;
		progress_fraction = 0.f;
		triggered = false;
		// must consume the triggered key in params as well i think
		nanim_player.parameters.erase(triggered_key);
	}

	// should be straghtforward i think... 
	// If i'm near the start of the animation, i blend in
	// if i'm near the end, i blend out
	// progress normally

	// advance the animation

	Ref<Animation> animation = nanim_player.get_animation(anim_name);
	if (!animation.is_valid())
	{
		if (anim_name != StringName("")) {
			print_line("cant find animation named ", anim_name);
		}
		return;
	}

	float animation_length = animation->get_length();
	float anim_speed = freeze ? 0.f : 1.f;

	// advance the timer, but i think this is only for auto_blend_out && looping
	timer += delta * anim_speed;

	// calculating progress as a fraction, just because i do it for the others
	float current_progress = progress_fraction;
	float next_progress = calculate_animation_progress_unclamped(current_progress, animation_length, anim_speed, delta);

	// loop or clamp next_progress, and set the flag
	Animation::LoopedFlag looped_flag = Animation::LOOPED_FLAG_NONE;
	if (looping)
	{
		if (Animation::is_greater_approx(next_progress, 1.0) && Animation::is_less_or_equal_approx(current_progress, 1.0))
		{
			looped_flag = Animation::LOOPED_FLAG_END;
		}
		next_progress = Math::fposmod(next_progress, 1.f);
	}
	else
	{
		next_progress = MIN(1.0, next_progress);
	}

	// calculate the actual position and delta for pi, using clamped/wrapped progress fractions
	float next_position = next_progress * animation_length;
	float current_position = current_progress * animation_length;
	float anim_delta = (next_position > current_position) ?
		next_position - current_position :
		(next_position + animation_length) - current_position;

	// calculate blend weights
	float blend_in_weight = Math::remap(timer, 0, blend_in_time, 0.f, 1.f);
	blend_in_weight = CLAMP(blend_in_weight, 0.f, 1.f);
	float blend_out_weight = 1.f;
	if (auto_blend_out)
	{
		float end_time = looping ? duration : animation_length;
		float blend_out_start = end_time - blend_out_time;
		blend_out_weight = Math::remap(timer, blend_out_start, end_time, 1.f, 0.f);
		blend_out_weight = CLAMP(blend_out_weight, 0.f, 1.f);
	}
	float combined_blend = MIN(blend_in_weight, blend_out_weight);

	AnimationMixer::PlaybackInfo pi;
	pi.time = next_position;
	pi.weight = combined_blend;
	pi.delta = anim_delta;
	pi.looped_flag = looped_flag;
	pi.is_external_seeking = false;

	pi.start = 0.f;
	pi.end = animation_length;
	pi.seeked = false;

	// must mute any oneshots that are frozen (i probably just use them for blending out)
	pi.mute_method_tracks = freeze;

	NAnimationInstance ai;
	ai.pi = pi;
	ai.anim_name = anim_name;
	out_instances.push_back(ai);

	// update the progress i suppose
	progress_fraction = next_progress;
	
}

void NAnimationMode_OneShot::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_triggered_key", "triggered_key"), &NAnimationMode_OneShot::set_triggered_key);
	ClassDB::bind_method(D_METHOD("get_triggered_key"), &NAnimationMode_OneShot::get_triggered_key);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "triggered_key"), "set_triggered_key", "get_triggered_key");

	ClassDB::bind_method(D_METHOD("set_one_shot_params_key", "one_shot_params_key"), &NAnimationMode_OneShot::set_one_shot_params_key);
	ClassDB::bind_method(D_METHOD("get_one_shot_params_key"), &NAnimationMode_OneShot::get_one_shot_params_key);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "one_shot_params_key"), "set_one_shot_params_key", "get_one_shot_params_key");

	ClassDB::bind_method(D_METHOD("set_anim_name", "anim_name"), &NAnimationMode_OneShot::set_anim_name);
	ClassDB::bind_method(D_METHOD("get_anim_name"), &NAnimationMode_OneShot::get_anim_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "anim_name"), "set_anim_name", "get_anim_name");

	ClassDB::bind_method(D_METHOD("set_blend_in_time", "blend_in_time"), &NAnimationMode_OneShot::set_blend_in_time);
	ClassDB::bind_method(D_METHOD("get_blend_in_time"), &NAnimationMode_OneShot::get_blend_in_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_in_time"), "set_blend_in_time", "get_blend_in_time");

	ClassDB::bind_method(D_METHOD("set_blend_out_time", "blend_out_time"), &NAnimationMode_OneShot::set_blend_out_time);
	ClassDB::bind_method(D_METHOD("get_blend_out_time"), &NAnimationMode_OneShot::get_blend_out_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_out_time"), "set_blend_out_time", "get_blend_out_time");

	ClassDB::bind_method(D_METHOD("set_duration", "duration"), &NAnimationMode_OneShot::set_duration);
	ClassDB::bind_method(D_METHOD("get_duration"), &NAnimationMode_OneShot::get_duration);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "duration"), "set_duration", "get_duration");


	ClassDB::bind_method(D_METHOD("set_looping", "looping"), &NAnimationMode_OneShot::set_looping);
	ClassDB::bind_method(D_METHOD("get_looping"), &NAnimationMode_OneShot::get_looping);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "looping"), "set_looping", "get_looping");

	ClassDB::bind_method(D_METHOD("set_auto_blend_out", "auto_blend_out"), &NAnimationMode_OneShot::set_auto_blend_out);
	ClassDB::bind_method(D_METHOD("get_auto_blend_out"), &NAnimationMode_OneShot::get_auto_blend_out);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_blend_out"), "set_auto_blend_out", "get_auto_blend_out");

	ClassDB::bind_method(D_METHOD("set_freeze", "freeze"), &NAnimationMode_OneShot::set_freeze);
	ClassDB::bind_method(D_METHOD("get_freeze"), &NAnimationMode_OneShot::get_freeze);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "freeze"), "set_freeze", "get_freeze");

}

void NAnimationMode_Blend::process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances)
{
	Variant parameter_value = nanim_player.parameters.get(blend_amount_key, blend_amount);
	if (parameter_value.get_type() == Variant::FLOAT)
	{
		blend_amount = parameter_value;
	}
	else
	{
		print_line("NAnimationMode_Blend::process_animation_mode blend_amount_key wasn't a float ", blend_amount_key);
	}

	Vector<NAnimationInstance> source_instances = {};
	if (source_mode.is_valid())
	{
		source_mode->process_animation_mode(nanim_player, delta, source_instances);
	}

	Vector<NAnimationInstance> target_instances = {};
	if (target_mode.is_valid())
	{
		target_mode->process_animation_mode(nanim_player, delta, target_instances);
	}

	filter.apply_filter(nanim_player, blend_amount, source_instances, target_instances);

	out_instances.append_array(source_instances);
	out_instances.append_array(target_instances);
}

void NAnimationMode_Blend::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_blend_amount", "blend_amount"), &NAnimationMode_Blend::set_blend_amount);
	ClassDB::bind_method(D_METHOD("get_blend_amount"), &NAnimationMode_Blend::get_blend_amount);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_amount"), "set_blend_amount", "get_blend_amount");

	ClassDB::bind_method(D_METHOD("set_blend_amount_key", "blend_amount_key"), &NAnimationMode_Blend::set_blend_amount_key);
	ClassDB::bind_method(D_METHOD("get_blend_amount_key"), &NAnimationMode_Blend::get_blend_amount_key);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "blend_amount_key"), "set_blend_amount_key", "get_blend_amount_key");

	ClassDB::bind_method(D_METHOD("set_source_mode", "source_mode"), &NAnimationMode_Blend::set_source_mode);
	ClassDB::bind_method(D_METHOD("get_source_mode"), &NAnimationMode_Blend::get_source_mode);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "source_mode", PROPERTY_HINT_RESOURCE_TYPE, "NAnimationMode"), "set_source_mode", "get_source_mode");

	ClassDB::bind_method(D_METHOD("set_target_mode", "target_mode"), &NAnimationMode_Blend::set_target_mode);
	ClassDB::bind_method(D_METHOD("get_target_mode"), &NAnimationMode_Blend::get_target_mode);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "target_mode", PROPERTY_HINT_RESOURCE_TYPE, "NAnimationMode"), "set_target_mode", "get_target_mode");

	ClassDB::bind_method(D_METHOD("set_filtered_tracks", "filtered_tracks"), &NAnimationMode_Blend::set_filtered_tracks);
	ClassDB::bind_method(D_METHOD("get_filtered_tracks"), &NAnimationMode_Blend::get_filtered_tracks);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "filtered_tracks", PROPERTY_HINT_TYPE_STRING), "set_filtered_tracks", "get_filtered_tracks");

	ClassDB::bind_method(D_METHOD("set_filter_type", "filter_type"), &NAnimationMode_Blend::set_filter_type);
	ClassDB::bind_method(D_METHOD("get_filter_type"), &NAnimationMode_Blend::get_filter_type);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "filter_type", PROPERTY_HINT_ENUM, "BLEND,ADD,NORMALIZE"), "set_filter_type", "get_filter_type");
}

void NAnimationMode_Locomotion1D::process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances)
{
	float blend_value = 0.0f;
	Variant value = nanim_player.parameters.get(blend_value_key, 0.f);
	if (value.get_type() == Variant::FLOAT)
	{
		blend_value = value;
	}
	else
	{
		print_line("NAnimationMode_Locomotion1D::process_animation_mode blend_value_key wasn't a float ", blend_value_key);
	}

	// find best assts to blend
	int lower_index = -1;
	int upper_index = -1;
	float lower_value = 0.f;
	float upper_value = 0.f;
	for (int i = 0; i < assets.size(); i++)
	{
		Ref<NAnimationAsset_Locomotion1D> asset = assets[i];
		if (asset->blend_value <= blend_value)
		{
			if (lower_index == -1 || asset->blend_value > lower_value)
			{
				lower_index = i;
				lower_value = asset->blend_value;
			}
		}
		else
		{
			if (upper_index == -1 || asset->blend_value < upper_value)
			{
				upper_index = i;
				upper_value = asset->blend_value;
			}
		}
	}

	float upper_weight = 0.0f;
	float lower_weight = 0.0f;

	if (lower_index == -1 && upper_index != -1)
	{
		// we are on the left side, no other point to the left
		// we just play the next point.
		upper_weight = 1.0;
	}
	else if (upper_index == -1)
	{
		// we are on the right side, no other point to the right
		// we just play the previous point
		lower_weight = 1.0;
	}
	else
	{
		// we are between two points.
		// figure out weights, then blend the animations
		float distance_between_points = upper_value - lower_value;
		float current_pos_inbetween = blend_value - lower_value;
		float blend_percentage = current_pos_inbetween / distance_between_points;
		float blend_lower = 1.0 - blend_percentage;
		float blend_higher = blend_percentage;

		lower_weight = blend_lower;
		upper_weight = blend_higher;
	}

	// only leader gets to call methods
	bool upper_leads = upper_weight > lower_weight;

	// cache this so i can calculate current_position for each animation, and use that to calculate delta
	float current_progress = progress_fraction;

	// calculate progress fractions
	float lower_progress = 0.0f;
	if (lower_index != -1)
	{
		Ref<NAnimationAsset_Locomotion> asset = assets[lower_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		lower_progress = calculate_animation_progress_unclamped(current_progress, animation_length, asset->animation_speed_scale, delta);
	}
	float upper_progress = 0.0f;
	if (upper_index != -1)
	{
		Ref<NAnimationAsset_Locomotion> asset = assets[upper_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		upper_progress = calculate_animation_progress_unclamped(current_progress, animation_length, asset->animation_speed_scale, delta);
	}
	// since progress values are unclamped, they could be greater than 1.0
	float lerped_progress = Math::lerp(lower_progress, upper_progress, upper_weight);

	// update looped flag based on the lerped progress, since that's the value we'll use to find next position
	Animation::LoopedFlag looped_flag = Animation::LOOPED_FLAG_NONE;
	if (Animation::is_less_approx(lerped_progress, 0) && Animation::is_greater_or_equal_approx(current_progress, 0))
	{
		looped_flag = Animation::LOOPED_FLAG_START;
	}
	if (Animation::is_greater_approx(lerped_progress, 1.0) && Animation::is_less_or_equal_approx(current_progress, 1.0))
	{
		looped_flag = Animation::LOOPED_FLAG_END;
	}
	// now i can wrap around 1.0, because its just fraction
	lerped_progress = Math::fposmod(lerped_progress, 1.f);

	// I think I have everything. now make anim instances...
	if (lower_index != -1 && lower_weight > 0.0f)
	{
		Ref<NAnimationAsset_Locomotion> asset = assets[lower_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		float current_position = current_progress * animation_length;
		float next_position = lerped_progress * animation_length;

		// since we could have wrapped around to the start, maybe add anim length before calculating delta
		float anim_delta = (next_position > current_position) ?
			next_position - current_position :
			(next_position + animation_length) - current_position;

		AnimationMixer::PlaybackInfo pi;
		pi.time = next_position;
		pi.weight = lower_weight;// * weight; // limit by the given weight later!
		pi.delta = anim_delta;
		pi.looped_flag = looped_flag;
		pi.is_external_seeking = false;

		pi.start = 0.f;
		pi.end = animation_length;
		pi.seeked = false;

		// mute lower methods if upper is heavier
		pi.mute_method_tracks = upper_leads;

		NAnimationInstance ai;
		ai.pi = pi;
		ai.anim_name = asset->animation;
		out_instances.push_back(ai);

		//nanim_player.make_animation_instance(asset->animation, pi);
	}

	if (upper_index != -1 && upper_weight > 0.0f)
	{
		Ref<NAnimationAsset_Locomotion> asset = assets[upper_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		float current_position = current_progress * animation_length;
		float next_position = lerped_progress * animation_length;

		// since we could have wrapped around to the start, maybe add anim length before calculating delta
		float anim_delta = (next_position > current_position) ?
			next_position - current_position :
			(next_position + animation_length) - current_position;

		AnimationMixer::PlaybackInfo pi;
		pi.time = next_position;
		pi.weight = upper_weight;// * weight; // limit by the given weight
		pi.delta = anim_delta;
		pi.looped_flag = looped_flag;
		pi.is_external_seeking = false;

		pi.start = 0.f;
		pi.end = animation_length;
		pi.seeked = false;

		// mute upper methods if upper is not heavier
		pi.mute_method_tracks = upper_leads == false;

		NAnimationInstance ai;
		ai.pi = pi;
		ai.anim_name = asset->animation;
		out_instances.push_back(ai);

		//nanim_player.make_animation_instance(asset->animation, pi);
	}

	progress_fraction = isnan(lerped_progress) ? 0 : lerped_progress;
}

void NAnimationMode_Locomotion1D::_bind_methods()
{

	ClassDB::bind_method(D_METHOD("set_blend_value_key", "blend_amount_key"), &NAnimationMode_Locomotion1D::set_blend_value_key);
	ClassDB::bind_method(D_METHOD("get_blend_value_key"), &NAnimationMode_Locomotion1D::get_blend_value_key);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "blend_value_key"), "set_blend_value_key", "get_blend_value_key");

	ClassDB::bind_method(D_METHOD("set_assets", "assets"), &NAnimationMode_Locomotion1D::set_assets);
	ClassDB::bind_method(D_METHOD("get_assets"), &NAnimationMode_Locomotion1D::get_assets);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "assets", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("NAnimationAsset_Locomotion1D")), "set_assets", "get_assets");
}

void NAnimationMode_Locomotion2D::triangulate_points()
{
	print_line("triangulating points...");

	triangles.clear();

	Vector<Vector2> points;
	points.resize(assets.size());
	for (int i = 0; i < assets.size(); i++)
	{
		Ref<NAnimationAsset_Locomotion2D> asset = assets[i];
		points.write[i] = asset->blend_position;
	}

	Vector<Delaunay2D::Triangle> tr = Delaunay2D::triangulate(points);

	for (int i = 0; i < tr.size(); i++)
	{
		add_triangle(tr[i].points[0], tr[i].points[1], tr[i].points[2]);
	}
}

void NAnimationMode_Locomotion2D::add_triangle(int p_x, int p_y, int p_z, int p_at_index)
{
	ERR_FAIL_INDEX(p_x, assets.size());
	ERR_FAIL_INDEX(p_y, assets.size());
	ERR_FAIL_INDEX(p_z, assets.size());

	BlendTriangle t;
	t.asset_indices[0] = p_x;
	t.asset_indices[1] = p_y;
	t.asset_indices[2] = p_z;

	SortArray<int> sort;
	sort.sort(t.asset_indices, 3);

	for (int i = 0; i < triangles.size(); i++)
	{
		bool all_equal = true;
		for (int j = 0; j < 3; j++)
		{
			if (triangles[i].asset_indices[j] != t.asset_indices[j])
			{
				all_equal = false;
				break;
			}
		}
		ERR_FAIL_COND(all_equal);
	}

	if (p_at_index == -1 || p_at_index == triangles.size())
	{
		triangles.push_back(t);
	}
	else
	{
		triangles.insert(p_at_index, t);
	}
}

void NAnimationMode_Locomotion2D::remove_triangle(int p_triangle)
{
	ERR_FAIL_INDEX(p_triangle, triangles.size());

	triangles.remove_at(p_triangle);
}
int NAnimationMode_Locomotion2D::get_triangle_asset_index(int p_triangle, int p_point)
{
	ERR_FAIL_INDEX_V(p_point, 3, -1);
	ERR_FAIL_INDEX_V(p_triangle, triangles.size(), -1);
	return triangles[p_triangle].asset_indices[p_point];
}


int NAnimationMode_Locomotion2D::get_triangle_count() const
{
	return triangles.size();
}

Vector2 NAnimationMode_Locomotion2D::get_closest_point(const Vector2& p_point)
{
	if (triangles.is_empty())
	{
		return Vector2();
	}

	Vector2 best_point;
	bool first = true;

	for (int i = 0; i < triangles.size(); i++)
	{
		Vector2 points[3];
		for (int j = 0; j < 3; j++)
		{
			points[j] = get_asset_blend_position(get_triangle_asset_index(i, j));
		}

		if (Geometry2D::is_point_in_triangle(p_point, points[0], points[1], points[2]))
		{
			return p_point;
		}

		for (int j = 0; j < 3; j++)
		{
			const Vector2 segment_a = points[j];
			const Vector2 segment_b = points[(j + 1) % 3];
			Vector2 closest_point = Geometry2D::get_closest_point_to_segment(p_point, segment_a, segment_b);
			if (first || closest_point.distance_to(p_point) < best_point.distance_to(p_point))
			{
				best_point = closest_point;
				first = false;
			}
		}
	}

	return best_point;
}

void NAnimationMode_Locomotion2D::calculate_triangle_weights(const Vector2& p_pos, const Vector2* p_points, float* r_weights)
{
	if (p_pos.is_equal_approx(p_points[0]))
	{
		r_weights[0] = 1;
		r_weights[1] = 0;
		r_weights[2] = 0;
		return;
	}
	if (p_pos.is_equal_approx(p_points[1]))
	{
		r_weights[0] = 0;
		r_weights[1] = 1;
		r_weights[2] = 0;
		return;
	}
	if (p_pos.is_equal_approx(p_points[2]))
	{
		r_weights[0] = 0;
		r_weights[1] = 0;
		r_weights[2] = 1;
		return;
	}

	Vector2 v0 = p_points[1] - p_points[0];
	Vector2 v1 = p_points[2] - p_points[0];
	Vector2 v2 = p_pos - p_points[0];

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);
	float denom = (d00 * d11 - d01 * d01);
	if (denom == 0)
	{
		r_weights[0] = 1;
		r_weights[1] = 0;
		r_weights[2] = 0;
		return;
	}
	float v = (d11 * d20 - d01 * d21) / denom;
	float w = (d00 * d21 - d01 * d20) / denom;
	float u = 1.0f - v - w;

	r_weights[0] = u;
	r_weights[1] = v;
	r_weights[2] = w;
}

Vector2 NAnimationMode_Locomotion2D::get_asset_blend_position(int p_index) const
{
	ERR_FAIL_INDEX_V(p_index, assets.size(), Vector2());
	Ref<NAnimationAsset_Locomotion2D> asset = assets[p_index];
	if (asset.is_valid() == false) { return Vector2(); }
	return asset->blend_position;
}

void NAnimationMode_Locomotion2D::process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances)
{
	Vector2 blend_pos = Vector2();
	Variant value = nanim_player.parameters.get(blend_position_key, Vector2());
	if (value.get_type() == Variant::VECTOR2)
	{
		blend_pos = value;
	}
	else
	{
		print_line("NAnimationMode_Locomotion2D::process_animation_mode blend_position_key wasn't a vector2 ", blend_position_key);
	}

	AnimationMixer::PlaybackInfo pi;

	if (triangles.is_empty())
	{
		print_line("NAnimationMode_Locomotion2D::process_animation_mode no triangles");
		triangulate_points();
		return;
	}

	Vector2 best_point;
	bool first = true;
	int blend_triangle = -1;
	float blend_weights[3] = { 0, 0, 0 };

	for (int i = 0; i < triangles.size(); i++)
	{
		Vector2 points[3];
		for (int j = 0; j < 3; j++)
		{
			points[j] = get_asset_blend_position(get_triangle_asset_index(i, j));
		}

		if (Geometry2D::is_point_in_triangle(blend_pos, points[0], points[1], points[2]))
		{
			blend_triangle = i;
			calculate_triangle_weights(blend_pos, points, blend_weights);
			break;
		}

		for (int j = 0; j < 3; j++)
		{
			const Vector2 segment_a = points[j];
			const Vector2 segment_b = points[(j + 1) % 3];
			Vector2 closest2 = Geometry2D::get_closest_point_to_segment(blend_pos, segment_a, segment_b);
			if (first || closest2.distance_to(blend_pos) < best_point.distance_to(blend_pos))
			{
				best_point = closest2;
				blend_triangle = i;
				first = false;
				const real_t d = segment_a.distance_to(segment_b);
				if (d == 0.0)
				{
					blend_weights[j] = 1.0;
					blend_weights[(j + 1) % 3] = 0.0;
					blend_weights[(j + 2) % 3] = 0.0;
				}
				else
				{
					const real_t c = segment_a.distance_to(closest2) / d;

					blend_weights[j] = 1.0 - c;
					blend_weights[(j + 1) % 3] = c;
					blend_weights[(j + 2) % 3] = 0.0;
				}
			}
		}
	}

	ERR_FAIL_COND_MSG(blend_triangle == -1, "sum ting wong"); //should never reach here

	int triangle_assets[3];
	for (int j = 0; j < 3; j++)
	{
		triangle_assets[j] = get_triangle_asset_index(blend_triangle, j);
	}


	// I guess I have a triangle to blend here in triangle_points
	// and I have the weights in blend_weights
	int heaviest_index = 0;
	float heaviest_weight = -1.f;
	for (int i = 0; i < 3; i++)
	{
		if (blend_weights[i] > heaviest_weight)
		{
			heaviest_weight = blend_weights[i];
			heaviest_index = i;
		}
	}
	// now that i have the heaviest index, i know which assets should mute methods
	
	// caching this so i can retroactively calculate delta for PlaybackInfos
	float current_progress = progress_fraction;
	
	// now I shall calculate the progress for each animation in the triangle, and blend between them
	float progress_sum = 0.f;
	for (int i = 0; i < 3; i++)
	{
		int asset_index = triangle_assets[i];
		Ref<NAnimationAsset_Locomotion> asset = assets[asset_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		float animation_progress = calculate_animation_progress_unclamped(current_progress, animation_length, asset->animation_speed_scale, delta);
		// in case I get nan somewhere...?
		//animation_progress = isnan(animation_progress) ? 0 : animation_progress;

		progress_sum += animation_progress * blend_weights[i];
	}
	//float lerped_progress = progress_sum / 3;
	float lerped_progress = progress_sum;
	if (isnan(lerped_progress)) {
		print_line("NAnimationMode_Locomotion2D::process_animation_mode got nan for progress!");
	}

	// update looped flag based on the lerped progress, since that's the value we'll use to find next position
	Animation::LoopedFlag looped_flag = Animation::LOOPED_FLAG_NONE;
	if (Animation::is_less_approx(lerped_progress, 0) && Animation::is_greater_or_equal_approx(current_progress, 0))
	{
		looped_flag = Animation::LOOPED_FLAG_START;
	}
	if (Animation::is_greater_approx(lerped_progress, 1.0) && Animation::is_less_or_equal_approx(current_progress, 1.0))
	{
		looped_flag = Animation::LOOPED_FLAG_END;
	}
	// now i can wrap around 1.0, because its just fraction
	lerped_progress = Math::fposmod(lerped_progress, 1.f);

	// now I make anim instances for each asset in the triangle
	for (int i = 0; i < 3; i++)
	{
		int asset_index = triangle_assets[i];
		float asset_weight = blend_weights[i];
		Ref<NAnimationAsset_Locomotion> asset = assets[asset_index];
		Ref<Animation> animation = nanim_player.get_animation(asset->animation);
		float animation_length = animation->get_length();
		float current_position = current_progress * animation_length;
		float next_position = lerped_progress * animation_length;

		// since we could have wrapped around to the start, maybe add anim length before calculating delta
		float anim_delta = (next_position > current_position) ?
			next_position - current_position :
			(next_position + animation_length) - current_position;

		AnimationMixer::PlaybackInfo pi;
		pi.time = next_position;
		pi.weight = asset_weight; // * weight; // limit by the given weight later!
		pi.delta = anim_delta;
		pi.looped_flag = looped_flag;
		pi.is_external_seeking = false;

		pi.start = 0.f;
		pi.end = animation_length;
		pi.seeked = false;

		// mute lower methods if upper is heavier
		pi.mute_method_tracks = (i != heaviest_index);

		NAnimationInstance ai;
		ai.pi = pi;
		ai.anim_name = asset->animation;
		
		out_instances.push_back(ai);
		//nanim_player.make_animation_instance(asset->animation, pi);

		// debugging shit
		const real_t *track_weights_ptr = pi.track_weights.ptr();
		int track_weights_count = pi.track_weights.size();
		Ref<Animation> a = animation;
		LocalVector<AnimationMixer::TrackCache*>& track_num_to_track_cache = nanim_player.animation_track_num_to_track_cache[a];
		const Vector<Animation::Track*> tracks = a->get_tracks();
		Animation::Track* const* tracks_ptr = tracks.ptr();
		real_t a_length = a->get_length();
		int count = tracks.size();
		for (int i = 0; i < count; i++)
		{
			const Animation::Track* animation_track = tracks_ptr[i];
			if (!animation_track->enabled)
			{
				continue;
			}
			AnimationMixer::TrackCache* track = track_num_to_track_cache[i];
			if (track == nullptr)
			{
				continue; // No path, but avoid error spamming.
			}
			int blend_idx = track->blend_idx;
			ERR_CONTINUE(blend_idx < 0 || blend_idx >= nanim_player.track_count);
			//real_t blend = blend_idx < track_weights_count ? track_weights_ptr[blend_idx] * weight : weight;
			//if (!nanim_player.deterministic)
			//{
			//	// If non-deterministic, do normalization.
			//	// It would be better to make this if statement outside the for loop, but come here since too much code...
			//	if (Math::is_zero_approx(track->total_weight))
			//	{
			//		continue;
			//	}
			//	blend = blend / track->total_weight;
			//}
			//Animation::TrackType ttype = animation_track->type;
			//track->root_motion = nanim_player.root_motion_track == animation_track->path;
		}

	}




	progress_fraction = lerped_progress;
}

void NAnimationMode_Locomotion2D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_blend_position_key", "blend_position_key"), &NAnimationMode_Locomotion2D::set_blend_position_key);
	ClassDB::bind_method(D_METHOD("get_blend_position_key"), &NAnimationMode_Locomotion2D::get_blend_position_key);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "blend_position_key"), "set_blend_position_key", "get_blend_position_key");


	ClassDB::bind_method(D_METHOD("set_assets", "assets"), &NAnimationMode_Locomotion2D::set_assets);
	ClassDB::bind_method(D_METHOD("get_assets"), &NAnimationMode_Locomotion2D::get_assets);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "assets", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("NAnimationAsset_Locomotion2D")), "set_assets", "get_assets");

	ClassDB::bind_method(D_METHOD("triangulate_points"), &NAnimationMode_Locomotion2D::triangulate_points);
	ClassDB::bind_method(D_METHOD("get_retriangulate"), &NAnimationMode_Locomotion2D::get_retriangulate);
	ClassDB::bind_method(D_METHOD("set_retriangulate", "triangulate"), &NAnimationMode_Locomotion2D::set_retriangulate);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "retriangulate"), "set_retriangulate", "get_retriangulate");
}


bool NAnimationPlayer::_set(const StringName& p_name, const Variant& p_value)
{
	String name = p_name;
	if (name.begins_with("action_playback/play"))
	{ // For backward compatibility.
		set_current_animation(p_value);
	}
	else if (name.begins_with("next/"))
	{
		String which = name.get_slicec('/', 1);
		animation_set_next(which, p_value);
	}
	else if (p_name == SceneStringName(blend_times))
	{
		Array array = p_value;
		int len = array.size();
		ERR_FAIL_COND_V(len % 3, false);

		for (int i = 0; i < len / 3; i++)
		{
			StringName from = array[i * 3 + 0];
			StringName to = array[i * 3 + 1];
			float time = array[i * 3 + 2];
			set_blend_time(from, to, time);
		}
#ifndef DISABLE_DEPRECATED
	}
	else if (p_name == "method_call_mode")
	{
		set_callback_mode_method(static_cast<AnimationCallbackModeMethod>((int)p_value));
	}
	else if (p_name == "playback_process_mode")
	{
		set_callback_mode_process(static_cast<AnimationCallbackModeProcess>((int)p_value));
	}
	else if (p_name == "playback_active")
	{
		set_active(p_value);
#endif // DISABLE_DEPRECATED
	}
	else
	{
		return false;
	}
	return true;
}

bool NAnimationPlayer::_get(const StringName& p_name, Variant& r_ret) const
{
	String name = p_name;

	if (name == "action_playback/play")
	{ // For backward compatibility.

		r_ret = get_current_animation();

	}
	else if (name.begins_with("next/"))
	{
		String which = name.get_slicec('/', 1);
		r_ret = animation_get_next(which);

	}
	else if (p_name == SceneStringName(blend_times))
	{
		Vector<BlendKey> keys;
		for (const KeyValue<BlendKey, double>& E : blend_times)
		{
			keys.ordered_insert(E.key);
		}

		Array array;
		for (int i = 0; i < keys.size(); i++)
		{
			array.push_back(keys[i].from);
			array.push_back(keys[i].to);
			array.push_back(blend_times.get(keys[i]));
		}

		r_ret = array;
#ifndef DISABLE_DEPRECATED
	}
	else if (name == "method_call_mode")
	{
		r_ret = get_callback_mode_method();
	}
	else if (name == "playback_process_mode")
	{
		r_ret = get_callback_mode_process();
	}
	else if (name == "playback_active")
	{
		r_ret = is_active();
#endif // DISABLE_DEPRECATED
	}
	else
	{
		return false;
	}

	return true;
}

void NAnimationPlayer::_validate_property(PropertyInfo& p_property) const
{
	AnimationMixer::_validate_property(p_property);

	if (p_property.name == "current_animation")
	{
		List<String> names;

		for (const KeyValue<StringName, AnimationData>& E : animation_set)
		{
			names.push_back(E.key);
		}
		names.push_front("[stop]");
		String hint;
		for (List<String>::Element* E = names.front(); E; E = E->next())
		{
			if (E != names.front())
			{
				hint += ",";
			}
			hint += E->get();
		}

		p_property.hint_string = hint;
	}
	else if (!auto_capture && p_property.name.begins_with("playback_auto_capture_"))
	{
		p_property.usage = PROPERTY_USAGE_NONE;
	}
}

void NAnimationPlayer::_get_property_list(List<PropertyInfo>* p_list) const
{
	List<PropertyInfo> anim_names;

	for (const KeyValue<StringName, AnimationData>& E : animation_set)
	{
		AHashMap<StringName, StringName>::ConstIterator F = animation_next_set.find(E.key);
		if (F && F->value != StringName())
		{
			anim_names.push_back(PropertyInfo(Variant::STRING, "next/" + String(E.key), PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL));
		}
	}

	for (const PropertyInfo& E : anim_names)
	{
		p_list->push_back(E);
	}

	p_list->push_back(PropertyInfo(Variant::ARRAY, "blend_times", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL));
}

void NAnimationPlayer::_notification(int p_what)
{
	switch (p_what)
	{
	case NOTIFICATION_READY: {
		if (!Engine::get_singleton()->is_editor_hint() && animation_set.has(autoplay))
		{
			set_active(active);
			play(autoplay);
			_check_immediately_after_start();
		}
	} break;
	}
}

void NAnimationPlayer::_process_playback_data(PlaybackData& cd, double p_delta, float p_blend, bool p_seeked, bool p_internal_seeked, bool p_just_started, bool p_is_current)
{
	double speed = speed_scale * cd.speed_scale;
	bool backwards = signbit(speed); // Negative zero means playing backwards too.
	double delta = p_just_started ? 0 : p_delta * speed;
	double next_pos = cd.pos + delta;

	double start = cd.get_start_time();
	double end = cd.get_end_time();

	Animation::LoopedFlag looped_flag = Animation::LOOPED_FLAG_NONE;

	switch (cd.from->animation->get_loop_mode())
	{
	case Animation::LOOP_NONE: {
		if (Animation::is_less_approx(next_pos, start))
		{
			next_pos = start;
		}
		else if (Animation::is_greater_approx(next_pos, end))
		{
			next_pos = end;
		}
		delta = next_pos - cd.pos; // Fix delta (after determination of backwards because negative zero is lost here).
	} break;

	case Animation::LOOP_LINEAR: {
		if (Animation::is_less_approx(next_pos, start) && Animation::is_greater_or_equal_approx(cd.pos, start))
		{
			looped_flag = Animation::LOOPED_FLAG_START;
		}
		if (Animation::is_greater_approx(next_pos, end) && Animation::is_less_or_equal_approx(cd.pos, end))
		{
			looped_flag = Animation::LOOPED_FLAG_END;
		}
		next_pos = Math::fposmod(next_pos - start, end - start) + start;
	} break;

	case Animation::LOOP_PINGPONG: {
		if (Animation::is_less_approx(next_pos, start) && Animation::is_greater_or_equal_approx(cd.pos, start))
		{
			cd.speed_scale *= -1.0;
			looped_flag = Animation::LOOPED_FLAG_START;
		}
		if (Animation::is_greater_approx(next_pos, end) && Animation::is_less_or_equal_approx(cd.pos, end))
		{
			cd.speed_scale *= -1.0;
			looped_flag = Animation::LOOPED_FLAG_END;
		}
		next_pos = Math::pingpong(next_pos - start, end - start) + start;
	} break;

	default:
		break;
	}

	double prev_pos = cd.pos; // The animation may be changed during process, so it is safer that the state is changed before process.

	// End detection.
	if (p_is_current)
	{
		if (cd.from->animation->get_loop_mode() == Animation::LOOP_NONE)
		{
			if (!backwards && Animation::is_less_or_equal_approx(prev_pos, end) && Math::is_equal_approx(next_pos, end))
			{
				// Playback finished.
				next_pos = end; // Snap to the edge.
				end_reached = true;
				end_notify = Animation::is_less_approx(prev_pos, end); // Notify only if not already at the end.
				//p_blend = 1.0;
			}
			if (backwards && Animation::is_greater_or_equal_approx(prev_pos, start) && Math::is_equal_approx(next_pos, start))
			{
				// Playback finished.
				next_pos = start; // Snap to the edge.
				end_reached = true;
				end_notify = Animation::is_greater_approx(prev_pos, start); // Notify only if not already at the beginning.
				//p_blend = 1.0;
			}
		}
	}

	cd.pos = next_pos;

	PlaybackInfo pi;
	if (p_just_started)
	{
		pi.time = prev_pos;
		pi.delta = 0;
		pi.start = start;
		pi.end = end;
		pi.seeked = true;
	}
	else
	{
		pi.time = next_pos;
		pi.delta = delta;
		pi.start = start;
		pi.end = end;
		pi.seeked = p_seeked;
	}
	if (Math::is_zero_approx(pi.delta) && backwards)
	{
		pi.delta = -0.0; // Sign is needed to handle converted Continuous track from Discrete track correctly.
	}
	// Immediately after playback, discrete keys should be retrieved with EXACT mode since behind keys must be ignored at that time.
	pi.is_external_seeking = !p_internal_seeked && !p_just_started;
	pi.looped_flag = looped_flag;
	pi.weight = p_blend;
	make_animation_instance(cd.from->name, pi);
}

float NAnimationPlayer::get_current_blend_amount()
{
	Playback& c = action_playback;
	float blend = 1.0;
	for (const Blend& b : c.blend)
	{
		blend = blend - b.blend_left;
	}
	return MAX(0, blend);
}


void NAnimationPlayer::_blend_capture(double p_delta)
{
	blend_capture(p_delta * Math::abs(speed_scale));
}

void NAnimationPlayer::_blend_post_process()
{
	//return; // don't want to stop everything, just the action animation...

	if (end_reached)
	{
		// If the method track changes current animation, the animation is not finished.
		if (tmp_from == action_playback.current.from->animation->get_instance_id())
		{
			if (playback_queue.size())
			{
				String old = action_playback.assigned;
				play(playback_queue.front()->get());
				String new_name = action_playback.assigned;
				playback_queue.pop_front();
				if (end_notify)
				{
					emit_signal(SceneStringName(animation_changed), old, new_name);
				}
			}
			else
			{
				//_clear_caches();
				playing_action = false;
				//_set_process(false);
				if (end_notify)
				{
					emit_signal(SceneStringName(animation_finished), action_playback.assigned);
					if (movie_quit_on_finish && OS::get_singleton()->has_feature("movie"))
					{
						print_line(vformat("Movie Maker mode is enabled. Quitting on animation finish as requested by: %s", get_path()));
						get_tree()->quit();
					}
				}
			}
		}
		end_reached = false;
		end_notify = false;
	}
	tmp_from = ObjectID();
}

void NAnimationPlayer::queue(const StringName& p_name)
{
	if (!is_playing_action())
	{
		play(p_name);
	}
	else
	{
		playback_queue.push_back(p_name);
	}
}

Vector<String> NAnimationPlayer::get_queue()
{
	Vector<String> ret;
	for (const StringName& E : playback_queue)
	{
		ret.push_back(E);
	}

	return ret;
}

void NAnimationPlayer::clear_queue()
{
	playback_queue.clear();
}

void NAnimationPlayer::play_backwards(const StringName& p_name, double p_custom_blend)
{
	play(p_name, p_custom_blend, -1, true);
}

void NAnimationPlayer::play_section_with_markers_backwards(const StringName& p_name, const StringName& p_start_marker, const StringName& p_end_marker, double p_custom_blend)
{
	play_section_with_markers(p_name, p_start_marker, p_end_marker, p_custom_blend, -1, true);
}

void NAnimationPlayer::play_section_backwards(const StringName& p_name, double p_start_time, double p_end_time, double p_custom_blend)
{
	play_section(p_name, p_start_time, p_end_time, p_custom_blend, -1, true);
}

void NAnimationPlayer::play(const StringName& p_name, double p_custom_blend, float p_custom_scale, bool p_from_end)
{
	if (auto_capture)
	{
		play_with_capture(p_name, auto_capture_duration, p_custom_blend, p_custom_scale, p_from_end, auto_capture_transition_type, auto_capture_ease_type);
	}
	else
	{
		_play(p_name, p_custom_blend, p_custom_scale, p_from_end);
	}
}

void NAnimationPlayer::_play(const StringName& p_name, double p_custom_blend, float p_custom_scale, bool p_from_end)
{
	play_section_with_markers(p_name, StringName(), StringName(), p_custom_blend, p_custom_scale, p_from_end);
}

void NAnimationPlayer::play_section_with_markers(const StringName& p_name, const StringName& p_start_marker, const StringName& p_end_marker, double p_custom_blend, float p_custom_scale, bool p_from_end)
{
	StringName name = p_name;

	if (name == StringName())
	{
		name = action_playback.assigned;
	}

	ERR_FAIL_COND_MSG(!animation_set.has(name), vformat("Animation not found: %s.", name));

	Ref<Animation> animation = animation_set[name].animation;

	ERR_FAIL_COND_MSG(p_start_marker == p_end_marker && p_start_marker, vformat("Start marker and end marker cannot be the same marker: %s.", p_start_marker));
	ERR_FAIL_COND_MSG(p_start_marker && !animation->has_marker(p_start_marker), vformat("Marker %s not found in animation: %s.", p_start_marker, name));
	ERR_FAIL_COND_MSG(p_end_marker && !animation->has_marker(p_end_marker), vformat("Marker %s not found in animation: %s.", p_end_marker, name));

	double start_time = p_start_marker ? animation->get_marker_time(p_start_marker) : -1;
	double end_time = p_end_marker ? animation->get_marker_time(p_end_marker) : -1;

	ERR_FAIL_COND_MSG(p_start_marker && p_end_marker && Animation::is_greater_approx(start_time, end_time), vformat("End marker %s is placed earlier than start marker %s in animation: %s.", p_end_marker, p_start_marker, name));

	if (p_start_marker && Animation::is_less_approx(start_time, 0))
	{
		WARN_PRINT_ED(vformat("Negative time start marker: %s is invalid in the section, so the start of the animation: %s is used instead.", p_start_marker, action_playback.current.from->animation->get_name()));
	}
	if (p_end_marker && Animation::is_less_approx(end_time, 0))
	{
		WARN_PRINT_ED(vformat("Negative time end marker: %s is invalid in the section, so the end of the animation: %s is used instead.", p_end_marker, action_playback.current.from->animation->get_name()));
	}

	play_section(name, start_time, end_time, p_custom_blend, p_custom_scale, p_from_end);
}

void NAnimationPlayer::play_section(const StringName& p_name, double p_start_time, double p_end_time, double p_custom_blend, float p_custom_scale, bool p_from_end)
{
	StringName name = p_name;

	if (name == StringName())
	{
		name = action_playback.assigned;
	}

	ERR_FAIL_COND_MSG(!animation_set.has(name), vformat("Animation not found: %s.", name));
	ERR_FAIL_COND_MSG(p_start_time >= 0 && p_end_time >= 0 && Math::is_equal_approx(p_start_time, p_end_time), "Start time and end time must not equal to each other.");
	ERR_FAIL_COND_MSG(p_start_time >= 0 && p_end_time >= 0 && Animation::is_greater_approx(p_start_time, p_end_time), vformat("Start time %f is greater than end time %f.", p_start_time, p_end_time));

	Playback& c = action_playback;

	if (c.current.from)
	{
		double blend_time = 0.0;
		// Find if it can blend.
		BlendKey bk;
		bk.from = c.current.from->name;
		bk.to = name;

		if (Animation::is_greater_or_equal_approx(p_custom_blend, 0))
		{
			blend_time = p_custom_blend;
		}
		else if (blend_times.has(bk))
		{
			blend_time = blend_times[bk];
		}
		else
		{
			bk.from = "*";
			if (blend_times.has(bk))
			{
				blend_time = blend_times[bk];
			}
			else
			{
				bk.from = c.current.from->name;
				bk.to = "*";

				if (blend_times.has(bk))
				{
					blend_time = blend_times[bk];
				}
			}
		}

		if (Animation::is_less_approx(p_custom_blend, 0) && Math::is_zero_approx(blend_time) && default_blend_time)
		{
			blend_time = default_blend_time;
		}
		if (Animation::is_greater_approx(blend_time, 0))
		{
			Blend b;
			b.data = c.current;
			b.blend_left = get_current_blend_amount();
			b.blend_time = blend_time;
			c.blend.push_back(b);
		}
		else
		{
			c.blend.clear();
		}
	}

	if (get_current_animation() != p_name)
	{
		_clear_playing_caches();
	}

	c.current.from = &animation_set[name];
	c.current.speed_scale = p_custom_scale;
	c.current.start_time = p_start_time;
	c.current.end_time = p_end_time;

	double start = action_playback.current.get_start_time();
	double end = action_playback.current.get_end_time();

	if (!end_reached)
	{
		playback_queue.clear();
	}

	//if (c.assigned != name)
	// always reset
	{ // Reset.
		c.current.pos = p_from_end ? end : start;
		c.assigned = name;
		emit_signal(SNAME("current_animation_changed"), c.assigned);
	}
	//else
	//{
	//	if (p_from_end && Math::is_equal_approx(c.current.pos, start))
	//	{
	//		// Animation reset but played backwards, set position to the end.
	//		seek_internal(end, true, true, true);
	//	}
	//	else if (!p_from_end && Math::is_equal_approx(c.current.pos, end))
	//	{
	//		// Animation resumed but already ended, set position to the beginning.
	//		seek_internal(start, true, true, true);
	//	}
	//	else if (playing_action)
	//	{
	//		return;
	//	}
	//}

	c.seeked = false;
	c.just_started = true;

	_set_process(true); // Always process when starting an animation.
	playing_action = true;

	emit_signal(SceneStringName(animation_started), c.assigned);

	if (is_inside_tree() && Engine::get_singleton()->is_editor_hint())
	{
		return; // No next in this case.
	}

	StringName next = animation_get_next(p_name);
	if (next != StringName() && animation_set.has(next))
	{
		queue(next);
	}
}

void NAnimationPlayer::_capture(const StringName& p_name, bool p_from_end, double p_duration, Tween::TransitionType p_trans_type, Tween::EaseType p_ease_type)
{
	StringName name = p_name;
	if (name == StringName())
	{
		name = action_playback.assigned;
	}

	Ref<Animation> anim = get_animation(name);
	if (anim.is_null() || !anim->is_capture_included())
	{
		return;
	}
	if (signbit(p_duration))
	{
		double max_dur = 0;
		double current_pos = action_playback.current.pos;
		if (action_playback.assigned != name)
		{
			current_pos = p_from_end ? anim->get_length() : 0;
		}
		for (int i = 0; i < anim->get_track_count(); i++)
		{
			if (anim->track_get_type(i) != Animation::TYPE_VALUE)
			{
				continue;
			}
			if (anim->value_track_get_update_mode(i) != Animation::UPDATE_CAPTURE)
			{
				continue;
			}
			if (anim->track_get_key_count(i) == 0)
			{
				continue;
			}
			max_dur = MAX(max_dur, p_from_end ? current_pos - anim->track_get_key_time(i, anim->track_get_key_count(i) - 1) : anim->track_get_key_time(i, 0) - current_pos);
		}
		p_duration = max_dur;
	}
	if (Math::is_zero_approx(p_duration))
	{
		return;
	}
	capture(name, p_duration, p_trans_type, p_ease_type);
}

void NAnimationPlayer::play_with_capture(const StringName& p_name, double p_duration, double p_custom_blend, float p_custom_scale, bool p_from_end, Tween::TransitionType p_trans_type, Tween::EaseType p_ease_type)
{
	_capture(p_name, p_from_end, p_duration, p_trans_type, p_ease_type);
	_play(p_name, p_custom_blend, p_custom_scale, p_from_end);
}

bool NAnimationPlayer::is_playing_action() const
{
	return playing_action;
}

void NAnimationPlayer::set_current_animation(const String& p_animation)
{
	if (p_animation == "[stop]" || p_animation.is_empty())
	{
		stop();
	}
	else if (!is_playing_action())
	{
		play(p_animation);
	}
	else if (action_playback.assigned != p_animation)
	{
		float speed = action_playback.current.speed_scale;
		play(p_animation, -1.0, speed, signbit(speed));
	}
	else
	{
		// Same animation, do not replay from start.
	}
}

String NAnimationPlayer::get_current_animation() const
{
	return (is_playing_action() ? action_playback.assigned : "");
}

void NAnimationPlayer::set_assigned_animation(const String& p_animation)
{
	if (is_playing_action())
	{
		float speed = action_playback.current.speed_scale;
		play(p_animation, -1.0, speed, signbit(speed));
	}
	else
	{
		ERR_FAIL_COND_MSG(!animation_set.has(p_animation), vformat("Animation not found: %s.", p_animation));
		action_playback.current.pos = 0;
		action_playback.current.from = &animation_set[p_animation];
		action_playback.current.start_time = -1;
		action_playback.current.end_time = -1;
		action_playback.assigned = p_animation;
		emit_signal(SNAME("current_animation_changed"), action_playback.assigned);
	}
}

String NAnimationPlayer::get_assigned_animation() const
{
	return action_playback.assigned;
}

void NAnimationPlayer::pause()
{
	_stop_internal(false, false);
}

void NAnimationPlayer::stop(bool p_keep_state)
{
	_stop_internal(true, p_keep_state);
}

void NAnimationPlayer::set_speed_scale(float p_speed)
{
	speed_scale = p_speed;
}

float NAnimationPlayer::get_speed_scale() const
{
	return speed_scale;
}

float NAnimationPlayer::get_playing_speed() const
{
	if (!playing_action)
	{
		return 0;
	}
	return speed_scale * action_playback.current.speed_scale;
}

void NAnimationPlayer::seek_internal(double p_time, bool p_update, bool p_update_only, bool p_is_internal_seek)
{
	if (!active)
	{
		return;
	}

	bool is_backward = Animation::is_less_approx(p_time, action_playback.current.pos);

	_check_immediately_after_start();

	action_playback.current.pos = p_time;
	if (!action_playback.current.from)
	{
		if (action_playback.assigned)
		{
			ERR_FAIL_COND_MSG(!animation_set.has(action_playback.assigned), vformat("Animation not found: %s.", action_playback.assigned));
			action_playback.current.from = &animation_set[action_playback.assigned];
		}
		if (!action_playback.current.from)
		{
			return; // There is no animation.
		}
	}

	double start = action_playback.current.get_start_time();
	double end = action_playback.current.get_end_time();

	// Clamp the seek position.
	p_time = CLAMP(p_time, start, end);

	action_playback.seeked = true;
	action_playback.internal_seeked = p_is_internal_seek;

	if (p_update)
	{
		_process_animation(is_backward ? -0.0 : 0.0, p_update_only);
		action_playback.seeked = false; // If animation was proceeded here, no more seek in internal process.
	}
}

void NAnimationPlayer::seek(double p_time, bool p_update, bool p_update_only)
{
	seek_internal(p_time, p_update, p_update_only);
}

void NAnimationPlayer::advance(double p_time)
{
	_check_immediately_after_start();
	AnimationMixer::advance(p_time);
}

void NAnimationPlayer::_check_immediately_after_start()
{
	if (action_playback.just_started)
	{
		_process_animation(0); // Force process current key for Discrete/Method/Audio/AnimationPlayback. Then, started flag is cleared.
	}
}

bool NAnimationPlayer::is_valid() const
{
	return (action_playback.current.from);
}

double NAnimationPlayer::get_current_animation_position() const
{
	ERR_FAIL_NULL_V_MSG(action_playback.current.from, 0, "NAnimationPlayer has no current animation.");
	return action_playback.current.pos;
}

double NAnimationPlayer::get_current_animation_length() const
{
	ERR_FAIL_NULL_V_MSG(action_playback.current.from, 0, "NAnimationPlayer has no current animation.");
	return action_playback.current.from->animation->get_length();
}

void NAnimationPlayer::set_section_with_markers(const StringName& p_start_marker, const StringName& p_end_marker)
{
	ERR_FAIL_NULL_MSG(action_playback.current.from, "NAnimationPlayer has no current animation.");
	ERR_FAIL_COND_MSG(p_start_marker == p_end_marker && p_start_marker, vformat("Start marker and end marker cannot be the same marker: %s.", p_start_marker));
	ERR_FAIL_COND_MSG(p_start_marker && !action_playback.current.from->animation->has_marker(p_start_marker), vformat("Marker %s not found in animation: %s.", p_start_marker, action_playback.current.from->animation->get_name()));
	ERR_FAIL_COND_MSG(p_end_marker && !action_playback.current.from->animation->has_marker(p_end_marker), vformat("Marker %s not found in animation: %s.", p_end_marker, action_playback.current.from->animation->get_name()));
	double start_time = p_start_marker ? action_playback.current.from->animation->get_marker_time(p_start_marker) : -1;
	double end_time = p_end_marker ? action_playback.current.from->animation->get_marker_time(p_end_marker) : -1;
	if (p_start_marker && Animation::is_less_approx(start_time, 0))
	{
		WARN_PRINT_ONCE_ED(vformat("Marker %s time must be positive in animation: %s.", p_start_marker, action_playback.current.from->animation->get_name()));
	}
	if (p_end_marker && Animation::is_less_approx(end_time, 0))
	{
		WARN_PRINT_ONCE_ED(vformat("Marker %s time must be positive in animation: %s.", p_end_marker, action_playback.current.from->animation->get_name()));
	}
	set_section(start_time, end_time);
}

void NAnimationPlayer::set_section(double p_start_time, double p_end_time)
{
	ERR_FAIL_NULL_MSG(action_playback.current.from, "NAnimationPlayer has no current animation.");
	ERR_FAIL_COND_MSG(Animation::is_greater_or_equal_approx(p_start_time, 0) && Animation::is_greater_or_equal_approx(p_end_time, 0) && Animation::is_greater_or_equal_approx(p_start_time, p_end_time), vformat("Start time %f is greater than end time %f.", p_start_time, p_end_time));
	action_playback.current.start_time = p_start_time;
	action_playback.current.end_time = p_end_time;
	action_playback.current.pos = CLAMP(action_playback.current.pos, action_playback.current.get_start_time(), action_playback.current.get_end_time());
}

void NAnimationPlayer::reset_section()
{
	action_playback.current.start_time = -1;
	action_playback.current.end_time = -1;
}

double NAnimationPlayer::get_section_start_time() const
{
	ERR_FAIL_NULL_V_MSG(action_playback.current.from, action_playback.current.start_time, "NAnimationPlayer has no current animation.");
	return action_playback.current.get_start_time();
}

double NAnimationPlayer::get_section_end_time() const
{
	ERR_FAIL_NULL_V_MSG(action_playback.current.from, action_playback.current.end_time, "NAnimationPlayer has no current animation.");
	return action_playback.current.get_end_time();
}

bool NAnimationPlayer::has_section() const
{
	return Animation::is_greater_or_equal_approx(action_playback.current.start_time, 0) || Animation::is_greater_or_equal_approx(action_playback.current.end_time, 0);
}

void NAnimationPlayer::set_autoplay(const String& p_name)
{
	if (is_inside_tree() && !Engine::get_singleton()->is_editor_hint())
	{
		WARN_PRINT("Setting autoplay after the node has been added to the scene has no effect.");
	}

	autoplay = p_name;
}

String NAnimationPlayer::get_autoplay() const
{
	return autoplay;
}

void NAnimationPlayer::set_movie_quit_on_finish_enabled(bool p_enabled)
{
	movie_quit_on_finish = p_enabled;
}

bool NAnimationPlayer::is_movie_quit_on_finish_enabled() const
{
	return movie_quit_on_finish;
}

void NAnimationPlayer::_stop_internal(bool p_reset, bool p_keep_state)
{
	_clear_caches();
	Playback& c = action_playback;
	// c.blend.clear();
	double start = c.current.from ? c.current.get_start_time() : 0;
	if (p_reset)
	{
		c.blend.clear();
		if (p_keep_state)
		{
			c.current.pos = start;
		}
		else
		{
			is_stopping = true;
			seek_internal(start, true, true, true);
			is_stopping = false;
		}
		c.current.from = nullptr;
		c.current.speed_scale = 1;
		emit_signal(SNAME("current_animation_changed"), "");
	}
	_set_process(false);
	playback_queue.clear();
	playing_action = false;
}

void NAnimationPlayer::animation_set_next(const StringName& p_animation, const StringName& p_next)
{
	ERR_FAIL_COND_MSG(!animation_set.has(p_animation), vformat("Animation not found: %s.", p_animation));
	animation_next_set[p_animation] = p_next;
}

StringName NAnimationPlayer::animation_get_next(const StringName& p_animation) const
{
	if (!animation_next_set.has(p_animation))
	{
		return StringName();
	}
	return animation_next_set[p_animation];
}

void NAnimationPlayer::set_default_blend_time(double p_default)
{
	default_blend_time = p_default;
}

double NAnimationPlayer::get_default_blend_time() const
{
	return default_blend_time;
}

void NAnimationPlayer::set_blend_time(const StringName& p_animation1, const StringName& p_animation2, double p_time)
{
	ERR_FAIL_COND_MSG(!animation_set.has(p_animation1), vformat("Animation not found: %s.", p_animation1));
	ERR_FAIL_COND_MSG(!animation_set.has(p_animation2), vformat("Animation not found: %s.", p_animation2));
	ERR_FAIL_COND_MSG(p_time < 0, "Blend time cannot be smaller than 0.");

	BlendKey bk;
	bk.from = p_animation1;
	bk.to = p_animation2;
	if (Math::is_zero_approx(p_time))
	{
		blend_times.erase(bk);
	}
	else
	{
		blend_times[bk] = p_time;
	}
}

double NAnimationPlayer::get_blend_time(const StringName& p_animation1, const StringName& p_animation2) const
{
	BlendKey bk;
	bk.from = p_animation1;
	bk.to = p_animation2;

	if (blend_times.has(bk))
	{
		return blend_times[bk];
	}
	else
	{
		return 0;
	}
}

void NAnimationPlayer::set_auto_capture(bool p_auto_capture)
{
	auto_capture = p_auto_capture;
	notify_property_list_changed();
}

bool NAnimationPlayer::is_auto_capture() const
{
	return auto_capture;
}

void NAnimationPlayer::set_auto_capture_duration(double p_auto_capture_duration)
{
	auto_capture_duration = p_auto_capture_duration;
}

double NAnimationPlayer::get_auto_capture_duration() const
{
	return auto_capture_duration;
}

void NAnimationPlayer::set_auto_capture_transition_type(Tween::TransitionType p_auto_capture_transition_type)
{
	auto_capture_transition_type = p_auto_capture_transition_type;
}

Tween::TransitionType NAnimationPlayer::get_auto_capture_transition_type() const
{
	return auto_capture_transition_type;
}

void NAnimationPlayer::set_auto_capture_ease_type(Tween::EaseType p_auto_capture_ease_type)
{
	auto_capture_ease_type = p_auto_capture_ease_type;
}

Tween::EaseType NAnimationPlayer::get_auto_capture_ease_type() const
{
	return auto_capture_ease_type;
}

#ifdef TOOLS_ENABLED
void NAnimationPlayer::get_argument_options(const StringName& p_function, int p_idx, List<String>* r_options) const
{
	const String pf = p_function;
	if (p_idx == 0 && (pf == "play" || pf == "play_backwards" || pf == "has_animation" || pf == "queue"))
	{
		List<StringName> al;
		get_animation_list(&al);
		for (const StringName& name : al)
		{
			r_options->push_back(String(name).quote());
		}
	}
	AnimationMixer::get_argument_options(p_function, p_idx, r_options);
}
#endif

void NAnimationPlayer::_animation_removed(const StringName& p_name, const StringName& p_library)
{
	AnimationMixer::_animation_removed(p_name, p_library);

	StringName name = p_library == StringName() ? p_name : StringName(String(p_library) + "/" + String(p_name));

	if (!animation_set.has(name))
	{
		return; // No need to update because not the one from the library being used.
	}

	_animation_set_cache_update();

	// Erase blends if needed
	List<BlendKey> to_erase;
	for (const KeyValue<BlendKey, double>& E : blend_times)
	{
		BlendKey bk = E.key;
		if (bk.from == name || bk.to == name)
		{
			to_erase.push_back(bk);
		}
	}

	while (to_erase.size())
	{
		blend_times.erase(to_erase.front()->get());
		to_erase.pop_front();
	}
}

void NAnimationPlayer::_rename_animation(const StringName& p_from_name, const StringName& p_to_name)
{
	AnimationMixer::_rename_animation(p_from_name, p_to_name);

	// Rename autoplay or blends if needed.
	List<BlendKey> to_erase;
	HashMap<BlendKey, double, BlendKey> to_insert;
	for (const KeyValue<BlendKey, double>& E : blend_times)
	{
		BlendKey bk = E.key;
		BlendKey new_bk = bk;
		bool erase = false;
		if (bk.from == p_from_name)
		{
			new_bk.from = p_to_name;
			erase = true;
		}
		if (bk.to == p_from_name)
		{
			new_bk.to = p_to_name;
			erase = true;
		}

		if (erase)
		{
			to_erase.push_back(bk);
			to_insert[new_bk] = E.value;
		}
	}

	while (to_erase.size())
	{
		blend_times.erase(to_erase.front()->get());
		to_erase.pop_front();
	}

	while (to_insert.size())
	{
		blend_times[to_insert.begin()->key] = to_insert.begin()->value;
		to_insert.remove(to_insert.begin());
	}

	if (autoplay == p_from_name)
	{
		autoplay = p_to_name;
	}
}







void NAnimationPlayer::my_blend_playback_data(double p_delta, bool p_just_started)
{
	// this function is calling process_playback_data on current and all the blended animations
	// I probably need to:
	// get all the current locomotion animations and process them, apply weights, mute low weight method tracks

	// if i treat action_playback as the montage, then maybe i can blend out of it?
	// and blend into locomotion?
	float locomotion_blend = (1 - action_blend);
	float blend_a = locomotion_blend * locomotion_blend_a;
	float blend_b = locomotion_blend * (1- locomotion_blend_a);

	Vector<NAnimationInstance> anim_instances;

	if (locomotion_anim_mode.is_valid())
	{
		locomotion_anim_mode->process_animation_mode(*this, p_delta, anim_instances);
	}

	

	for (int i = 0; i < anim_instances.size(); i++)
	{
		NAnimationInstance& ai = anim_instances.ptrw()[i];
		ai.pi.weight *= locomotion_blend;
		make_animation_instance(ai.anim_name, ai.pi); 
		//make_animation_instance(anim_instances[i].anim_name, anim_instances[i].pi); 
	}


	// unless i start using the animation player as an animation player, i don't want this stuff
	// ..
	return;

	//_process_playback_data(nanim_a.current, p_delta, blend_a, false/*seeked*/, false/*internal_seeked*/, false/*just_started*/, false/*is_current*/);
	//_process_playback_data(nanim_b.current, p_delta, blend_b, false/*seeked*/, false/*internal_seeked*/, false/*just_started*/, false/*is_current*/);

	Playback& c = action_playback;

	bool seeked = c.seeked; // The animation may be changed during process, so it is safer that the state is changed before process.
	bool internal_seeked = c.internal_seeked;

	if (!Math::is_zero_approx(p_delta))
	{
		c.seeked = false;
		c.internal_seeked = false;
	}


	//if (Math::is_zero_approx(action_blend))
	//{
	//	return;
	//}
	float current_action_blend = get_current_blend_amount() * action_blend;

	// Second, process current animation to check if the animation end reached.
	_process_playback_data(c.current, p_delta, current_action_blend, seeked, internal_seeked, p_just_started, true);

	// Finally, if not end the animation, do blending.
	if (end_reached)
	{
		action_playback.blend.clear();
		return;
	}
	List<List<Blend>::Element*> to_erase;
	for (List<Blend>::Element* E = c.blend.front(); E; E = E->next())
	{
		Blend& b = E->get();
		b.blend_left = MAX(0, b.blend_left - Math::absf(speed_scale * p_delta) / b.blend_time);
		if (Animation::is_less_or_equal_approx(b.blend_left, 0))
		{
			to_erase.push_back(E);
			b.blend_left = CMP_EPSILON; // May want to play last frame.
		}
		// Note: There may be issues if an animation event triggers an animation change while this blend is active,
		// so it is best to use "deferred" calls instead of "immediate" for animation events that can trigger new animations.
		_process_playback_data(b.data, p_delta, b.blend_left * current_action_blend, false, false, false);
	}
	for (List<Blend>::Element*& E : to_erase)
	{
		c.blend.erase(E);
	}
}

void NAnimationPlayer::_blend_playback_data(double p_delta, bool p_just_started)
{
	Playback& c = action_playback;

	bool seeked = c.seeked; // The animation may be changed during process, so it is safer that the state is changed before process.
	bool internal_seeked = c.internal_seeked;

	if (!Math::is_zero_approx(p_delta))
	{
		c.seeked = false;
		c.internal_seeked = false;
	}

	// Second, process current animation to check if the animation end reached.
	_process_playback_data(c.current, p_delta, get_current_blend_amount(), seeked, internal_seeked, p_just_started, true);

	// Finally, if not end the animation, do blending.
	if (end_reached)
	{
		action_playback.blend.clear();
		return;
	}
	List<List<Blend>::Element*> to_erase;
	for (List<Blend>::Element* E = c.blend.front(); E; E = E->next())
	{
		Blend& b = E->get();
		b.blend_left = MAX(0, b.blend_left - Math::absf(speed_scale * p_delta) / b.blend_time);
		if (Animation::is_less_or_equal_approx(b.blend_left, 0))
		{
			to_erase.push_back(E);
			b.blend_left = CMP_EPSILON; // May want to play last frame.
		}
		// Note: There may be issues if an animation event triggers an animation change while this blend is active,
		// so it is best to use "deferred" calls instead of "immediate" for animation events that can trigger new animations.
		_process_playback_data(b.data, p_delta, b.blend_left, false, false, false);
	}
	for (List<Blend>::Element*& E : to_erase)
	{
		c.blend.erase(E);
	}
}

bool NAnimationPlayer::my_blend_pre_process(double p_delta, int p_track_count, const AHashMap<NodePath, int>& p_track_map)
{
	// make sure everything is valid? need many 'playback's
	//if (!action_playback.current.from)
	//{
	//	_set_process(false);
	//	return false;
	//}

	//tmp_from = action_playback.current.from->animation->get_instance_id();
	end_reached = false;
	end_notify = false;

	bool just_started = action_playback.just_started; // The animation may be changed during process, so it is safer that the state is changed before process.
	if (action_playback.just_started)
	{
		action_playback.just_started = false;
	}

	AnimationData* prev_from = action_playback.current.from;
	my_blend_playback_data(p_delta, just_started);

	//if (prev_from != action_playback.current.from)
	//{
	//	return false; // Animation has been changed in the process (may be caused by method track), abort process.
	//}

	return true;
}

bool NAnimationPlayer::_blend_pre_process(double p_delta, int p_track_count, const AHashMap<NodePath, int>& p_track_map)
{
	// 
	return my_blend_pre_process(p_delta, p_track_count, p_track_map);
	//

	if (!action_playback.current.from)
	{
		_set_process(false);
		return false;
	}

	tmp_from = action_playback.current.from->animation->get_instance_id();
	end_reached = false;
	end_notify = false;

	bool just_started = action_playback.just_started; // The animation may be changed during process, so it is safer that the state is changed before process.
	if (action_playback.just_started)
	{
		action_playback.just_started = false;
	}

	AnimationData* prev_from = action_playback.current.from;
	_blend_playback_data(p_delta, just_started);

	if (prev_from != action_playback.current.from)
	{
		return false; // Animation has been changed in the process (may be caused by method track), abort process.
	}

	return true;
}

void NAnimationPlayer::set_locomotion_mode(NAnimationMode* p_locomotion_mode)
{
	locomotion_anim_mode = p_locomotion_mode;
}
void NAnimationPlayer::set_override_mode(NAnimationMode* p_override_mode)
{
	override_anim_mode = p_override_mode;
}

void NAnimationPlayer::set_nanim_a(const StringName& p_animation)
{
	set_nanim(nanim_a, p_animation);
}
void NAnimationPlayer::set_nanim_b(const StringName& p_animation)
{
	set_nanim(nanim_b, p_animation);
}

void NAnimationPlayer::set_nanim(Playback& p_playback, const StringName& p_animation)
{
	ERR_FAIL_COND_MSG(!animation_set.has(p_animation), vformat("Animation not found: %s.", p_animation));
	p_playback.current.pos = 0;
	p_playback.current.from = &animation_set[p_animation];
	p_playback.current.start_time = -1;
	p_playback.current.end_time = -1;
	p_playback.assigned = p_animation;
}

void NAnimationPlayer::set_locomotion_blend(float p_action_blend, float p_locomotion_blend_a)
{
	action_blend = p_action_blend;
	locomotion_blend_a = p_locomotion_blend_a;
}



void NAnimationPlayer::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_parameters", "parameters"), &NAnimationPlayer::set_parameters);
	ClassDB::bind_method(D_METHOD("get_parameters"), &NAnimationPlayer::get_parameters);
	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "parameters"), "set_parameters", "get_parameters");

	ClassDB::bind_method(D_METHOD("set_locomotion_mode", "locomotion_anim_mode"), &NAnimationPlayer::set_locomotion_mode);
	ClassDB::bind_method(D_METHOD("set_override_mode", "override_anim_mode"), &NAnimationPlayer::set_override_mode);


	ClassDB::bind_method(D_METHOD("set_locomotion_blend", "action_blend", "locomotion_blend_a"), &NAnimationPlayer::set_locomotion_blend);
	ClassDB::bind_method(D_METHOD("set_nanim_a", "animation"), &NAnimationPlayer::set_nanim_a);
	ClassDB::bind_method(D_METHOD("set_nanim_b", "animation"), &NAnimationPlayer::set_nanim_b);


	ClassDB::bind_method(D_METHOD("animation_set_next", "animation_from", "animation_to"), &NAnimationPlayer::animation_set_next);
	ClassDB::bind_method(D_METHOD("animation_get_next", "animation_from"), &NAnimationPlayer::animation_get_next);

	ClassDB::bind_method(D_METHOD("set_blend_time", "animation_from", "animation_to", "sec"), &NAnimationPlayer::set_blend_time);
	ClassDB::bind_method(D_METHOD("get_blend_time", "animation_from", "animation_to"), &NAnimationPlayer::get_blend_time);

	ClassDB::bind_method(D_METHOD("set_default_blend_time", "sec"), &NAnimationPlayer::set_default_blend_time);
	ClassDB::bind_method(D_METHOD("get_default_blend_time"), &NAnimationPlayer::get_default_blend_time);

	ClassDB::bind_method(D_METHOD("set_auto_capture", "auto_capture"), &NAnimationPlayer::set_auto_capture);
	ClassDB::bind_method(D_METHOD("is_auto_capture"), &NAnimationPlayer::is_auto_capture);
	ClassDB::bind_method(D_METHOD("set_auto_capture_duration", "auto_capture_duration"), &NAnimationPlayer::set_auto_capture_duration);
	ClassDB::bind_method(D_METHOD("get_auto_capture_duration"), &NAnimationPlayer::get_auto_capture_duration);
	ClassDB::bind_method(D_METHOD("set_auto_capture_transition_type", "auto_capture_transition_type"), &NAnimationPlayer::set_auto_capture_transition_type);
	ClassDB::bind_method(D_METHOD("get_auto_capture_transition_type"), &NAnimationPlayer::get_auto_capture_transition_type);
	ClassDB::bind_method(D_METHOD("set_auto_capture_ease_type", "auto_capture_ease_type"), &NAnimationPlayer::set_auto_capture_ease_type);
	ClassDB::bind_method(D_METHOD("get_auto_capture_ease_type"), &NAnimationPlayer::get_auto_capture_ease_type);

	ClassDB::bind_method(D_METHOD("play", "name", "custom_blend", "custom_speed", "from_end"), &NAnimationPlayer::play, DEFVAL(StringName()), DEFVAL(-1), DEFVAL(1.0), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("play_section_with_markers", "name", "start_marker", "end_marker", "custom_blend", "custom_speed", "from_end"), &NAnimationPlayer::play_section_with_markers, DEFVAL(StringName()), DEFVAL(StringName()), DEFVAL(StringName()), DEFVAL(-1), DEFVAL(1.0), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("play_section", "name", "start_time", "end_time", "custom_blend", "custom_speed", "from_end"), &NAnimationPlayer::play_section, DEFVAL(StringName()), DEFVAL(-1), DEFVAL(-1), DEFVAL(-1), DEFVAL(1.0), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("play_backwards", "name", "custom_blend"), &NAnimationPlayer::play_backwards, DEFVAL(StringName()), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("play_section_with_markers_backwards", "name", "start_marker", "end_marker", "custom_blend"), &NAnimationPlayer::play_section_with_markers_backwards, DEFVAL(StringName()), DEFVAL(StringName()), DEFVAL(StringName()), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("play_section_backwards", "name", "start_time", "end_time", "custom_blend"), &NAnimationPlayer::play_section_backwards, DEFVAL(StringName()), DEFVAL(-1), DEFVAL(-1), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("play_with_capture", "name", "duration", "custom_blend", "custom_speed", "from_end", "trans_type", "ease_type"), &NAnimationPlayer::play_with_capture, DEFVAL(StringName()), DEFVAL(-1.0), DEFVAL(-1), DEFVAL(1.0), DEFVAL(false), DEFVAL(Tween::TRANS_LINEAR), DEFVAL(Tween::EASE_IN));
	ClassDB::bind_method(D_METHOD("pause"), &NAnimationPlayer::pause);
	ClassDB::bind_method(D_METHOD("stop", "keep_state"), &NAnimationPlayer::stop, DEFVAL(false));
	ClassDB::bind_method(D_METHOD("is_playing_action"), &NAnimationPlayer::is_playing_action);

	ClassDB::bind_method(D_METHOD("set_current_animation", "animation"), &NAnimationPlayer::set_current_animation);
	ClassDB::bind_method(D_METHOD("get_current_animation"), &NAnimationPlayer::get_current_animation);
	ClassDB::bind_method(D_METHOD("set_assigned_animation", "animation"), &NAnimationPlayer::set_assigned_animation);
	ClassDB::bind_method(D_METHOD("get_assigned_animation"), &NAnimationPlayer::get_assigned_animation);
	ClassDB::bind_method(D_METHOD("queue", "name"), &NAnimationPlayer::queue);
	ClassDB::bind_method(D_METHOD("get_queue"), &NAnimationPlayer::get_queue);
	ClassDB::bind_method(D_METHOD("clear_queue"), &NAnimationPlayer::clear_queue);

	ClassDB::bind_method(D_METHOD("set_speed_scale", "speed"), &NAnimationPlayer::set_speed_scale);
	ClassDB::bind_method(D_METHOD("get_speed_scale"), &NAnimationPlayer::get_speed_scale);
	ClassDB::bind_method(D_METHOD("get_playing_speed"), &NAnimationPlayer::get_playing_speed);

	ClassDB::bind_method(D_METHOD("set_autoplay", "name"), &NAnimationPlayer::set_autoplay);
	ClassDB::bind_method(D_METHOD("get_autoplay"), &NAnimationPlayer::get_autoplay);

	ClassDB::bind_method(D_METHOD("find_animation", "animation"), &NAnimationPlayer::find_animation);
	ClassDB::bind_method(D_METHOD("find_animation_library", "animation"), &NAnimationPlayer::find_animation_library);

	ClassDB::bind_method(D_METHOD("set_movie_quit_on_finish_enabled", "enabled"), &NAnimationPlayer::set_movie_quit_on_finish_enabled);
	ClassDB::bind_method(D_METHOD("is_movie_quit_on_finish_enabled"), &NAnimationPlayer::is_movie_quit_on_finish_enabled);

	ClassDB::bind_method(D_METHOD("get_current_animation_position"), &NAnimationPlayer::get_current_animation_position);
	ClassDB::bind_method(D_METHOD("get_current_animation_length"), &NAnimationPlayer::get_current_animation_length);

	ClassDB::bind_method(D_METHOD("set_section_with_markers", "start_marker", "end_marker"), &NAnimationPlayer::set_section_with_markers, DEFVAL(StringName()), DEFVAL(StringName()));
	ClassDB::bind_method(D_METHOD("set_section", "start_time", "end_time"), &NAnimationPlayer::set_section, DEFVAL(-1), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("reset_section"), &NAnimationPlayer::reset_section);

	ClassDB::bind_method(D_METHOD("get_section_start_time"), &NAnimationPlayer::get_section_start_time);
	ClassDB::bind_method(D_METHOD("get_section_end_time"), &NAnimationPlayer::get_section_end_time);
	ClassDB::bind_method(D_METHOD("has_section"), &NAnimationPlayer::has_section);

	ClassDB::bind_method(D_METHOD("seek", "seconds", "update", "update_only"), &NAnimationPlayer::seek, DEFVAL(false), DEFVAL(false));

	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "current_animation", PROPERTY_HINT_ENUM, "", PROPERTY_USAGE_EDITOR), "set_current_animation", "get_current_animation");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "assigned_animation", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_assigned_animation", "get_assigned_animation");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "autoplay", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR), "set_autoplay", "get_autoplay");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "current_animation_length", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_current_animation_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "current_animation_position", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_current_animation_position");

	ADD_GROUP("Playback Options", "playback_");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "playback_auto_capture"), "set_auto_capture", "is_auto_capture");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "playback_auto_capture_duration", PROPERTY_HINT_NONE, "suffix:s"), "set_auto_capture_duration", "get_auto_capture_duration");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "playback_auto_capture_transition_type", PROPERTY_HINT_ENUM, "Linear,Sine,Quint,Quart,Quad,Expo,Elastic,Cubic,Circ,Bounce,Back,Spring"), "set_auto_capture_transition_type", "get_auto_capture_transition_type");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "playback_auto_capture_ease_type", PROPERTY_HINT_ENUM, "In,Out,InOut,OutIn"), "set_auto_capture_ease_type", "get_auto_capture_ease_type");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "playback_default_blend_time", PROPERTY_HINT_RANGE, "0,4096,0.01,suffix:s"), "set_default_blend_time", "get_default_blend_time");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed_scale", PROPERTY_HINT_RANGE, "-4,4,0.001,or_less,or_greater"), "set_speed_scale", "get_speed_scale");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "movie_quit_on_finish"), "set_movie_quit_on_finish_enabled", "is_movie_quit_on_finish_enabled");

	ADD_SIGNAL(MethodInfo(SNAME("current_animation_changed"), PropertyInfo(Variant::STRING, "name")));
	ADD_SIGNAL(MethodInfo(SNAME("animation_changed"), PropertyInfo(Variant::STRING_NAME, "old_name"), PropertyInfo(Variant::STRING_NAME, "new_name")));
}

NAnimationPlayer::NAnimationPlayer()
{
	//deterministic = true;
}

NAnimationPlayer::~NAnimationPlayer()
{
}
