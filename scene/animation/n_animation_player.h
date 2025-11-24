
#pragma once

#include "animation_mixer.h"
#include "scene/resources/animation.h"


class NAnimationPlayer;


struct NAnimationInstance
{
	AnimationMixer::PlaybackInfo pi;
	StringName anim_name;
};

enum FilterType : uint8_t
{
	BLEND,
	ADD,
	NORMALIZE
};


struct NAnimationFilter
{
	AHashMap<NodePath, bool> filter_map;
	FilterType filter_type = FilterType::BLEND;

	void set_filtered_tracks(const TypedArray<String>& filtered_tracks)
	{
		filter_map.clear();
		for (int i = 0; i < filtered_tracks.size(); i++)
		{
			NodePath new_filter_path = NodePath(filtered_tracks[i]);
			filter_map[new_filter_path] = true;
		}
	}

	void apply_filter(NAnimationPlayer& nanim_player, float amount, Vector<NAnimationInstance>& source_instances, Vector<NAnimationInstance>& target_instances);
};


class NAnimationMode : public Resource
{
	GDCLASS(NAnimationMode, Resource);
public:

	virtual void process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances) {};

	static void _bind_methods();

};

class NAnimationMode_OneShot : public NAnimationMode
{
	GDCLASS(NAnimationMode_OneShot, NAnimationMode);
public:

	// for duration i guess
	float timer = 0.f;
	float progress_fraction = 0.f;

	bool triggered = false;
	StringName triggered_key;
	void set_triggered_key(const StringName& p_value) { triggered_key = p_value; };
	StringName get_triggered_key() { return triggered_key; };

	Ref<NAnimationMode> source_mode;


	// I'll pass a dictionary if i need to change the params for this mode, because freak all this typing
	// can't believe there's no macro for this
	StringName one_shot_params_key;
	void set_one_shot_params_key(const StringName& p_value) { one_shot_params_key = p_value; };
	StringName get_one_shot_params_key() { return one_shot_params_key; };

	// some shit I might want to set with one shot params, but also have default values in case i don't care
	StringName anim_name;
	void set_anim_name(const StringName& p_value) { anim_name = p_value; };
	StringName get_anim_name() { return anim_name; };

	float blend_in_time = 0.1f;
	void set_blend_in_time(float p_value) { blend_in_time = p_value; };
	float get_blend_in_time() { return blend_in_time; };

	float blend_out_time = 0.1f;
	void set_blend_out_time(float p_value) { blend_out_time = p_value; };
	float get_blend_out_time() { return blend_out_time; };

	// only use this when looping and auto_blend_out?
	float duration = 0.0f;
	void set_duration(float p_value) { duration = p_value; };
	float get_duration() { return duration; };

	bool looping = false;
	void set_looping(bool p_value) { looping = p_value; };
	bool get_looping() { return looping; };

	bool auto_blend_out = true;
	void set_auto_blend_out(bool p_value) { auto_blend_out = p_value; };
	bool get_auto_blend_out() { return auto_blend_out; };

	bool freeze = false;
	void set_freeze(bool p_value) { freeze = p_value; };
	bool get_freeze() { return freeze; };

	virtual void process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances) override;

	static void _bind_methods();

};

class NAnimationMode_Blend : public NAnimationMode
{
	GDCLASS(NAnimationMode_Blend, NAnimationMode);
public:
	Ref<NAnimationMode> source_mode;
	Ref<NAnimationMode> get_source_mode() { return source_mode; };
	void set_source_mode(NAnimationMode* p_value) { source_mode = p_value; };

	Ref<NAnimationMode> target_mode;
	Ref<NAnimationMode> get_target_mode() { return target_mode; };
	void set_target_mode(NAnimationMode* p_value) { target_mode = p_value; };

	float blend_amount = 1.0;
	float get_blend_amount() { return blend_amount; };
	void set_blend_amount(float p_value) { blend_amount = p_value; };

	StringName blend_amount_key;
	StringName get_blend_amount_key() { return blend_amount_key; };
	void set_blend_amount_key(StringName p_value) { blend_amount_key = p_value; };

	NAnimationFilter filter;
	// do i need this to make it an editable property? even though its store in the struct? pass it on?
	FilterType filter_type = FilterType::BLEND;

	TypedArray<String> filtered_tracks;
	void set_filtered_tracks(const TypedArray<String>& p_value)
	{
		filtered_tracks = p_value;
		filter.set_filtered_tracks(filtered_tracks);
	};
	TypedArray<String> get_filtered_tracks() const { return filtered_tracks; };

	void set_filter_type(int p_value)
	{
		// kinda hate this duplication... should I just pass it as a param? i hate those too...
		filter_type = static_cast<FilterType>(p_value);
		filter.filter_type = static_cast<FilterType>(p_value);
	};
	int get_filter_type() { return static_cast<int>(filter_type); };


	virtual void process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances) override;

	static void _bind_methods();

};


class NAnimationAsset_Locomotion : public Resource
{
	GDCLASS(NAnimationAsset_Locomotion, Resource);
public:

	StringName animation;
	float animation_speed_scale = 1.f;
	float animation_base_speed = 0.f;


	StringName get_animation() const { return animation; };
	void set_animation(const StringName& p_animation) { animation = p_animation; };
	float get_animation_speed_scale() const { return animation_speed_scale; };
	void set_animation_speed_scale(float p_value) { animation_speed_scale = p_value;  };
	float get_animation_base_speed() const { return animation_base_speed; };
	void set_animation_base_speed(float p_value) { animation_base_speed = p_value; };

	static void _bind_methods();
};


class NAnimationAsset_Locomotion1D : public NAnimationAsset_Locomotion
{
	GDCLASS(NAnimationAsset_Locomotion1D, NAnimationAsset_Locomotion);
public:
	float blend_value = 0.f;

	float get_blend_value() const { return blend_value; };
	void set_blend_value(float p_value) { blend_value = p_value; };

	static void _bind_methods();
};

class NAnimationAsset_Locomotion2D : public NAnimationAsset_Locomotion
{
	GDCLASS(NAnimationAsset_Locomotion2D, NAnimationAsset_Locomotion);
public:

	Vector2 blend_position = Vector2();

	Vector2 get_blend_position() const { return blend_position; };
	void set_blend_position(const Vector2& p_value) { blend_position = p_value; };

	static void _bind_methods();
};


class NAnimationMode_Locomotion : public NAnimationMode
{
	GDCLASS(NAnimationMode_Locomotion, NAnimationMode);
public:

};

class NAnimationMode_Locomotion1D : public NAnimationMode_Locomotion
{
	GDCLASS(NAnimationMode_Locomotion1D, NAnimationMode_Locomotion);
public:

	StringName blend_value_key;
	StringName get_blend_value_key() { return blend_value_key; };
	void set_blend_value_key(StringName p_value) { blend_value_key = p_value; };

	float progress_fraction = 0.f;

	TypedArray<NAnimationAsset_Locomotion1D> assets;
	void set_assets(const TypedArray<NAnimationAsset_Locomotion1D>& p_assets) { assets = p_assets; };
	TypedArray<NAnimationAsset_Locomotion1D> get_assets() const { return assets; };

	virtual void process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances) override;
	static void _bind_methods();
};

class NAnimationMode_Locomotion2D : public NAnimationMode_Locomotion
{
	GDCLASS(NAnimationMode_Locomotion2D, NAnimationMode_Locomotion);
public:

	StringName blend_position_key;
	StringName get_blend_position_key() { return blend_position_key; };
	void set_blend_position_key(StringName p_value) { blend_position_key = p_value; };

	float progress_fraction = 0.f;

	struct BlendTriangle
	{
		int asset_indices[3] = {};
	};
	Vector<BlendTriangle> triangles;
	void triangulate_points();
	void add_triangle(int p_x, int p_y, int p_z, int p_at_index = -1);
	void remove_triangle(int p_triangle);
	int get_triangle_asset_index(int p_triangle, int p_point);
	int get_triangle_count() const;

	Vector2 get_closest_point(const Vector2& p_point); // not used?
	void calculate_triangle_weights(const Vector2& p_pos, const Vector2* p_points, float* r_weights);
	Vector2 get_asset_blend_position(int p_index) const;

	bool retriangulate = false;
	bool get_retriangulate() { return retriangulate; };
	void set_retriangulate(bool p_value) { triangulate_points(); };

public:

	TypedArray<NAnimationAsset_Locomotion2D> assets;
	void set_assets(const TypedArray<NAnimationAsset_Locomotion2D>& p_assets) { assets = p_assets; };
	TypedArray<NAnimationAsset_Locomotion2D> get_assets() const { return assets; };

	virtual void process_animation_mode(NAnimationPlayer& nanim_player, float delta, Vector<NAnimationInstance>& out_instances) override;

	static void _bind_methods();

};

class NAnimationMode_Overlay : public NAnimationMode
{


};

class NAnimationPlayer : public AnimationMixer
{
	GDCLASS(NAnimationPlayer, AnimationMixer);

private:
	AHashMap<StringName, StringName> animation_next_set; // For auto advance.

	float speed_scale = 1.0;
	double default_blend_time = 0.0;

	bool auto_capture = true;
	double auto_capture_duration = -1.0;
	Tween::TransitionType auto_capture_transition_type = Tween::TRANS_LINEAR;
	Tween::EaseType auto_capture_ease_type = Tween::EASE_IN;

	bool is_stopping = false;

public:

	HashMap<NodePath, int> track_map;


private:
	struct ActionParameters
	{
		float blend_in_duration = 0.f;
		float blend_out_duration = 0.f;
		bool auto_blend_out = true;

	};

	struct PlaybackData
	{
		AnimationData* from = nullptr;
		double pos = 0.0;
		float speed_scale = 1.0;
		double start_time = 0.0;
		double end_time = 0.0;
		double get_start_time() const
		{
			if (from && (Animation::is_less_approx(start_time, 0) || Animation::is_greater_approx(start_time, from->animation->get_length())))
			{
				return 0;
			}
			return start_time;
		}
		double get_end_time() const
		{
			if (from && (Animation::is_less_approx(end_time, 0) || Animation::is_greater_approx(end_time, from->animation->get_length())))
			{
				return from->animation->get_length();
			}
			return end_time;
		}
	};

	struct Blend
	{
		PlaybackData data;
		double blend_time = 0.0;
		double blend_left = 0.0;
	};

	struct Playback
	{
		PlaybackData current;
		StringName assigned;
		bool seeked = false;
		bool internal_seeked = false;
		bool just_started = false;
		List<Blend> blend;
	} action_playback;

	struct BlendKey
	{
		StringName from;
		StringName to;
		static uint32_t hash(const BlendKey& p_key)
		{
			return hash_one_uint64((uint64_t(p_key.from.hash()) << 32) | uint32_t(p_key.to.hash()));
		}
		bool operator==(const BlendKey& bk) const
		{
			return from == bk.from && to == bk.to;
		}
		bool operator<(const BlendKey& bk) const
		{
			if (from == bk.from)
			{
				return StringName::AlphCompare()(to, bk.to);
			}
			else
			{
				return StringName::AlphCompare()(from, bk.from);
			}
		}
	};

	HashMap<BlendKey, double, BlendKey> blend_times;

	List<StringName> playback_queue;
	ObjectID tmp_from;
	bool end_reached = false;
	bool end_notify = false;

	StringName autoplay;

	bool reset_on_save = true;
	bool movie_quit_on_finish = false;

	bool playing_action = false;

	void _play(const StringName& p_name, double p_custom_blend = -1, float p_custom_scale = 1.0, bool p_from_end = false);
	void _capture(const StringName& p_name, bool p_from_end = false, double p_duration = -1.0, Tween::TransitionType p_trans_type = Tween::TRANS_LINEAR, Tween::EaseType p_ease_type = Tween::EASE_IN);
	void _process_playback_data(PlaybackData& cd, double p_delta, float p_blend, bool p_seeked, bool p_internal_seeked, bool p_just_started, bool p_is_current = false);
	void _blend_playback_data(double p_delta, bool p_just_started);
	void _stop_internal(bool p_reset, bool p_keep_state);
	void _check_immediately_after_start();

	float get_current_blend_amount();


protected:
	bool _set(const StringName& p_name, const Variant& p_value);
	bool _get(const StringName& p_name, Variant& r_ret) const;
	virtual void _validate_property(PropertyInfo& p_property) const override;
	void _get_property_list(List<PropertyInfo>* p_list) const;
	void _notification(int p_what);



	virtual void _animation_removed(const StringName& p_name, const StringName& p_library) override;
	virtual void _rename_animation(const StringName& p_from_name, const StringName& p_to_name) override;


public:
	void animation_set_next(const StringName& p_animation, const StringName& p_next);
	StringName animation_get_next(const StringName& p_animation) const;

	void set_blend_time(const StringName& p_animation1, const StringName& p_animation2, double p_time);
	double get_blend_time(const StringName& p_animation1, const StringName& p_animation2) const;

	void set_default_blend_time(double p_default);
	double get_default_blend_time() const;

	void set_auto_capture(bool p_auto_capture);
	bool is_auto_capture() const;
	void set_auto_capture_duration(double p_auto_capture_duration);
	double get_auto_capture_duration() const;
	void set_auto_capture_transition_type(Tween::TransitionType p_auto_capture_transition_type);
	Tween::TransitionType get_auto_capture_transition_type() const;
	void set_auto_capture_ease_type(Tween::EaseType p_auto_capture_ease_type);
	Tween::EaseType get_auto_capture_ease_type() const;

#ifdef TOOLS_ENABLED
	void get_argument_options(const StringName& p_function, int p_idx, List<String>* r_options) const override;
#endif

	void play(const StringName& p_name = StringName(), double p_custom_blend = -1, float p_custom_scale = 1.0, bool p_from_end = false);
	void play_section_with_markers(const StringName& p_name = StringName(), const StringName& p_start_marker = StringName(), const StringName& p_end_marker = StringName(), double p_custom_blend = -1, float p_custom_scale = 1.0, bool p_from_end = false);
	void play_section(const StringName& p_name = StringName(), double p_start_time = -1, double p_end_time = -1, double p_custom_blend = -1, float p_custom_scale = 1.0, bool p_from_end = false);
	void play_backwards(const StringName& p_name = StringName(), double p_custom_blend = -1);
	void play_section_with_markers_backwards(const StringName& p_name = StringName(), const StringName& p_start_marker = StringName(), const StringName& p_end_marker = StringName(), double p_custom_blend = -1);
	void play_section_backwards(const StringName& p_name = StringName(), double p_start_time = -1, double p_end_time = -1, double p_custom_blend = -1);
	void play_with_capture(const StringName& p_name = StringName(), double p_duration = -1.0, double p_custom_blend = -1, float p_custom_scale = 1.0, bool p_from_end = false, Tween::TransitionType p_trans_type = Tween::TRANS_LINEAR, Tween::EaseType p_ease_type = Tween::EASE_IN);
	void queue(const StringName& p_name);
	Vector<String> get_queue();
	void clear_queue();
	void pause();
	void stop(bool p_keep_state = false);
	bool is_playing_action() const;
	String get_current_animation() const;
	void set_current_animation(const String& p_animation);
	String get_assigned_animation() const;
	void set_assigned_animation(const String& p_animation);
	bool is_valid() const;

	void set_speed_scale(float p_speed);
	float get_speed_scale() const;
	float get_playing_speed() const;

	void set_autoplay(const String& p_name);
	String get_autoplay() const;

	void set_movie_quit_on_finish_enabled(bool p_enabled);
	bool is_movie_quit_on_finish_enabled() const;

	void seek_internal(double p_time, bool p_update = false, bool p_update_only = false, bool p_is_internal_seek = false);
	void seek(double p_time, bool p_update = false, bool p_update_only = false);

	double get_current_animation_position() const;
	double get_current_animation_length() const;

	void set_section_with_markers(const StringName& p_start_marker = StringName(), const StringName& p_end_marker = StringName());
	void set_section(double p_start_time = -1, double p_end_time = -1);
	void reset_section();

	double get_section_start_time() const;
	double get_section_end_time() const;
	bool has_section() const;

	virtual void advance(double p_time) override;


	// my stuff

public:

	Dictionary parameters = {};
	void set_parameters(const Dictionary& p_value) { parameters = p_value; };
	Dictionary get_parameters() { return parameters; };

	Ref<NAnimationMode> locomotion_anim_mode;
	Ref<NAnimationMode> override_anim_mode;
	void set_locomotion_mode(NAnimationMode* p_locomotion_mode);
	void set_override_mode(NAnimationMode* p_override_mode);


	Playback nanim_a;
	Playback nanim_b;
	float action_blend = 0.f;
	float locomotion_blend_a = 0.f;
	void set_nanim_a(const StringName& p_animation);
	void set_nanim_b(const StringName& p_animation);
	void set_nanim(Playback& p_playback, const StringName& p_animation);
	void set_locomotion_blend(float p_action_blend, float p_locomotion_blend_a);

	void my_blend_playback_data(double p_delta, bool p_just_started);
	bool my_blend_pre_process(double p_delta, int p_track_count, const AHashMap<NodePath, int>& p_track_map);

	// Make animation instances.
	virtual bool _blend_pre_process(double p_delta, int p_track_count, const AHashMap<NodePath, int>& p_track_map) override;
	virtual void _blend_capture(double p_delta) override;
	virtual void _blend_post_process() override;


	static void _bind_methods();

	NAnimationPlayer();
	~NAnimationPlayer();
};
