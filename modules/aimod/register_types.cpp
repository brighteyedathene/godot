/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"
#include "aimod.h"

VisibilitySystem *_visibility_system = nullptr;


void initialize_aimod_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}
	ClassDB::register_class<VisionQueryParameters>();

	_visibility_system = memnew(VisibilitySystem);
	ClassDB::register_class<VisibilitySystem>();
	Engine::get_singleton()->add_singleton(Engine::Singleton("VisibilitySystem", VisibilitySystem::get_singleton()));
}

void uninitialize_aimod_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}

	if (_visibility_system) {
		memdelete(_visibility_system);
	}
}
