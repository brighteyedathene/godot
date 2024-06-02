/* register_types.cpp */

#include "register_types.h"

#include "core/object/class_db.h"
#include "aimod.h"

void initialize_aimod_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}
	ClassDB::register_class<VisionQueryParameters>();
	ClassDB::register_class<VisibilityComponent>();
	ClassDB::register_class<EyeComponent>();
	ClassDB::register_class<VisibilitySystem>();
}

void uninitialize_aimod_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
			return;
	}
   // Nothing to do here in this example.
}
