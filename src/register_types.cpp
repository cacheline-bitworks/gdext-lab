#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "example_class.h"
#include "frame_data.h"
#include "simulated_entity.h"
#include "tire_model.h"
#include "simulation_scheduler.h"
#include "vehicle_body_t1.h"
#include "tire_model_v2.h"
#include "vehicle_body_t2a.h"

using namespace godot;

void initialize_gdextension_types(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(ExampleClass);
	GDREGISTER_CLASS(SimulatedEntity);
	GDREGISTER_CLASS(SimulationScheduler);
	GDREGISTER_CLASS(FrameData);
    GDREGISTER_ABSTRACT_CLASS(TireModel);
	GDREGISTER_CLASS(VehicleBodyT1);
	GDREGISTER_CLASS(TireModelV1);
	GDREGISTER_CLASS(TireModelV2);
	GDREGISTER_CLASS(VehicleBodyT2a);
}

void uninitialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C"
{
	GDExtensionBool GDE_EXPORT gdext_lab_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
	{
		GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
		init_obj.register_initializer(initialize_gdextension_types);
		init_obj.register_terminator(uninitialize_gdextension_types);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

		return init_obj.init();
	}
}