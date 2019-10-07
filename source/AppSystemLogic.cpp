#include "AppSystemLogic.h"
#include <UnigineConsole.h>
#include "ComponentSystem/ComponentSystem.h"

int AppSystemLogic::init()
{
	// run ComponentSystem
	ComponentSystem::get()->initialize();

	// show all warning/error messages to the log
	ComponentSystem::get()->setWarningLevel(ComponentSystem::WARNING_LEVEL::HIGH);

	return 1;
}

