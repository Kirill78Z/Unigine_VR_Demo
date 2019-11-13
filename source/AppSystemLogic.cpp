#include "AppSystemLogic.h"
#include <UnigineConsole.h>
#include "ComponentSystem/ComponentSystem.h"
#include <UnigineSplash.h>
#include <UnigineMathLib.h>
#include <UnigineFileSystem.h>

int AppSystemLogic::init()
{
	ConfigureSplashScreen();

	// run ComponentSystem
	ComponentSystem::get()->initialize();

	// show all warning/error messages to the log
	ComponentSystem::get()->setWarningLevel(ComponentSystem::WARNING_LEVEL::HIGH);

	return 1;
}

void AppSystemLogic::ConfigureSplashScreen() {
	Unigine::Splash::get()->setWorldBackground(Unigine::Math::vec4::ZERO);
	Unigine::Splash::get()->setSplashBackground(Unigine::Math::vec4::ZERO);
	Unigine::Splash::get()->setSystemBackground(Unigine::Math::vec4::ZERO);

	Unigine::Splash::get()->setWorld(0);
	Unigine::Splash::get()->setSplash(0);
	Unigine::Splash::get()->setSystem(0);
	Unigine::Splash::get()->setWorldText(0);
	Unigine::Splash::get()->setSplashText(0);
	Unigine::Splash::get()->setSystemText(0);
}