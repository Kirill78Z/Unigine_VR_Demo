/* Copyright (C) 2005-2018, UNIGINE. All rights reserved.
 *
 * This file is a part of the UNIGINE 2.7.3.1 SDK.
 *
 * Your use and / or redistribution of this software in source and / or
 * binary form, with or without modification, is subject to: (i) your
 * ongoing acceptance of and compliance with the terms and conditions of
 * the UNIGINE License Agreement; and (ii) your inclusion of this notice
 * in any version of this software that you use or redistribute.
 * A copy of the UNIGINE License Agreement is available by contacting
 * UNIGINE. at http://unigine.com/
 */


#include "AppWorldLogic.h"
#include <UnigineGui.h>
#include <UnigineEditor.h>
#include <UnigineGame.h>
#include <UnigineFileSystem.h>
#include <UnigineWorlds.h>
#include <UnigineConsole.h>
#include <UnigineWorlds.h>
 // World logic, it takes effect only when the world is loaded.
 // These methods are called right after corresponding world script's (UnigineScript) methods.

AppWorldLogic::AppWorldLogic() {

}

AppWorldLogic::~AppWorldLogic() {

}

int AppWorldLogic::init() {
	// Write here code to be called on world initialization: initialize resources for your world scene during the world start.


	//settings window
	Unigine::GuiPtr gui = Unigine::Gui::get();
	settingsWindow = Unigine::WidgetWindow::create(gui);
	gui->addChild(settingsWindow->getWidget(),
		Unigine::Gui::ALIGN_OVERLAP | Unigine::Gui::ALIGN_BOTTOM | Unigine::Gui::ALIGN_LEFT);


	trafficHBox = Unigine::WidgetHBox::create(gui);
	settingsWindow->addChild(trafficHBox->getWidget(), Unigine::Gui::ALIGN_LEFT);

	trafficLbl = Unigine::WidgetLabel::create(gui, "Traffic");
	trafficHBox->addChild(trafficLbl->getWidget(), Unigine::Gui::ALIGN_LEFT);

	trafficCheckBox = Unigine::WidgetCheckBox::create(gui);
	trafficCheckBox->setChecked(1);
	trafficCheckBox->addCallback(Unigine::Gui::CHANGED, MakeCallback(this, &AppWorldLogic::enableTraffic));
	trafficHBox->addChild(trafficCheckBox->getWidget(), Unigine::Gui::ALIGN_RIGHT);






	Unigine::Vector<Unigine::String> splToLoad;
	Unigine::FileSystem* fs = Unigine::FileSystem::get();
	Unigine::Vector<Unigine::String> files;
	fs->getVirtualFiles(files);
	for (int f = 0; f < files.size(); f++) {
		Unigine::String fname = files[f];
		Unigine::String dir = fname.dirname();
		Unigine::String ext = fname.extension();
		if (dir == "LoadingSplineGraphs/" && ext == "spl") {
			splToLoad.append(fname);
		}
	}

	if (splToLoad.size() > 0) {
		//Загрузка сплайнов
		for (int f = 0; f < splToLoad.size(); f++) {
			Unigine::String fname = splToLoad[f];
			Unigine::StringArray<> coordStrs = Unigine::String::split(fname.filename(), "_");
			if (coordStrs.size() != 3) continue;
			Unigine::String xstr = coordStrs[0];
			double x = Unigine::String::atod(xstr);
			Unigine::String ystr = coordStrs[1];
			double y = Unigine::String::atod(ystr);
			Unigine::String zstr = coordStrs[2];
			double z = Unigine::String::atod(zstr);

			Unigine::WorldSplineGraphPtr splGr = Unigine::WorldSplineGraph::create(fname);
			//if (splGr->load(fname) != 1) continue;
			splGr->setName(fname.filename());

			splGr->release();
			Unigine::Editor::get()->addNode(splGr->getNode(), 0);

			splGr->setWorldParent(Unigine::NodePtr::Ptr());

			splGr->setWorldPosition(Unigine::Math::dvec3(x, y, z));
		}


		Unigine::Console::get()->run("world_save");
		Unigine::Console::get()->flush();

	}


	//traffic
	trafficSimulation = new TrafficSimulation;


	//simple movement
	//TODO: отвести для траекторий отдельную специальную папку в world
	Unigine::WorldSplineGraphPtr test_path = Unigine::WorldSplineGraph
		::cast(Unigine::Editor::get()->getNodeByName("testPath"));
	MovementPath* testPath = new MovementPath(test_path);
	movementPaths.append(testPath);

	return 1;
}

void AppWorldLogic::enableTraffic() {
	Unigine::NodePtr trafficNode = Unigine::Editor::get()->getNodeByName("Traffic");
	if (!trafficNode)
		trafficNode = Unigine::Editor::get()->getNodeByName("traffic");

	if (!trafficNode) return;
	if (trafficCheckBox->isChecked())
		trafficNode->setEnabled(1);
	else trafficNode->setEnabled(0);
}


// start of the main loop
int AppWorldLogic::update() {
	// Write here code to be called before updating each render frame: specify all graphics-related functions you want to be called every frame while your application executes.
	trafficSimulation->update();

	for (MovementPath* path : movementPaths) {
		path->update();
	}
	return 1;
}

int AppWorldLogic::render() {
	// The engine calls this function before rendering each render frame: correct behavior after the state of the node has been updated.

	return 1;
}

int AppWorldLogic::flush() {
	// Write here code to be called before updating each physics frame: control physics in your application and put non-rendering calculations.
	// The engine calls flush() with the fixed rate (60 times per second by default) regardless of the FPS value.
	// WARNING: do not create, delete or change transformations of nodes here, because rendering is already in progress.

	return 1;
}
// end of the main loop

int AppWorldLogic::shutdown() {
	// Write here code to be called on world shutdown: delete resources that were created during world script execution to avoid memory leaks.

	delete trafficSimulation;

	return 1;
}

int AppWorldLogic::destroy() {
	// Write here code to be called when the video mode is changed or the application is restarted (i.e. video_restart is called). It is used to reinitialize the graphics context.

	return 1;
}

int AppWorldLogic::save(const Unigine::StreamPtr &stream) {
	// Write here code to be called when the world is saving its state (i.e. state_save is called): save custom user data to a file.

	UNIGINE_UNUSED(stream);
	return 1;
}

int AppWorldLogic::restore(const Unigine::StreamPtr &stream) {
	// Write here code to be called when the world is restoring its state (i.e. state_restore is called): restore custom user data to a file here.

	UNIGINE_UNUSED(stream);
	return 1;
}
