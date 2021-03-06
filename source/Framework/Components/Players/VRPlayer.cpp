#include "VRPlayer.h"

#include <UnigineGame.h>
#include <UnigineEditor.h>
#include <UnigineEngine.h>
#include <UnigineApp.h>

#include "../Objects/ObjSwitch.h"
#include "../Objects/ObjHandle.h"

using namespace Unigine;
using namespace Math;

VRPlayer* VRPlayer::instance;

int VRPlayer::isVRPluginLoaded()
{
	return (Engine::get()->findPlugin("AppVive") != -1 || Engine::get()->findPlugin("AppOculus") != -1);
}

VRPlayer* VRPlayer::get()
{
	return instance;
}

void VRPlayer::init()
{
	// callbacks
	use_callback = nullptr;
	grab_callback = nullptr;
	hold_callback = nullptr;
	throw_callback = nullptr;

	// run application even when unfocused
	App::get()->setUpdate(1);

	instance = this;

}

void VRPlayer::render()
{
	for (int i = 0; i < getNumHands(); i++)
	{
		int hand_state = getHandState(i);
		if (hand_state != HAND_FREE)
		{
			auto &components = getGrabComponents(i);
			switch (hand_state)
			{
			case HAND_GRAB:
				for (int j = 0; j < components.size(); j++)
					components[j]->grabIt(this, i);
				if (grab_callback) grab_callback->run(getGrabNode(i));
				break;
			case HAND_HOLD:
				for (int j = 0; j < components.size(); j++)
					components[j]->holdIt(this, i);
				if (hold_callback) hold_callback->run(getGrabNode(i));
				break;
			case HAND_THROW:
				for (int j = 0; j < components.size(); j++)
					components[j]->throwIt(this, i);
				if (throw_callback) throw_callback->run(getGrabNode(i));
				break;
			default:
				break;
			}

			// use holded object
			if (getControllerButtonDown(i, BUTTON::TRIGGER))
			{
				for (int j = 0; j < components.size(); j++)
					components[j]->useIt(this, i);
				if (use_callback) use_callback->run(getGrabNode(i));
			}
		}
	}
	update_button_states();
}

void VRPlayer::shutdown()
{
	// callbacks
	delete use_callback;
	delete grab_callback;
	delete hold_callback;
	delete throw_callback;

	instance = nullptr;

	if (gui)
	{
		for (int i = 0; i < hotpoints.size(); i++) {
			pageVBox->removeChild(hotpoints[i]->toggle->getWidget());
		}

		menuVBox->removeChild(pageVBox->getWidget());
		controlsHBox->removeChild(nextPageBtn->getWidget());
		controlsHBox->removeChild(prevPageBtn->getWidget());
		menuVBox->removeChild(controlsHBox->getWidget());
		hBox->removeChild(menuVBox->getWidget());
		imageContainer->removeChild(image->getWidget());
		hBox->removeChild(imageContainer->getWidget());
		gui->removeChild(hBox->getWidget());
		gui->removeChild(background->getWidget());

	}

}

void VRPlayer::setPlayerPosition(const Vec3 &pos)
{
	getPlayer()->setPosition(pos);
}

void VRPlayer::landPlayerTo(const Vec3 &position, const vec3 &direction)
{
	Vec3 pos1 = position;
	Vec3 pos2 = position + Vec3(0, 0, -1) * getPlayer()->getZFar();
	WorldIntersectionPtr intersection = WorldIntersection::create();
	ObjectPtr hitObj = World::get()->getIntersection(pos1, pos2, 1, intersection);
	if (hitObj)
		getPlayer()->setWorldTransform(setTo(intersection->getPoint(), intersection->getPoint() + Vec3(direction), vec3::UP));
}

int VRPlayer::can_i_grab_it(const NodePtr &node)
{
	return ComponentSystem::get()->getComponent<VRInteractable>(node) != nullptr;
}

int VRPlayer::is_it_switcher(const NodePtr &node)
{
	return ComponentSystem::get()->getComponent<ObjSwitch>(node) != nullptr;
}

int VRPlayer::is_it_handle(const NodePtr &node)
{
	return ComponentSystem::get()->getComponent<ObjHandle>(node) != nullptr;
}





