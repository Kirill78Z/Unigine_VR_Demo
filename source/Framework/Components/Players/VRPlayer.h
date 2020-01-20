#pragma once
#include <UnigineNode.h>
#include <UnigineNodes.h>
#include <UnigineWorld.h>
#include <UniginePlayers.h>
#include <UniginePhysics.h>
#include <UnigineVector.h>
#include <UnigineGui.h>
#include <UnigineMap.h>
#include <UnigineGame.h>
#include <UnigineHashMap.h>

#include "../../../ComponentSystem/ComponentSystem.h"
#include "../VRInteractable.h"
#include <UnigineWidgets.h>



struct HotPoint
{
	HotPoint(Unigine::NodePtr hp, int index, int pageNum) {
		hotpoint = hp;
		_index = index;
		_pageNum = pageNum;
		name = hotpoint->getName();
		image = Unigine::Image::create("Images/default.jpg");

		int n = hotpoint->findProperty("hotpoint");
		if (n != -1)
		{
			Unigine::PropertyPtr prop = hotpoint->getProperty(n);
			n = prop->findParameter("Name");
			if (n != -1)
				name = prop->getParameterString(n);

			n = prop->findParameter("Image");
			if (n != -1) {
				Unigine::String fileName = prop->getParameterFile(n);
				if (Unigine::FileSystem::get()->isFileExist(fileName))
					image = Unigine::Image::create(fileName);
			}


			n = prop->findParameter("Teleport bound");
			if (n != -1) {
				Unigine::NodePtr node = prop->getParameterNode(n);
				if (node && node->isObject()) {
					teleport_bound = Unigine::Object::cast(node);
					tel_bound_def_transp = teleport_bound->getMaterialParameter("transparent", 0);
					ShowBound(false);
				}

			}

		}

		if (name.size() > 43) {
			//Unigine::String::substr()
			name = name.substr(0, 43);
			//name.append("...");
		}



	}

	void ShowBound(bool show) {
		if (!teleport_bound) return;

		if (show) {
			teleport_bound->setMaterialParameter("transparent", tel_bound_def_transp, 0);
		}
		else
		{
			teleport_bound->setMaterialParameter("transparent", Unigine::Math::vec4(0.0), 0);
		}
	}

	int _index;

	int _pageNum;

	Unigine::String name;

	Unigine::ImagePtr image;

	Unigine::NodePtr hotpoint;

	Unigine::ObjectPtr teleport_bound;

	Unigine::Math::vec4 tel_bound_def_transp = Unigine::Math::vec4::ZERO;

	Unigine::WidgetButtonPtr toggle;

	Unigine::WidgetHBoxPtr toggleContainer;

};


class VRInteractable;

class VRPlayer : public ComponentBase
{
public:
	COMPONENT(VRPlayer, ComponentBase);
	COMPONENT_INIT(init);
	COMPONENT_RENDER(render);
	COMPONENT_SHUTDOWN(shutdown);

	///////////////////////////////////////
	// enums
	///////////////////////////////////////

	enum {
		HAND_FREE,
		HAND_GRAB,
		HAND_HOLD,
		HAND_THROW
	};

	enum GRAB_MODE
	{
		BOUNDBOX,
		INTERSECTIONS
	};

	// baseline controls for VR Controllers (Vive, Oculus, XBox):
	// http://metanautvr.com/wp-content/uploads/2017/07/VRControllersBaselineComparison2C.png
	enum BUTTON
	{
		STICK_X,
		STICK_Y,
		TRIGGER,
		GRIP,
		XA, // X or A
		YB, // Y or B
		MENU,

		COUNT, // reserved. buttons count
	};

	///////////////////////////////////////
	// static
	///////////////////////////////////////

	// is -extern_plugin "AppVive/AppOculus" added to command line and successfully loaded?
	static int isVRPluginLoaded();

	// return pointer to last created PlayerVR
	static VRPlayer* get();

	///////////////////////////////////////
	// methods
	///////////////////////////////////////

	// sets intersection mask for teleportation (where player can be)
	virtual void setTeleportationMask(int mask) {}

	// player
	virtual Unigine::PlayerPtr getPlayer() { return Unigine::Game::get()->getPlayer(); }
	virtual void setLock(int lock) {}
	virtual void setPlayerPosition(const Unigine::Math::Vec3 &pos);

	// move player to position (and then land him to ground) with some direction
	virtual void landPlayerTo(const Unigine::Math::Vec3 &position, const Unigine::Math::vec3 &direction);
	UNIGINE_INLINE virtual void landPlayerTo(const Unigine::Math::Mat4 &transform)
	{
		landPlayerTo(transform.getColumn3(3), normalize(Unigine::Math::vec3(-transform.getColumn3(2))));
	}

	// head
	UNIGINE_INLINE virtual Unigine::NodePtr getHead() { return head->getNode(); }

	// hands
	virtual void setGrabMode(GRAB_MODE mode) { grab_mode = mode; } // grab via BoundBox-BoundBox or Line-Surface intersection?
	virtual int getNumHands() { return 0; }						// get count of hands
	virtual Unigine::NodePtr getHandNode(int num) = 0;			// get hand's node
	virtual int getHandDegreesOfFreedom(int num) = 0;	// get hand's degrees of freedom (0-5 for PC, 6 for vive)
	virtual Unigine::Math::vec3 getHandLinearVelocity(int num) = 0;	// get speed of hand
	virtual Unigine::Math::vec3 getHandAngularVelocity(int num) = 0;	// get angular speed of hand
	virtual int getHandState(int num) = 0;				// get hand state (grab, hold, throw)
	virtual const Unigine::NodePtr &getGrabNode(int num) const = 0;			// get object, grabbed by hand
	virtual const Unigine::Vector<VRInteractable*> &getGrabComponents(int num) const = 0;
	UNIGINE_INLINE virtual Unigine::Math::vec3 getHandyPos() { return Unigine::Math::vec3(0, -0.024f, 0.067f); }
	UNIGINE_INLINE virtual Unigine::Math::quat getHandyRot() { return Unigine::Math::quat(10.0f, 0, 0); }

	// hand's controllers
	virtual int getControllerButton(int controller_num, int button) = 0;
	virtual int getControllerButtonDown(int controller_num, int button) = 0;
	virtual int getControllerButtonUp(int controller_num, int button) = 0;
	virtual float getControllerAxis(int controller_num, int button) = 0;
	virtual void vibrateController(int controller_num, float amplitude = 1.0f) = 0;


	// callbacks
	// called when player holds "use" button when holding some object
	UNIGINE_INLINE void setUseCallback(Unigine::CallbackBase* callback) { delete use_callback; use_callback = callback; } // (NodePtr node)
	// called when player grab some object
	UNIGINE_INLINE void setGrabCallback(Unigine::CallbackBase* callback) { delete grab_callback; grab_callback = callback; } // (NodePtr node)
	// called when player holding some object
	UNIGINE_INLINE void setHoldCallback(Unigine::CallbackBase* callback) { delete hold_callback; hold_callback = callback; } // (NodePtr node)
	// called when player throw some object
	UNIGINE_INLINE void setThrowCallback(Unigine::CallbackBase* callback) { delete throw_callback; throw_callback = callback; } // (NodePtr node)

protected:
	void init();
	void render();
	void shutdown();

	virtual void update_button_states() = 0;

	// find specific components on the node
	int can_i_grab_it(const Unigine::NodePtr &node);
	int is_it_switcher(const Unigine::NodePtr &node);
	int is_it_handle(const Unigine::NodePtr &node);

	Unigine::NodeDummyPtr head;
	GRAB_MODE grab_mode = GRAB_MODE::BOUNDBOX;

	// navigation gui
	Unigine::GuiPtr gui;
	Unigine::WidgetSpritePtr background;
	Unigine::WidgetGridBoxPtr hBox;
	Unigine::WidgetGridBoxPtr menuVBox;
	Unigine::WidgetVBoxPtr pageVBox;
	const int btnsOnPageMaxCount = 12;
	Unigine::WidgetHBoxPtr controlsHBox;
	Unigine::WidgetButtonPtr nextPageBtn;
	Unigine::WidgetButtonPtr prevPageBtn;

	Unigine::HashMap<int, Unigine::Vector<HotPoint*>> pages;
	int currPage;
	int pageCount;

	//Unigine::WidgetScrollPtr testScroll;

	Unigine::WidgetHBoxPtr imageContainer;
	Unigine::WidgetSpritePtr image;


	//info gui
	const int info_gui_width = 1;
	const int info_gui_height = 1;
	const int info_gui_width_pixel = info_gui_width * 1000;
	const int info_gui_height_pixel = info_gui_height * 1000;

	Unigine::GuiPtr info_gui;
	Unigine::WidgetSpritePtr info_background;

	Unigine::WidgetLabelPtr info_text;
	Unigine::WidgetVBoxPtr infoVBox;
	Unigine::WidgetSpritePtr infoImage;
	Unigine::ImagePtr currentImg;
	Unigine::HashMap<unsigned int, Unigine::ImagePtr> cashedImgs;
	bool prev_show_info = false;


	Unigine::Vector<HotPoint*> hotpoints;


private:
	// singleton
	static VRPlayer* instance;

	// callbacks
	Unigine::CallbackBase* use_callback;
	Unigine::CallbackBase* grab_callback;
	Unigine::CallbackBase* hold_callback;
	Unigine::CallbackBase* throw_callback;





};