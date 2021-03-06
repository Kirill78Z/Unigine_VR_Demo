#include "VRPlayerVR.h"
#include <UnigineRender.h>
#include <UnigineConfig.h>
#include <UnigineSounds.h>
#include "../../Utils.h"
#include "../Objects/ObjMovable.h"

using namespace Unigine;
using namespace Math;

const vec4 TELEPORT_COLOR_ENABLED(1.0f);
const vec4 TELEPORT_COLOR_DISABLED(1.0f, 0.0f, 0.0f, 1.0f);

void VRPlayerVR::setTeleportationMask(int mask)
{
	teleportation_mask = mask;
}

PlayerPtr VRPlayerVR::getPlayer()
{
	return player->getPlayer();
}

int VRPlayerVR::getNumHands()
{
	return CONTROLLER_COUNT;
}

NodePtr VRPlayerVR::getHandNode(int num)
{
	return controller[num]->getNode();
}

int VRPlayerVR::getHandDegreesOfFreedom(int num)
{
	return 6; // x, y, z + pitch, yaw, roll
}

int VRPlayerVR::getHandState(int num)
{
	return hand_state[num];
}

const NodePtr &VRPlayerVR::getGrabNode(int num) const
{
	return grab_node[num];
}

const Vector<VRInteractable*> &VRPlayerVR::getGrabComponents(int num) const
{
	return grab_components[num];
}

vec3 VRPlayerVR::getHandLinearVelocity(int num)
{
	return linearRegression(hand_linear_velocity[num]);
}

vec3 VRPlayerVR::getHandAngularVelocity(int num)
{
	return linearRegression(hand_angular_velocity[num]);
}

int VRPlayerVR::isControllerValid(int controller_num)
{
	return controller_valid[controller_num];
}

int VRPlayerVR::getControllerButtonDown(int controller_num, int button)
{
	return getControllerButton(controller_num, button) && !buttons_prev_state[controller_num][button];
}

int VRPlayerVR::getControllerButtonUp(int controller_num, int button)
{
	return !getControllerButton(controller_num, button) && buttons_prev_state[controller_num][button];
}


void VRPlayerVR::update_button_states()
{
	for (int k = 0; k < CONTROLLER_COUNT; k++)
		for (int i = 0; i < BUTTON::COUNT; i++)
			buttons_prev_state[k][i] = getControllerButton(k, i);
}

void VRPlayerVR::init_player()
{
	player = PlayerDummy::cast(node);
	Game::get()->setPlayer(player->getPlayer());

	head = NodeDummy::create();

	xpad360 = ControlsXPad360::create(0);

	triggers.init(obstacles, triggers_scale);

	Sound::get()->setHRTF(1);


	hotpoints_init("vr_hotpoints");
}

void VRPlayerVR::load_player_height(float default_value)
{
	Config *c = Config::get();
	if (c->isExist("player_height"))
		player_height = c->getFloat("player_height");
	else
		player_height = default_value;

	player_height = clamp(player_height, -5.0f, 5.0f);
}

void VRPlayerVR::save_player_height()
{
	Config *c = Config::get();
	c->setFloat("player_height", player_height);
	c->flush();
}

void VRPlayerVR::put_head_to_position(const mat4 &hmd_transform, const Vec3 &pos)
{
	Mat4 player_transform = player->getWorldTransform();
	Mat4 hmd_transform_world = player_transform * Mat4(hmd_transform);
	Vec3 head_offset =
		player_transform.getTranslate() - hmd_transform_world.getTranslate();
	head_offset.z = player_height;

	player->setWorldPosition(pos + head_offset);
}

void VRPlayerVR::land_player_to(const mat4 &hmd_transform, const Vec3 &position, const vec3 &direction)
{
	// rotate player
	vec3 dir = direction;
	dir.z = 0;
	dir = normalize(dir);
	quat rot = conjugate(quat(lookAt(Vec3(0, 0, 0), Vec3(dir), vec3(0, 0, 1))));
	vec3 angles = decomposeRotationYXZ(mat3(hmd_transform));
	player->setWorldRotation(rot * quat(0.0f, -angles.y, 0.0f));

	// move player
	Mat4 player_transform = player->getWorldTransform();
	Mat4 hmd_transform_world = player_transform * Mat4(hmd_transform);
	Vec3 head_offset = Vec3(0, 0, 0);
	head_offset = player_transform.getTranslate() - hmd_transform_world.getTranslate();
	head_offset.z = player_height;

	// ground player
	Vec3 pos1 = position;
	Vec3 pos2 = position + Vec3(0, 0, -1) * player->getZFar();
	ObjectPtr hitObj = World::get()->getIntersection(pos1, pos2, teleportation_mask, intersection);
	if (hitObj)
		player->setWorldPosition(intersection->getPoint() + head_offset);

	//��������� ������ � �������
	//�� �������� ��� ��� ��������
	if (hitObj) {
		player->setWorldParent(hitObj->getNode());
		if (object_gui)
			object_gui->setWorldParent(hitObj->getNode());
		if(info_object_gui)
			info_object_gui->setWorldParent(hitObj->getNode());
	}
	else
	{
		player->setWorldParent(Unigine::NodePtr::Ptr());
		if (object_gui)
			object_gui->setWorldParent(Unigine::NodePtr::Ptr());
		if (info_object_gui)
			info_object_gui->setWorldParent(Unigine::NodePtr::Ptr());
	}
}

void VRPlayerVR::move_update(const Mat4 &world_head_transform)
{
	float ifps = Game::get()->getIFps() / Game::get()->getScale();

	xpad360->updateEvents();
	if (xpad360->isAvailable())
	{
		// moving
		float lx = xpad360->getLeftX();
		float ly = xpad360->getLeftY();
		if (Math::abs(lx) < dead_zone) lx = 0;
		if (Math::abs(ly) < dead_zone) ly = 0;

		if (lx != 0 || ly != 0)
		{
			if (!is_moving)
			{
				head_dir = world_head_transform.getRotate();
				is_moving = true;
			}
			vec3 move_dir = head_dir * vec3(lx, 0, -ly);
			move_dir.z = 0;
			player->setWorldPosition(player->getPosition() + Vec3(move_dir) * ifps);
		}
		else
			is_moving = false;


		// rotation
		float rx = xpad360->getRightX();
		if (Math::abs(rx) > dead_zone)
		{
			discrete_timer -= Game::get()->getIFps() / Game::get()->getScale();
			if (discrete_timer < 0)
			{
				/*
				// slow motion on small values and fast motion on high values
				rx = UNIGINE_PI * Math::sign(rx) * (Math::sin(rx * UNIGINE_PI - UNIGINE_PI * 0.5f) + 1.0f) * 0.5f;
				quat rot = quat(0, 0, -rx * 45.0f * step);
				*/

				quat rot = quat(0, 0, -Math::sign(rx) * 75.0f * step);
				Vec3 offset0 = world_head_transform.getTranslate() - player->getWorldPosition();
				Vec3 offset1 = rot * offset0;
				player->setWorldPosition(player->getPosition() + offset0 - offset1);
				player->setWorldRotation(rot * player->getRotation());

				// apply rotation to moving too
				head_dir = rot * head_dir;

				discrete_timer = step;
			}
		}
		else
			discrete_timer = 0;
	}
}

void VRPlayerVR::collisions_update(const Mat4 &head_transform, const Vec3 &offset)
{
	// triggers
	if (triggers.isInside(head_transform.getTranslate()))
	{
		float depth = triggers.getDepth();
		depth = clamp(depth / triggers_scale, 0.0f, 1.0f);
		fade_alpha = depth;
		vibrateController(0, depth);
		vibrateController(1, depth);

		if (trigger_timer <= 0)
		{
			before_collision_dir = player->getPosition() - offset - before_collision_point;
			before_collision_dir.z = 0;
			before_collision_dir = normalize(before_collision_dir);
		}

		trigger_timer += Game::get()->getIFps() / Game::get()->getScale();
		if (trigger_timer > black_screen_max_sec)
		{
			// teleport to last point, where was no collisions
			player->setWorldPosition(before_collision_point - before_collision_dir + offset);
		}
	}
	else
	{
		fade_alpha = moveTowards(fade_alpha, 0, Game::get()->getIFps() * 2.0f / Game::get()->getScale());
		vibrateController(0, 0);
		vibrateController(1, 0);
		trigger_timer = 0;
		before_collision_point = player->getPosition() - offset;
	}

	Render::get()->setFadeColor(vec4(0, 0, 0, fade_alpha));
}

void VRPlayerVR::controllers_init()
{
	controller[0] = NodeReference::cast(controller_0);
	controller[1] = NodeReference::cast(controller_1);

	hand_linear_velocity.clear();
	hand_angular_velocity.clear();

	for (int i = 0; i < CONTROLLER_COUNT; i++)
	{
		controller_ref[i] = controller[i]->getReference();

		controller_obj[i].clear();
		find_obj_in_children(controller_ref[i], &controller_obj[i]);

		Vector<vec3> lv;
		hand_linear_velocity.append(lv);

		Vector<vec3> av;
		hand_angular_velocity.append(av);
	}

}

#pragma region GUI



void VRPlayerVR::gui_init() {
#pragma region Navigation gui
	// GUI near eyes
	//ObjectGui::cast(controller[0]->getChild(gui_index))->setEnabled(0);

	object_gui = ObjectGui::create(4, 2);
	object_gui->setScreenSize(4000, 2000);
	object_gui->setBackground(0);
	object_gui->setMouseMode(ObjectGui::MOUSE_VIRTUAL);
	object_gui->setEnabled(0);

	object_gui->setDepthTest(0);

	object_gui->setIntersectionMask(2, 0);

	if (player)
		object_gui->setWorldParent(player->getParent());


	gui = object_gui->getGui();

	// create background
	if (gui != Gui::get())
	{
		background = WidgetSprite::create(gui, "black.dds");
		background->setColor(Math::vec4(1, 1, 1, 0.25f));
		gui->addChild(background->getWidget(), Gui::ALIGN_BACKGROUND | Gui::ALIGN_EXPAND);
	}




	//create main HBox
	hBox = Unigine::WidgetGridBox::create(gui, 2);
	gui->addChild(hBox->getWidget(), Gui::ALIGN_OVERLAP | Gui::ALIGN_EXPAND);




	//create menu vbox
	menuVBox = Unigine::WidgetGridBox::create(gui, 1);
	menuVBox->setWidth(1500);
	menuVBox->setHeight(gui->getHeight() - 30);

	hBox->addChild(menuVBox->getWidget(), Gui::ALIGN_LEFT | Gui::ALIGN_TOP);

	//create page vbox
	pageVBox = Unigine::WidgetVBox::create(gui);
	pageVBox->setWidth(1500);
	pageVBox->setHeight(gui->getHeight() - 100);
	menuVBox->addChild(pageVBox->getWidget(), Gui::ALIGN_LEFT | Gui::ALIGN_TOP);

	//create controls hbox with buttons
	controlsHBox = Unigine::WidgetHBox::create(gui, 0, 0);
	controlsHBox->setWidth(1500);
	controlsHBox->setHeight(100);
	//controlsHBox->setColor(vec4(1, 0, 0, 1));

	menuVBox->addChild(controlsHBox->getWidget(), Gui::ALIGN_LEFT | Gui::ALIGN_BOTTOM);

	prevPageBtn = Unigine::WidgetButton::create(gui, "<");
	prevPageBtn->setFontSize(96);
	prevPageBtn->addCallback(Gui::CLICKED, MakeCallback(this, &VRPlayerVR::prevPage));
	controlsHBox->addChild(prevPageBtn->getWidget(), Gui::ALIGN_RIGHT | Gui::ALIGN_BOTTOM);

	nextPageBtn = Unigine::WidgetButton::create(gui, ">");
	nextPageBtn->setFontSize(96);
	nextPageBtn->addCallback(Gui::CLICKED, MakeCallback(this, &VRPlayerVR::nextPage));
	controlsHBox->addChild(nextPageBtn->getWidget(), Gui::ALIGN_RIGHT | Gui::ALIGN_BOTTOM);


	//create toggles
	for (int i = 0; i < hotpoints.size(); i++) {
		create_hotpoint_toggle(hotpoints[i]);
	}

	//image container
	imageMaxWidth = gui->getWidth() - menuVBox->getWidth();
	imageMaxHeight = gui->getHeight();
	imageContainer = WidgetHBox::create(gui);
	imageContainer->setWidth(imageMaxWidth);
	imageContainer->setHeight(imageMaxHeight);
	hBox->addChild(imageContainer->getWidget(), Gui::ALIGN_CENTER | Gui::ALIGN_EXPAND);

	//create image
	image = WidgetSprite::create(gui);

	image->setWidth(imageMaxWidth);
	image->setHeight(imageMaxHeight);
	imageContainer->addChild(image->getWidget(), Gui::ALIGN_CENTER);

	//set current hotpoint 0
	if (hotpoints.size() > 0) {
		setCurrHotpoint(0);
		setPage(0);
	}

	if (pageCount == 1) {
		nextPageBtn->setEnabled(0);
	}
#pragma endregion

#pragma region Information gui
	info_object_gui = ObjectGui::create(info_gui_physical_width(), info_gui_physical_height());
	info_object_gui->setScreenSize(info_gui_width_pixel, info_gui_height_pixel);
	info_object_gui->setBackground(0);
	info_object_gui->setMouseMode(ObjectGui::MOUSE_VIRTUAL);
	info_object_gui->setEnabled(0);

	info_object_gui->setDepthTest(0);

	info_object_gui->setIntersectionMask(2, 0);

	if (player)
		info_object_gui->setWorldParent(player->getParent());

	info_gui = info_object_gui->getGui();

	// create background
	if (info_gui != Gui::get())
	{
		info_background = WidgetSprite::create(info_gui, "black.dds");
		info_background->setColor(Math::vec4(1, 1, 1, 0.25f));
		info_gui->addChild(info_background->getWidget(), Gui::ALIGN_BACKGROUND | Gui::ALIGN_EXPAND);
	}

	//container
	infoBox = WidgetGridBox::create(info_gui, 1);
	infoBox->setWidth(info_gui_width_pixel);
	infoBox->setHeight(info_gui_height_pixel);
	infoBox->setPadding(50, 50, 50, 50);
	info_gui->addChild(infoBox->getWidget(), Gui::ALIGN_OVERLAP | Gui::ALIGN_EXPAND);


	//image
	infoImage = WidgetSprite::create(info_gui);
	infoImage->setWidth(info_gui_width_pixel / 2);
	infoImage->setHeight(info_gui_height_pixel / 4);
	infoBox->addChild(infoImage->getWidget(), Gui::ALIGN_LEFT | Gui::ALIGN_TOP);

	//text
	info_text = WidgetLabel::create(info_gui, "");
	info_text->setWidth(info_gui_width_pixel * 0.95);
	info_text->setFontWrap(1);
	info_text->setFontSize(36);
	infoBox->addChild(info_text->getWidget(), Gui::ALIGN_LEFT | Gui::ALIGN_TOP);

#pragma endregion

}

void VRPlayerVR::set_image(Unigine::ImagePtr img) {
	if (!image) return;

	//set aspect ratio of WidgetSprite same as Image
	float w, h;

	fitImage(img, w, h, imageMaxWidth, imageMaxHeight);

	image->setWidth(w);
	image->setHeight(h);

	image->setImage(img);
	image->arrange();
	hBox->arrange();
}

void VRPlayerVR::fitImage(Unigine::ImagePtr &img, float &w, float &h, int maxWidth, int maxHeight)
{
	int imgWidth = img->getWidth();
	int imgHeight = img->getHeight();

	w = imgWidth;
	h = imgHeight;

	float imgAspect = (float)imgWidth / imgHeight;
	float spaceAspect = (float)maxWidth / maxHeight;

	if (imgAspect > spaceAspect) {
		w = maxWidth;
		h = (float)maxWidth / imgAspect;
	}
	else
	{
		h = maxHeight;
		w = (float)imgAspect *maxHeight;
	}
}


void VRPlayerVR::create_hotpoint_toggle(HotPoint* hotpt) {
	hotpt->toggleContainer = WidgetHBox::create(gui);


	hotpt->toggle = WidgetButton::create(gui, hotpt->name);
	//hotpt->toggle->setToggleable(1);
	hotpt->toggle->setFontSize(96);
	hotpt->toggle->setWidth(1500);
	hotpt->toggle->setHeight(150);
	//pageVBox->addChild(hotpt->toggle->getWidget(), Gui::ALIGN_LEFT);

	//callback
	hotpt->toggle->addCallback(Gui::CLICKED, MakeCallback(this, &VRPlayerVR::setCurrHotpoint, hotpt->_index));


	hotpt->toggleContainer->addChild(hotpt->toggle->getWidget());

}

void VRPlayerVR::setPage(int pageNum) {
	if (currPage == pageNum || !pageVBox) return;

	//clear pageVBox
	Unigine::Vector<HotPoint*> hps = pages[currPage];
	for (int i = 0; i < hps.size(); i++)
		pageVBox->removeChild(hps[i]->toggleContainer->getWidget());

	//populate pageVBox
	hps = pages[pageNum];
	for (int i = 0; i < hps.size(); i++)
		pageVBox->addChild(hps[i]->toggleContainer->getWidget());

	currPage = pageNum;

	if (currPage == pageCount - 1) nextPageBtn->setEnabled(0);
	else nextPageBtn->setEnabled(1);

	if (currPage == 0) prevPageBtn->setEnabled(0);
	else prevPageBtn->setEnabled(1);
}

void VRPlayerVR::nextPage()
{
	if (pageCount == 1) return;

	int pageNum = currPage + 1;
	if (pageNum >= pageCount) return;

	setPage(pageNum);
}

void VRPlayerVR::prevPage()
{
	if (pageCount == 1) return;

	int pageNum = currPage - 1;
	if (pageNum < 0) return;

	setPage(pageNum);
}


void VRPlayerVR::turn_page_update(int button_prev, int button_next) {
	if (pageCount == 1)
		return;

	if (button_prev)
		prevPage();

	if (button_next)
		nextPage();
}

void VRPlayerVR::refresh_current_page() {
	//clear pageVBox
	Unigine::Vector<HotPoint*> hps = pages[currPage];
	for (int i = 0; i < hps.size(); i++)
		pageVBox->removeChild(hps[i]->toggleContainer->getWidget());

	//populate pageVBox
	for (int i = 0; i < hps.size(); i++)
		pageVBox->addChild(hps[i]->toggleContainer->getWidget());
}

void VRPlayerVR::update_gui()
{
	int menu_btn_down = -1;
	if (getControllerButtonDown(0, BUTTON::MENU))
		menu_btn_down = 0;
	if (getControllerButtonDown(1, BUTTON::MENU))
		menu_btn_down = 1;

	if (menu_btn_down == 0 || menu_btn_down == 1)
	{
		object_gui->setEnabled(1 - object_gui->isEnabled());

		if (!object_gui->isEnabled())
		{
			int pressed = 0;
			for (int i = 0; i < CONTROLLER_COUNT; i++)
				pressed |= teleport_button_pressed[i];

			teleport_ray->setEnabled(pressed);
		}
		else
		{
			controller_menu_btn_down = menu_btn_down;
			// put GUI around player
			object_gui->setWorldTransform(gui_near_eyes_pos()* Mat4(translate(0.0f, player_height, -2.0f)));

			refresh_current_page();

			unhighlight();
		}
	}

	if (!object_gui->isEnabled())
		return;

	int hit_gui = 0;

	// update visuals (we are using teleport ray)


	int c = controller_menu_btn_down;

	Vec3 p0 = controller[c]->getWorldPosition();
	mat4 m = mat4(controller[c]->getWorldTransform());
	Vec3 p1 = controller[c]->getWorldPosition() + Vec3(controller[c]->getWorldDirection(Math::AXIS_Y)) * player->getZFar();


	Vec3 p1_end = p1;
	ObjectPtr hitObj = World::get()->getIntersection(p0, p1, 2, intersection);
	if (hitObj)
	{
		p1_end = intersection->getPoint();
		if (hitObj->getID() == object_gui->getID())
			hit_gui = 1;
	}

	teleport_ray->clearVertex();
	teleport_ray->clearIndices();
	dmat4 transf = teleport_ray->getIWorldTransform();
	vec3 from = vec3(transf*p0);
	vec3 to = vec3(transf*p1_end);
	vec3 from_right = cross(normalize(to - from), vec3(
		normalize(vec3(transf*head->getWorldPosition()) - from)));
	vec3 to_right = cross(normalize(to - from), vec3(
		normalize(vec3(transf*head->getWorldPosition()) - to)));
	addLineSegment(teleport_ray, from, to, from_right, to_right, 0.01f);
	teleport_ray->updateBounds();
	teleport_ray->updateTangents();
	teleport_ray->flushVertex();
	teleport_ray->flushIndices();
	teleport_ray->setEnabled(1);

	// update gui
	object_gui->setMouse(p0, p1, hit_gui * getControllerButton(c, BUTTON::TRIGGER), 0);
}


void VRPlayerVR::update_teleport_ray_visibility() {
	teleport_ray->setEnabled(object_gui->isEnabled()
		|| info_button_pressed[0] || info_button_pressed[1]
		|| teleport_button_pressed[0] || teleport_button_pressed[1]);
}


void VRPlayerVR::update_info_ray_shooting(int num, int button_pressed) {
	// check if nobody using our info ray
	if (button_pressed &&
		!info_button_pressed[num] &&
		info_button_pressed[1 - num]) // if controllers > 2 it doesn't work!
		return;

	if (!button_pressed && !info_button_pressed[num])
		return;

	if (object_gui->isEnabled())
		return;

	unhighlight();

	int last_button_state = info_button_pressed[num];
	info_button_pressed[num] = button_pressed;


	Vec3 pos1 = controller[num]->getWorldPosition();
	Vec3 pos2 = controller[num]->getWorldPosition() + Vec3(controller[num]->getWorldDirection(Math::AXIS_Y)) * player->getZFar();

	ObjectPtr hitObj = World::get()->getIntersection(pos1, pos2, teleportation_mask, intersection);
	if (hitObj) {
		//highlight obj
		highlight(hitObj);
	}



	// show ray
	if (button_pressed) {
		teleport_ray->clearVertex();
		teleport_ray->clearIndices();

		dmat4 transf = teleport_ray->getIWorldTransform();
		vec3 from = vec3(transf * pos1);
		vec3 to = vec3(transf * pos2);
		vec3 from_right = cross(normalize(to - from), vec3(
			normalize(vec3(transf*head->getWorldPosition()) - from)));
		vec3 to_right = cross(normalize(to - from), vec3(
			normalize(vec3(transf*head->getWorldPosition()) - to)));

		addLineSegment(teleport_ray, from, to, from_right, to_right, 0.01f);
		teleport_ray->updateBounds();
		teleport_ray->updateTangents();
		teleport_ray->flushVertex();
		teleport_ray->flushIndices();
		teleport_ray->setEnabled(1);
	}



}
#pragma endregion


void VRPlayerVR::find_obj_in_children(const NodePtr &node, Vector<ObjectPtr> *obj)
{
	if (!node) return;

	ObjectPtr o = Object::cast(node);
	if (o) obj->append(o);

	for (int i = 0; i < node->getNumChildren(); i++)
	{
		NodeReferencePtr nodeRef = NodeReference::cast(node->getChild(i));
		if (nodeRef)
			find_obj_in_children(nodeRef->getReference(), obj);
		else
			find_obj_in_children(node->getChild(i), obj);
	}
}

void VRPlayerVR::controller_update(
	int num,
	const Mat4 &player_transform,
	int is_device_connected,
	const mat4 &device_pose)
{
	if (is_device_connected)
	{
		controller[num]->setWorldTransform(player_transform * Mat4(device_pose) * Mat4(rotateX(-90.0f)));

		if (controller[num]->isEnabled())
			controller_ref[num]->setEnabled(1);
	}
	else if (controller[num]->isEnabled())
		controller_ref[num]->setEnabled(0);
}

void VRPlayerVR::teleport_init()
{
	// marker init
	teleport_marker->setEnabled(0);

	// ray init
	teleport_ray = ObjectMeshDynamic::create();


	teleport_ray->setWorldParent(player->getNode());

	for (int i = 0; i < teleport_ray->getNumSurfaces(); i++)
	{
		teleport_ray->setMaterial(teleport_marker_mat.get(), i);
		teleport_ray->setSurfaceProperty("surface_base", i);
		teleport_ray->setCastShadow(0, i);
		teleport_ray->setCastWorldShadow(0, i);
		teleport_ray->setIntersection(0, i);
		teleport_ray->setCollision(0, i);
	}
	teleport_ray->setPosition(Vec3::ZERO);
	teleport_ray->setRotation(quat::IDENTITY);

	// clear array
	for (int i = 0; i < CONTROLLER_COUNT; i++)
		teleport_button_pressed[i] = 0;

	for (int i = 0; i < CONTROLLER_COUNT; i++)
		info_button_pressed[i] = 0;
}

void VRPlayerVR::teleport_update(int num, int button_pressed, const Vec3 &offset)
{
	// check if nobody using our teleport
	if (button_pressed &&
		!teleport_button_pressed[num] &&
		teleport_button_pressed[1 - num]) // if controllers > 2 it doesn't work!
		return;

	if (!button_pressed && !teleport_button_pressed[num])
		return;

	if (object_gui->isEnabled())
		return;

	int last_button_state = teleport_button_pressed[num];
	teleport_button_pressed[num] = button_pressed;

	Vec3 pos1 = controller[num]->getWorldPosition();
	Vec3 pos2 = controller[num]->getWorldPosition() + Vec3(controller[num]->getWorldDirection(Math::AXIS_Y)) * player->getZFar();

	if (button_pressed == 1 || (button_pressed == 0 && last_button_state == 1))
	{
		ObjectPtr hitObj = World::get()->getIntersection(pos1, pos2, teleportation_mask, intersection);
		if (hitObj)
		{
			bool inside_bound = true;

			Unigine::Math::dvec3 hitPt = intersection->getPoint();

			if (hotpoints.size() > 0 && hotpoints[cur_hotpoint]->teleport_bound) {
				//inside_bound = teleport_bound_box.inside(intersection->getPoint(), UNIGINE_EPSILON);
				Unigine::Math::dvec3 auxPt1 = hitPt + Vec3(0, 0, 1) * 1000000;
				Unigine::Math::dvec3 auxPt2 = hitPt + Vec3(0, 0, -1) * 1000000;

				hotpoints[cur_hotpoint]->teleport_bound->setIntersectionMask(128, 0);
				hotpoints[cur_hotpoint]->teleport_bound->setIntersection(1, 0);
				Unigine::WorldIntersectionPtr intersectionAux = Unigine::WorldIntersection::create();
				ObjectPtr o1 = World::get()->getIntersection(hitPt, auxPt1, 128, intersectionAux);
				ObjectPtr o2 = World::get()->getIntersection(hitPt, auxPt2, 128, intersectionAux);
				inside_bound = o1 || o2;

				hotpoints[cur_hotpoint]->ShowBound(!inside_bound);
			}


			// show marker
			if (button_pressed == 1)
			{
				pos2 = hitPt + Vec3(0, 0, 0.1f);
				teleport_marker->setWorldPosition(pos2);
				teleport_marker->setEnabled(1);

				/*const auto teleport_color = inside_bound ?
					TELEPORT_COLOR_ENABLED : TELEPORT_COLOR_DISABLED;*/
					//TODO: Visualize restriction!
			}

			// teleport!
			if (button_pressed == 0 && last_button_state == 1)
			{
				if (inside_bound) {
					player->setWorldPosition(hitPt + offset);

					//��������� ������ � �������
					player->setWorldParent(hitObj->getNode());
					if (object_gui)
						object_gui->setWorldParent(hitObj->getNode());

					unhighlight();
				}

				teleport_marker->setEnabled(0);
				if (hotpoints.size() > 0)
					hotpoints[cur_hotpoint]->ShowBound(false);
			}
		}
		else
		{
			teleport_marker->setEnabled(0);
			if (hotpoints.size() > 0)
				hotpoints[cur_hotpoint]->ShowBound(false);
		}
	}

	// show ray
	if (button_pressed)
	{
		teleport_ray->clearVertex();
		teleport_ray->clearIndices();

		int num = 30;	// num of quads
		float inum = 1.0f / num;


		dmat4 transf = teleport_ray->getIWorldTransform();
		Vec3 last_p = transf * pos1;

		for (int i = 1; i <= num; i++)
		{
			Vec3 p = getHermiteSpline(transf * pos1, transf * pos1, transf * pos2, transf * (pos2 + Vec3(0, 0, -3)), inum * i);
			addLineSegment(teleport_ray, vec3(last_p), vec3(p), 0.025f);
			last_p = p;
		}

		teleport_ray->updateBounds();
		teleport_ray->updateTangents();
		teleport_ray->flushVertex();
		teleport_ray->flushIndices();

		teleport_ray->setEnabled(1);
	}
	else
	{
		teleport_ray->setEnabled(0);
	}
}

void VRPlayerVR::grab_init()
{
	hand_state.clear();
	node_grabbed.clear();
	node_selected.clear();
	last_selected.clear();
	last_selected_timer.clear();
	grab_node.clear();
	grab_components.clear();
	throw_trigger_value.clear();
	handy_pos_state.clear();

	NodePtr node_null;
	ObjectPtr obj_null;
	for (int i = 0; i < CONTROLLER_COUNT; i++)
	{
		grab_node.append(node_null);
		grab_components.append();
		hand_state.append(0);
		node_grabbed.append(0);
		last_selected.append(obj_null);
		last_selected_timer.append(0);
		node_selected.append(0);
		throw_trigger_value.append(0);
		handy_pos_state.append(0);
	}

	//pointer_init();
}

void VRPlayerVR::grab_update(int num, int pose_valid, float trigger_value, int button_touch, float pressed_pos, float release_pos)
{
	// calculate button_pressed (user experience improvement)
	int button_pressed = 0;
	if (hand_state[num] == HAND_FREE)
	{
		throw_trigger_value[num] = min(throw_trigger_value[num], trigger_value);
		button_pressed = trigger_value > pressed_pos + throw_trigger_value[num]; // around +30% from minimum pressure
	}
	else
	{
		throw_trigger_value[num] = max(throw_trigger_value[num], trigger_value);
		button_pressed = trigger_value > release_pos * throw_trigger_value[num]; // around 80% from maximum pressure
	}

	// you can't grab anything if controller doesn't have valid pose
	if (!pose_valid)
	{
		set_outline(num, 0);
		return;
	}

	// hide outline
	if (last_selected_timer[num] >= 0)
	{
		last_selected_timer[num] -= Game::get()->getIFps() / Game::get()->getScale();
		if (last_selected_timer[num] < 0)
			set_outline(num, 0);
	}

	//pointer_update(num, node_grabbed[num] == 0);

	if (node_grabbed[num] == 0)
	{
		hand_state[num] = HAND_FREE;

		ObjectPtr hitObj = last_selected[num];

		// get intersections, find new hitObj
		if (last_selected_timer[num] < 0)
		{
			if (grab_mode == GRAB_MODE::BOUNDBOX)
			{
				hitObj = ObjectPtr(); // null pointer

				intersections.clear();
				if (World::get()->getIntersection(controller[num]->getWorldBoundBox(), intersections))
				{
					// find nearest interactive object (can i grab it?)
					int nearest = -1;
					Scalar min_dist = UNIGINE_INFINITY;
					for (int k = 0; k < intersections.size(); k++)
						if (can_i_grab_it(intersections[k]->getNode()))
						{
							Scalar dist = length2(intersections[k]->getWorldPosition() - controller[num]->getWorldPosition());
							if (min_dist > dist)
							{
								min_dist = dist;
								nearest = k;
							}
						}

					// select nearest object
					if (nearest >= 0)
						hitObj = intersections[nearest];
				}
			}
			else if (grab_mode == GRAB_MODE::INTERSECTIONS)
			{
				Vec3 pos1 = controller[num]->getWorldPosition() - Vec3(controller[num]->getWorldDirection(Math::AXIS_NZ)) * ray_back_length;
				Vec3 pos2 = controller[num]->getWorldPosition() + Vec3(controller[num]->getWorldDirection(Math::AXIS_NZ)) * ray_forward_length;

				hitObj = World::get()->getIntersection(pos1, pos2, 1, intersection);
				if (!hitObj) // second intersection (for best player's experience)
					hitObj = World::get()->getIntersection(pos1, pos2 + Vec3(controller[num]->getWorldDirection(Math::AXIS_X)) * ptr_width * 0.5f, 1, intersection);
				if (!hitObj) // third intersection (for best player's experience)
					hitObj = World::get()->getIntersection(pos1, pos2 + Vec3(controller[num]->getWorldDirection(Math::AXIS_NX)) * ptr_width * 0.5f, 1, intersection);
			}
		}

		if (hitObj)
		{
			// check if this object grabbed by another controller
			for (int z = 0; z < grab_node.size(); z++)
				if (z != num && node_grabbed[z] && hitObj->getNode() == grab_node[z])
					return;

			// all right. It's movable or switcher?
			if (can_i_grab_it(hitObj->getNode()))
			{
				vibrateController(num, 0.5f);

				// show controller's outline
				last_selected[num] = hitObj;
				if (last_selected_timer[num] < 0)
					last_selected_timer[num] = 0.1f;
				set_outline(num, 1);

				// ... and grab it (or touch)
				if (button_pressed || (button_touch && is_it_switcher(hitObj->getNode())))
				{
					last_selected_timer[num] = 0;
					grab_node[num] = hitObj->getNode();
					getComponents<VRInteractable>(grab_node[num], grab_components[num]);
					node_grabbed[num] = 1;
					hand_state[num] = HAND_GRAB;

					// don't need to holding the objects if they are change our controller
					ObjMovable* obj_movable = getComponent<ObjMovable>(grab_node[num]);
					if (obj_movable && obj_movable->use_handy_pos)
						handy_pos_state[num] = GRAB_HANDY::GRAB;
					else
						handy_pos_state[num] = GRAB_HANDY::NON_HANDY;
				}
			}
		}
	}
	else
	{
		if (button_pressed)
		{
			// hold
			hand_state[num] = HAND_HOLD;

			if (handy_pos_state[num] == GRAB_HANDY::GRAB_RELEASED)
				handy_pos_state[num] = GRAB_HANDY::GRAB_AGAIN;
		}
		else
		{
			if (handy_pos_state[num] == GRAB_HANDY::NON_HANDY || handy_pos_state[num] == GRAB_HANDY::GRAB_AGAIN)
			{
				// throw!
				node_grabbed[num] = 0;
				hand_state[num] = HAND_THROW;
				handy_pos_state[num] = GRAB_HANDY::NON_HANDY;
			}
			else
				handy_pos_state[num] = GRAB_HANDY::GRAB_RELEASED;
		}
	}
}

void VRPlayerVR::set_outline(int num, int enable)
{
	if (node_selected[num] == enable)
		return;

	node_selected[num] = enable;

	// set material state (to controller)
	for (int i = 0; i < controller_obj[num].size(); i++)
		for (int k = 0; k < controller_obj[num][i]->getNumSurfaces(); k++)
			controller_obj[num][i]->setMaterialState("auxiliary", enable, k);

	// set material state (to object)
	if (last_selected[num])
		for (int i = 0; i < last_selected[num]->getNumSurfaces(); i++)
			last_selected[num]->setMaterialState("auxiliary", enable, i);

	// set post materials
	if (enable)
	{
		player->setPostMaterials("vr_post_filter_selection");
	}
	else
	{
		// if other hand use post materials, then return
		for (int i = 0; i < node_selected.size(); i++)
			if (node_selected[i] == 1)
				return;

		player->setPostMaterials("");
	}
}

void VRPlayerVR::pointer_init()
{
	ptr_mesh.clear();
	ptr_anim.clear();
	for (int i = 0; i < CONTROLLER_COUNT; i++)
	{
		ObjectMeshDynamicPtr m = ObjectMeshDynamic::create();
		for (int k = 0; k < m->getNumSurfaces(); k++)
		{
			m->setMaterial("mesh_base", k);
			m->setSurfaceProperty("surface_base", k);
			m->setCastShadow(0, k);
			m->setCastWorldShadow(0, k);
			m->setIntersection(0, k);
			m->setCollision(0, k);
		}
		m->setPosition(Vec3::ZERO);
		m->setRotation(quat::IDENTITY);

		m->clearVertex();
		m->clearIndices();
		m->addTriangleQuads(1);
		m->addVertex(vec3::ZERO);	m->addTexCoord(vec4(0, 0, 0, 0));
		m->addVertex(vec3::ZERO);	m->addTexCoord(vec4(1, 0, 0, 0));
		m->addVertex(vec3::ZERO);	m->addTexCoord(vec4(1, 1, 0, 0));
		m->addVertex(vec3::ZERO);	m->addTexCoord(vec4(0, 1, 0, 0));
		m->updateBounds();
		m->updateTangents();
		m->flushVertex();
		m->flushIndices();

		ptr_mesh.append(m);
		ptr_anim.append(0);
	}
}

void VRPlayerVR::pointer_update(int num, int find_mode)
{
	// find objects around hand
	float n = -1;
	if (find_mode)
	{
		UNIGINE_BOUND_SPHERE s;
		Vector<ObjectPtr> objs;

		s.set(controller[num]->getWorldPosition(), ray_forward_length);
		if (World::get()->getIntersection(s, objs))
			for (int i = 0; i < objs.size(); i++)
				if (can_i_grab_it(objs[i]->getNode()))
				{
					n = 1;
					break;
				}
	}

	// animation
	ptr_anim[num] = saturate(ptr_anim[num] + n * 7.5f * Game::get()->getIFps() / Game::get()->getScale());

	// update mesh
	vec3 view_dir = head->getWorldDirection();
	vec3 pb = vec3(controller[num]->getWorldPosition());
	vec3 pe = pb + controller[num]->getWorldDirection(Math::AXIS_NZ) * ray_forward_length * ptr_anim[num];
	vec3 dir = normalize(pe - pb);
	vec3 right = cross(view_dir, dir);

	vec3 p0 = pb - right * ptr_width * 0.5f;	// 0, 0
	vec3 p1 = pb + right * ptr_width * 0.5f;	// 1, 0
	vec3 p2 = pe + right * ptr_width * 0.5f;	// 1, 1
	vec3 p3 = pe - right * ptr_width * 0.5f;	// 0, 1

	ptr_mesh[num]->setVertex(0, p0);
	ptr_mesh[num]->setVertex(1, p1);
	ptr_mesh[num]->setVertex(2, p2);
	ptr_mesh[num]->setVertex(3, p3);

	ptr_mesh[num]->updateBounds();
	//ptr_mesh[num]->updateTangents();
	ptr_mesh[num]->flushVertex();
	//ptr_mesh[num]->flushIndices();
}

void VRPlayerVR::push_hand_linear_velocity(int num, const vec3 &velocity)
{
	Vector<vec3> *buffer = &hand_linear_velocity[num];

	if (buffer->size() < controller_buffer_count)
		buffer->append(velocity);
	else
	{
		for (int i = 0; i < buffer->size() - 1; i++)
			buffer->get(i) = buffer->get(i + 1);

		buffer->get(buffer->size() - 1) = velocity;
	}
}

void VRPlayerVR::push_hand_angular_velocity(int num, const vec3 &velocity)
{
	Vector<vec3> *buffer = &hand_angular_velocity[num];

	if (buffer->size() < controller_buffer_count)
		buffer->append(velocity);
	else
	{
		for (int i = 0; i < buffer->size() - 1; i++)
			buffer->get(i) = buffer->get(i + 1);

		buffer->get(buffer->size() - 1) = velocity;
	}
}




//hot points
void VRPlayerVR::hotpoints_init(const char * hotpoints_name)
{
	int pageNum = 0;
	pages.append(pageNum);
	pageCount = 1;


	Unigine::NodePtr hotpointsNode = Editor::get()->getNodeByName(hotpoints_name);
	if (hotpointsNode)
	{
		int nc = hotpointsNode->getNumChildren();

		for (int c = 0; c < nc; c++) {

			if (pages[pageNum].size() >= btnsOnPageMaxCount) {
				//next page
				pageNum++;
				pages.append(pageNum);
				pageCount++;
			}


			Unigine::NodePtr hpNode = hotpointsNode->getChild(c);
			HotPoint* hp = new HotPoint(hpNode, c, pageNum);

			pages[pageNum].append(hp);

			hotpoints.append(hp);
		}
	}

	//��������� ����������� � ��������
	Unigine::Vector<Unigine::NodePtr> hotpoints_binded;
	Editor::get()->getNodesByName("vr_hotpoint_binded", hotpoints_binded);

	for (Unigine::NodePtr node : hotpoints_binded) {
		if (node->getType() != Unigine::Node::PLAYER_DUMMY) continue;

		if (pages[pageNum].size() >= btnsOnPageMaxCount) {
			//next page
			pageNum++;
			pages.append(pageNum);
			pageCount++;
		}

		HotPoint* hp = new HotPoint(node, hotpoints.size(), pageNum);//TODO: ��������� ������

		pages[pageNum].append(hp);
		hotpoints.append(hp);
	}

	if (hotpoints.size() > 0) {
		//set current hotpoint 0
		setCurrHotpoint(0);
	}

}

void VRPlayerVR::setCurrHotpoint(int index)
{
	if (hotpoints.size() > 0)
		hotpoints[cur_hotpoint]->ShowBound(false);

	cur_hotpoint = index;

	HotPoint* hotpoint = hotpoints[cur_hotpoint];
	Unigine::NodePtr hpNode = hotpoint->hotpoint;

	Mat4 guiLocalTransf = Mat4(translate(0.0f, player_height, -2.0f));
	if (object_gui) {
		Mat4 baseGuiCoords = Unigine::Math::inverse(gui_near_eyes_pos());
		Mat4 guiWorldTransf = object_gui->getWorldTransform();

		guiLocalTransf = baseGuiCoords * guiWorldTransf;
	}


	landPlayerTo(hpNode->getWorldPosition(), hpNode->getWorldDirection());
	//hotpoint_button_pressed = 0;

	//teleport_bound = hotpoint->teleport_bound;


	if (object_gui)
		object_gui->setWorldTransform(gui_near_eyes_pos() * guiLocalTransf);

	//toggles
	for (int h = 0; h < hotpoints.size(); h++) {
		if (hotpoints[h]->toggleContainer && h != cur_hotpoint)
			hotpoints[h]->toggleContainer->setColor(vec4(1, 1, 1, 1));
	}
	if (hotpoint->toggleContainer)
		hotpoint->toggleContainer->setColor(vec4(0.5, 0.8, 0.8, 1));

	//image
	set_image(hotpoint->image);

	//page
	setPage(hotpoint->_pageNum);
}






void VRPlayerVR::hotpoints_update(int button_prev, int button_next)
{
	if (hotpoints.size() == 0)
		return;

	if (button_prev || button_next)
	{
		cur_hotpoint += button_next - button_prev;
		if (cur_hotpoint < 0)
			cur_hotpoint = hotpoints.size() - 1;
		else if (cur_hotpoint >= hotpoints.size())
			cur_hotpoint = 0;

		setCurrHotpoint(cur_hotpoint);

		unhighlight();
	}
}
