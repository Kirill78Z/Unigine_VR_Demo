#include "VRPlayerVive.h"
#include <UnigineVisualizer.h>
#include <UnigineGame.h>
#include "../../Utils.h"

#define HIDE_BASESTATIONS

REGISTER_COMPONENT(VRPlayerVive);

using namespace Unigine;
using namespace Math;

void VRPlayerVive::init()
{
	VRPlayer::init();
	vive.init();
	find_devices();
	init_player();
	load_player_height(0);
	landPlayerTo(player->getWorldPosition(), player->getWorldDirection());
	controllers_init();
	basestation[0] = NodeReference::cast(basestation_0);
	basestation[1] = NodeReference::cast(basestation_1);
#ifdef HIDE_BASESTATIONS
	basestation[0]->setEnabled(0);
	basestation[1]->setEnabled(0);
#endif
	teleport_init();
	grab_init();

	gui_init();
}

void VRPlayerVive::update()
{
	find_devices();

	controller_valid[0] = CONTROLLER_DEVICE_0 != -1 && vive.isDeviceConnected(CONTROLLER_DEVICE_0) && vive.isPoseValid(CONTROLLER_DEVICE_0);
	controller_valid[1] = CONTROLLER_DEVICE_1 != -1 && vive.isDeviceConnected(CONTROLLER_DEVICE_1) && vive.isPoseValid(CONTROLLER_DEVICE_1);

	// if system menu is open
	if ((controller_valid[0] && vive.getControllerButtonPressed(CONTROLLER_DEVICE_0, BUTTON_SYSTEM) < 0) ||
		(controller_valid[1] && vive.getControllerButtonPressed(CONTROLLER_DEVICE_1, BUTTON_SYSTEM) < 0))
	{
		for (int i = 0; i < 2; i++)
		{
			controller_ref[i]->setEnabled(0);
			teleport_button_pressed[i] = 0;
		}
		teleport_marker->setEnabled(0);
		teleport_ray->setEnabled(0);

		return;
	}

	Mat4 player_transform = player->getWorldTransform();
	Mat4 hmd_transform = Mat4(vive.getDevicePose(HMD_DEVICE_0));
	Mat4 hmd_transform_world = player_transform * hmd_transform;
	Vec3 head_offset =
		(vive.isDeviceConnected(HMD_DEVICE_0) && HMD_DEVICE_0 != -1) ?
		player_transform.getTranslate() - hmd_transform_world.getTranslate() :
		Vec3::ZERO;
	head_offset.z = 0;

	head->setWorldTransform(hmd_transform_world);

#ifndef HIDE_BASESTATIONS
	basestation_update(0, player_transform, BASESTATION_DEVICE_0);
	basestation_update(1, player_transform, BASESTATION_DEVICE_1);
#endif
	controller_update(0, player_transform, CONTROLLER_DEVICE_0);
	controller_update(1, player_transform, CONTROLLER_DEVICE_1);

	teleport_update(0, controller_valid[0] ?
		//vive.getControllerButtonPressed(CONTROLLER_DEVICE_0, /*BUTTON_DPAD_UP*/BUTTON_STEAMVR_TOUCHPAD) 
		vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_UP)
		: 0, head_offset);
	teleport_update(1, controller_valid[1] ?
		//vive.getControllerButtonPressed(CONTROLLER_DEVICE_1, /*BUTTON_DPAD_UP*/BUTTON_STEAMVR_TOUCHPAD) 
		vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_UP)
		: 0, head_offset);

	unhighlightAll();

	update_information(0, getControllerButton(0, BUTTON::TRIGGER));
	update_information(1, getControllerButton(1, BUTTON::TRIGGER));

	bool show_info = highlightedObjs.size() > 0;
	if (show_info) {
		assert(highlightedObjs.size() == 1);

		player->setPostMaterials("vr_post_filter_selection");

		Unigine::ObjectPtr o = highlightedObjs[0];
		int n = o->findProperty("data");
		if (n != -1) {
			Unigine::PropertyPtr prop = o->getProperty();

			n = prop->findParameter("Text");
			if (n != -1) {
				const char* text = prop->getParameterString(n);
				info_text->setText(text);

				infoImage->setHeight(0);

				n = prop->findParameter("Image");
				if (n != -1) {
					Unigine::String fileName = prop->getParameterFile(n);
					if (Unigine::FileSystem::get()->isFileExist(fileName)) {
						unsigned int key = Unigine::String::hash(fileName, fileName.size());
						Unigine::ImagePtr image = Unigine::ImagePtr::Ptr();
						if (cashedImgs.find(key) != cashedImgs.end()) {
							image = cashedImgs[key];
						}
						else
						{
							image = Unigine::Image::create(fileName);
							cashedImgs[key] = image;
						}

						if (image) {
							if (currentImg != image) {
								infoImage->setImage(image);
								currentImg = image;
							}


							infoImage->setHeight(info_gui_height_pixel / 2);
						}
					}

				}


				//info gui
				info_object_gui->setEnabled(1);

				// put GUI around player
				if (!prev_show_info) {
					info_object_gui->setWorldTransform(info_gui_pos());
				}

			}

			else
				show_info = false;
		}
		else
			show_info = false;


	}

	if (!show_info)
	{
		player->setPostMaterials("");
		info_object_gui->setEnabled(0);
	}

	prev_show_info = show_info;

	move_update(hmd_transform_world);

	collisions_update(hmd_transform_world, head_offset);

	grab_update(0, controller_valid[0], getControllerAxis(0, BUTTON::GRIP), getControllerButtonDown(0, BUTTON::TRIGGER));
	grab_update(1, controller_valid[1], getControllerAxis(1, BUTTON::GRIP), getControllerButtonDown(1, BUTTON::TRIGGER));

	quat rot = player->getRotation();
	if (CONTROLLER_DEVICE_0 != -1)
	{
		push_hand_linear_velocity(0, rot * vive.getDeviceVelocity(CONTROLLER_DEVICE_0) * hand_force_multiplier);
		push_hand_angular_velocity(0, rot * vive.getDeviceAngularVelocity(CONTROLLER_DEVICE_0));
	}
	if (CONTROLLER_DEVICE_1 != -1)
	{
		push_hand_linear_velocity(1, rot * vive.getDeviceVelocity(CONTROLLER_DEVICE_1) * hand_force_multiplier);
		push_hand_angular_velocity(1, rot * vive.getDeviceAngularVelocity(CONTROLLER_DEVICE_1));
	}



	int dpadLeftReleased0 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_LEFT) && dpadLeftPressed0;
	int dpadRightReleased0 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_RIGHT) && dpadRightPressed0;
	int dpadUpReleased0 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_UP) && dpadUpPressed0;
	int dpadDownReleased0 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_DOWN) && dpadDownPressed0;

	int dpadLeftReleased1 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_LEFT) && dpadLeftPressed1;
	int dpadRightReleased1 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_RIGHT) && dpadRightPressed1;
	int dpadUpReleased1 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_UP) && dpadUpPressed1;
	int dpadDownReleased1 = !vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_DOWN) && dpadDownPressed1;

	turn_page_update(
		(controller_valid[0] && dpadLeftReleased0) ||
		(controller_valid[1] && dpadLeftReleased1),
		(controller_valid[0] && dpadRightReleased0) ||
		(controller_valid[1] && dpadRightReleased1)
	);
	update_gui();

	if (!object_gui->isEnabled())
		hotpoints_update(
		(controller_valid[0] && dpadLeftReleased0) ||
			(controller_valid[1] && dpadLeftReleased1),
			(controller_valid[0] && dpadRightReleased0) ||
			(controller_valid[1] && dpadRightReleased1));
	else
		hotpoints_update(
		(controller_valid[0] && dpadUpReleased0) ||
			(controller_valid[1] && dpadUpReleased1)
			,
			(controller_valid[0] && dpadDownReleased0) ||
			(controller_valid[1] && dpadDownReleased1)
		);


	dpadLeftPressed0 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_LEFT);
	dpadRightPressed0 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_RIGHT);
	dpadUpPressed0 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_UP);
	dpadDownPressed0 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_0, BUTTON_DPAD_DOWN);

	dpadLeftPressed1 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_LEFT);
	dpadRightPressed1 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_RIGHT);
	dpadUpPressed1 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_UP);
	dpadDownPressed1 = vive_getControllerDPadPressed(CONTROLLER_DEVICE_1, BUTTON_DPAD_DOWN);


	update_teleport_ray_visibility();
}

void VRPlayerVive::setLock(int lock)
{
	vive.setHeadPositionLock(lock);
	vive.setHeadRotationLock(lock);
}

void VRPlayerVive::setPlayerPosition(const Vec3 &pos)
{
	put_head_to_position(vive.getDevicePose(HMD_DEVICE_0), pos);
}

void VRPlayerVive::landPlayerTo(const Vec3 &position, const vec3 &direction)
{
	land_player_to(vive.getDevicePose(HMD_DEVICE_0), position, direction);
}

void VRPlayerVive::basestation_update(int num, const Mat4 &player_transform, int device_id)
{
	if (device_id != -1 && vive.isDeviceConnected(device_id))
	{
		mat4 transform = vive.getDevicePose(device_id);
		basestation[num]->setWorldTransform(player_transform * Mat4(transform));
		basestation[num]->setEnabled(1);
	}
	else
		basestation[num]->setEnabled(0);
}

void VRPlayerVive::controller_update(int num, const Mat4 &player_transform, int device_id)
{
	if (device_id != -1 && vive.isDeviceConnected(device_id))
	{
		mat4 transform = vive.getDevicePose(device_id);
		controller[num]->setWorldTransform(player_transform * Mat4(transform) * Mat4(rotateX(-90.0f)));

		// update buttons
		float trigger_position = vive.getControllerAxis(device_id, 1).x;
		set_controller_trigger_position(controller[num], trigger_position);

		if (controller[num]->isEnabled())
			controller_ref[num]->setEnabled(1);
	}
	else if (controller[num]->isEnabled())
		controller_ref[num]->setEnabled(0);
}

void VRPlayerVive::set_controller_trigger_position(NodeReferencePtr &controller, float position)
{
	NodePtr root = controller->getReference();
	int pivot_id = root->findChild("vive_trigger");
	if (pivot_id != -1)
	{
		NodePtr pivot_node = root->getChild(pivot_id);
		pivot_node->setTransform(Mat4(translate(0.0f, -0.039f, -0.018f) * rotateX(-position * 20.0f)));
	}
}

void VRPlayerVive::find_devices()
{
	int curTracking = 0;
	int curController = 0;
	for (int i = 0; i < MAX_TRACKED_DEVICE_COUNT; i++)
	{
		int deviceType = vive.getDeviceType(i);
		switch (deviceType)
		{
		case TRACKED_DEVICE_TRACKING:
			if (curTracking == 0)
				BASESTATION_DEVICE_0 = i;
			else
				BASESTATION_DEVICE_1 = i;
			curTracking++;
			break;
		case TRACKED_DEVICE_CONTROLLER:
			if (curController == 0)
				CONTROLLER_DEVICE_0 = i;
			else
				CONTROLLER_DEVICE_1 = i;
			curController++;
			break;
		case TRACKED_DEVICE_HMD:
			HMD_DEVICE_0 = i;
			break;
		default:
			break;
		}
	}
}

void VRPlayerVive::vibrateController(int controller_num, float amplitude)
{
	if (xpad360->isAvailable())
	{
		if (controller_num == 0) xpad360->setLeftMotor(amplitude * 0.2f);
		if (controller_num == 1) xpad360->setRightMotor(amplitude * 0.2f);
	}

	if (amplitude > 0)
	{
		if (controller_num == 0) vive.setControllerVibration(CONTROLLER_DEVICE_0, 100);
		if (controller_num == 1) vive.setControllerVibration(CONTROLLER_DEVICE_1, 100);
	}
}

Mat4 VRPlayerVive::getControllerTransform(int controller_num)
{
	return Mat4(controller_num == 0 ? vive.getDevicePose(CONTROLLER_DEVICE_0) : vive.getDevicePose(CONTROLLER_DEVICE_1));
}

Mat4 VRPlayerVive::gui_near_eyes_pos() {
	Mat4 flatHeadTransform = Mat4::IDENTITY;

	Mat4 headTransform = Mat4(vive.getDevicePose(HMD_DEVICE_0));
	flatHeadTransform.setRotateY(headTransform.getRotate().getAngle(vec3::FORWARD));

	dvec3 transl = headTransform.getTranslate();
	flatHeadTransform.setColumn3(3, transl);


	return player->getWorldTransform()
		* flatHeadTransform;
}

Mat4 VRPlayerVive::info_gui_pos() {
	Mat4 flatHeadTransform = Mat4::IDENTITY;

	Mat4 headTransform = Mat4(vive.getDevicePose(HMD_DEVICE_0));
	flatHeadTransform.setRotateY(headTransform.getRotate().getAngle(vec3::FORWARD) + 20);

	dvec3 transl = headTransform.getTranslate();
	flatHeadTransform.setColumn3(3, transl);

	
	return player->getWorldTransform()
		* flatHeadTransform 
		* Mat4(rotate(vec3(1, 0, 0), -15))
		* Mat4(translate(0.0f, player_height, -1.0f));
}


int VRPlayerVive::getControllerButton(int controller_num, int button)
{
	if (!controller_valid[controller_num])
		return 0;

	int device = (controller_num == 0 ? CONTROLLER_DEVICE_0 : CONTROLLER_DEVICE_1);

	switch (button)
	{
	case STICK_X: return Math::abs(vive.getControllerAxis(device, 0).x) > 0.5f;
	case STICK_Y: return Math::abs(vive.getControllerAxis(device, 0).y) > 0.5f;
	case TRIGGER: return vive.getControllerAxis(device, 1).x > 0.5f;
	case GRIP: return vive.getControllerButtonPressed(device, BUTTON_GRIP);
	case XA: return vive_getControllerDPadPressed(device, BUTTON_DPAD_DOWN);
	case YB: return vive_getControllerDPadPressed(device, BUTTON_DPAD_UP);
	case MENU: return vive.getControllerButtonPressed(device, BUTTON_APPLICATIONMENU);
	}

	return 0;
}

float VRPlayerVive::getControllerAxis(int controller_num, int button)
{
	if (!controller_valid[controller_num])
		return 0;

	int device = (controller_num == 0 ? CONTROLLER_DEVICE_0 : CONTROLLER_DEVICE_1);

	switch (button)
	{
	case STICK_X: return vive.getControllerAxis(device, 0).x;
	case STICK_Y: return vive.getControllerAxis(device, 0).y;
	case TRIGGER: return vive.getControllerAxis(device, 1).x;
	case GRIP: return itof(vive.getControllerButtonPressed(device, BUTTON_GRIP));
	case XA: return itof(vive_getControllerDPadPressed(device, BUTTON_DPAD_DOWN));
	case YB: return itof(vive_getControllerDPadPressed(device, BUTTON_DPAD_UP));
	case MENU: return itof(vive.getControllerButtonPressed(device, BUTTON_APPLICATIONMENU));
	}

	return 0;
}

int VRPlayerVive::vive_getControllerDPadPressed(int device, int button)
{
	// You might expect that pressing one of the edges of the SteamVR controller touchpad could
	// be detected with a call to k_EButton_DPad_* (BUTTON_DPAD_*), but currently this always returns false.
	// Not sure whether this is SteamVR's design intent, not yet implemented, or a bug.
	// The expected behaviour can be achieved by detecting overall Touchpad press, with Touch-Axis comparison to an edge threshold.

	if (vive.getControllerButtonPressed(device, BUTTON_STEAMVR_TOUCHPAD))
	{
		vec2 axis = vive.getControllerAxis(device, 0);
		if ((axis.y > 0.6f && button == BUTTON_DPAD_UP) ||
			(axis.y < -0.6f && button == BUTTON_DPAD_DOWN) ||
			(axis.x > 0.6f && button == BUTTON_DPAD_RIGHT) ||
			(axis.x < -0.6f && button == BUTTON_DPAD_LEFT))
			return 1;
	}

	return 0;
}