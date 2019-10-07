@echo off
setlocal EnableDelayedExpansion
set APP="bin\main_double_x64.exe"
if exist %APP% (
	start "" %APP%  -video_app auto -video_vsync 0 -video_refresh 0 -video_mode 1 -video_resizable 1 -video_fullscreen 0 -video_debug 0 -video_gamma 1.000000 -sound_app auto -data_path "../data/"  -engine_config "../data/unigine.cfg" -extern_plugin "FbxImporter,CadImporter,Editor2" -console_command "config_readonly 0"
) else (
	set MESSAGE=%APP% not found"
	echo !MESSAGE!
)