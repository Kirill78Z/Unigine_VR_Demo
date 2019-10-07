// Copyright (C) 2005-2018, Unigine Corp. All rights reserved.

#define THICKNESS 0.0025f

#include <core/shaders/common/fragment.h>

INIT_TEXTURE(0,TEX_COLOR)
INIT_TEXTURE(1,TEX_AUXILIARY)

STRUCT(FRAGMENT_IN)
	INIT_POSITION
	INIT_IN(float2,0)
	INIT_IN(float3,1)
END

MAIN_BEGIN(FRAGMENT_OUT,FRAGMENT_IN)
	
	float2 uv = IN_DATA(0).xy;
	
	float4 color = TEXTURE_BIAS_ZERO(TEX_COLOR,uv); 
	float4 auxiliary = TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv); 

	float iaspect = s_viewport.y / s_viewport.x;
	float offset_y = THICKNESS;
	float offset_x = offset_y * iaspect;
	float4 selection = TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(0, offset_y));
		  selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(0, -offset_y));
		  selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(offset_x, 0));
		  selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(-offset_x, 0));
		  //selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(offset_x, offset_y));
		  //selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(-offset_x, offset_y));
		  //selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(offset_x, -offset_y));
		 // selection += TEXTURE_BIAS_ZERO(TEX_AUXILIARY,uv + float2(-offset_x, -offset_y));
	selection = saturate(selection);
	
	OUT_COLOR = color + selection - auxiliary;
	
MAIN_END
