// setup constants
.alias myconst c32 as (1.0, 0.0, 0.5, 1.0)

// setup output map
.alias outpos  o0      as position
.alias outcol  o1      as color
.alias outtex0 o2.xyzw as texcoord0 // Would like to use .xy instead, but this is not supported by ctrulib currently
.alias outtex1 o3.xyzw as texcoord1
.alias outtex2 o4.xyzw as texcoord2

// setup uniform map, for use with SHDR_GetUniformRegister
.alias projection     c0-c3
.alias modelview      c4-c7

.alias num_lights     i1

.alias light_dir      c8
.alias light_diffuse  c9
.alias light_ambient  c10
.alias light_dir2     c11
.alias light_diffuse2 c12
.alias light_ambient2 c13

main:
	mov r1.xyz,  v0.xyz
	mov r1.w,    myconst.w

mdvl:  // tempreg = mdlvMtx * in.pos
	dp4 r0.x,  modelview[0],  r1
	dp4 r0.y,  modelview[1],  r1
	dp4 r0.z,  modelview[2],  r1
	mov r0.w,  myconst.w

proj:  // result.pos = projMtx * tempreg
	dp4 outpos.x,  projection[0],  r0
	dp4 outpos.y,  projection[1],  r0
	dp4 outpos.z,  projection[2],  r0
	dp4 outpos.w,  projection[3],  r0

tex:  // result.texcoord = in.texcoord
	mov outtex0,  v1.xyzw
	mov outtex1,  myconst.yyyw
	mov outtex2,  myconst.yyyw

lighting: // color = sum over all lights(diffuse * clamp(dot(L,N),0) + ambient)
	mov r0, myconst.yyyw

    loop num_lights
		mov r1.xyz, myconst.yyy
		dp3 r1.xyz, light_dir[lcnt].xyz, v2.xyz
		max r1.xyz, r1.xyz, myconst.yyy
		mul r1.xyz, r1.xyz, light_diffuse[lcnt].xyz
		add r1.xyz, r1.xyz, light_ambient[lcnt].xyz
		add r0.xyz, r1.xyz, r0.xyz
		nop
	endloop
	min r0.xyz, r0.xyz, myconst.xxx

	mov outcol, r0



	nop
	end

endmain:
