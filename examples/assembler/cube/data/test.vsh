// setup constants
.alias myconst c32 as (1.0, 0.0, 0.5, 1.0)

// setup output map
.alias outpos  o0      as position
.alias outcol  o1      as color
.alias outtex0 o2.xyzw as texcoord0 // Would like to use .xy instead, but this is not supported by ctrulib currently
.alias outtex1 o3.xyzw as texcoord1
.alias outtex2 o4.xyzw as texcoord2

// setup uniform map, for use with SHDR_GetUniformRegister
.alias projection     c0  // -c3
.alias modelview      c4  // -c7
.alias lightDirection c8
.alias lightAmbient   c9

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
	mov outtex0,  v1
	mov outtex1,  myconst.yyyw
	mov outtex2,  myconst.yyyw

col:  // Hacky lighting: color = ambient.xyz + clamp(dot(L,N), 1.0) * ambient.www
	dp3 r0.xyz,     lightDirection.xyz, v2.xyz
	max r0.xyz,     myconst.yyy,        r0.xyz
	mul r0.xyz,     lightAmbient.www,   r0.xyz
	add outcol.xyz, lightAmbient.xyz,   r0.xyz
	mov outcol.w,   myconst.w

	nop
	end

endmain:
