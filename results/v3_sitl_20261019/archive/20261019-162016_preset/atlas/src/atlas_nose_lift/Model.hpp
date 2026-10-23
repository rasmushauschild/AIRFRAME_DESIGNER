#pragma once
#include <parameters/param.h>
#include <cmath>
struct AtlasGroundModel { float mass{}, gx{}, gz{}, moment{}, w9{}; };
inline bool atlas_model_matches(float hover, AtlasGroundModel &m) {
 int32_t enabled{}, count{};
 param_get(param_find("NLF_CFG_OK"), &enabled);
 param_get(param_find("CA_ROTOR_COUNT"), &count);
 if (enabled!=1 || count!=10 || !std::isfinite(hover)) {
  PX4_ERR("unsupported ground geometry; Update PX4 with a ten-motor tripod model"); return false;
 }
 const char *names[]={"NLF_MASS","NLF_GX","NLF_GZ","NLF_MOM","NLF_W9"};
 float *values[]={&m.mass,&m.gx,&m.gz,&m.moment,&m.w9};
 for (int i=0;i<5;i++) {
  param_t p=param_find(names[i]);
  if(p==PARAM_INVALID || param_get(p,values[i])!=0 || !std::isfinite(*values[i])) { return false; }
 }
 float factor{}; param_t p=param_find("THR_MDL_FAC");
 if(p==PARAM_INVALID || param_get(p,&factor)!=0 || !std::isfinite(factor) || fabsf(factor)>.0001f) { PX4_ERR("THR_MDL_FAC must be 0 (Update PX4 sets it)"); return false; }
 if (!(m.mass>0.f && m.gx>0.f && m.moment>0.f && m.w9>0.f && m.w9<2.f)) { PX4_ERR("NLF_MASS/GX/MOM/W9 out of range; Update PX4"); return false; }
 return true;
}
