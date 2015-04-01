/**
	@file
	kinect - a max object
	
	
	Q: is it better to have a separate freenect_context per device?
	
*/

extern "C" {
	#include "ext.h"		
	#include "ext_obex.h"
	#include "ext_dictionary.h"
	#include "ext_dictobj.h"
	#include "ext_systhread.h"

	#include "jit.common.h"
	#include "jit.gl.h"
}

#include <new>
#include "stdint.h"

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480



class MaxKinectBase {
public:

	struct vec2i { int x, y; };
	struct vec2f { float x, y; };
	struct vec3f { float x, y, z; };
	struct vec3c { uint8_t x, y, z; };
	
	inline vec2f sample2f(vec2f * data, vec2f coord, int stridey) {
		// warning: no bounds checking!
		vec2f c00 = data[(int)(coord.x) + (int)(coord.y)*stridey];
		vec2f c01 = data[(int)(coord.x) + (int)(coord.y+1.f)*stridey];
		vec2f c10 = data[(int)(coord.x+1.f) + (int)(coord.y)*stridey];
		vec2f c11 = data[(int)(coord.x+1.f) + (int)(coord.y+1.f)*stridey];
		float bx = coord.x - (int)coord.x;
		float by = coord.y - (int)coord.y;
		float ax = 1.f - bx;
		float ay = 1.f - by;
		vec2f result;
		result.x = c00.x * ax * ay
				 + c01.x * ax * by
				 + c10.x * bx * ay
				 + c11.x * bx * by;
		result.y = c00.y * ax * ay
				 + c01.y * ax * by
				 + c10.y * bx * ay
				 + c11.y * bx * by;
		return result;
	}
	
	inline void sample3c(vec3c& result, vec3c * data, vec2f coord, int stridey) {
		// warning: no bounds checking!
		vec3c c00 = data[(int)(coord.x) + (int)(coord.y)*stridey];
		vec3c c01 = data[(int)(coord.x) + (int)(coord.y+1.f)*stridey];
		vec3c c10 = data[(int)(coord.x+1.f) + (int)(coord.y)*stridey];
		vec3c c11 = data[(int)(coord.x+1.f) + (int)(coord.y+1.f)*stridey];
		float bx = coord.x - (int)coord.x;
		float by = coord.y - (int)coord.y;
		float ax = 1.f - bx;
		float ay = 1.f - by;
		result.x = c00.x * ax * ay
				 + c01.x * ax * by
				 + c10.x * bx * ay
				 + c11.x * bx * by;
		result.y = c00.y * ax * ay
				 + c01.y * ax * by
				 + c10.y * bx * ay
				 + c11.y * bx * by;
		result.z = c00.z * ax * ay
				 + c01.z * ax * by
				 + c10.z * bx * ay
				 + c11.z * bx * by;
	}

	t_object	ob;			// the object itself (must be first)

	void *		outlet_cloud;
	void *		outlet_rgb;
	void *		outlet_depth;
	void *		outlet_msg;
	
	// rgb matrix for raw output:
	void *		rgb_mat;
	void *		rgb_mat_wrapper;
	t_atom		rgb_name[1];
	vec3c * 	rgb_back;
	
	// depth matrix for raw output:
	void *		depth_mat;
	void *		depth_mat_wrapper;
	t_atom		depth_name[1];
	uint32_t *	depth_back;
	
	// cloud matrix for output:
	void *		cloud_mat;
	void *		cloud_mat_wrapper;
	t_atom		cloud_name[1];
	vec3f *		cloud_back;
	
	// transformed cloud matrix for output:
	void *		trans_cloud_mat;
	void *		trans_cloud_mat_wrapper;
	t_atom		trans_cloud_name[1];
	vec3f *		trans_cloud_back;
	
	// rgb matrix for cloud output:
	void *		rgb_cloud_mat;
	void *		rgb_cloud_mat_wrapper;
	t_atom		rgb_cloud_name[1];
	vec3c *		rgb_cloud_back;
	
	// attributes:
	vec2f		depth_focal;
	vec2f		depth_center;
	vec2f		rgb_focal;
	vec2f		rgb_center;
	vec3f		rgb_translate;
	vec3f		rgb_rotate[3];
	vec3f		trans_translate;
	vec3f		trans_rotate[3];
	float		depth_base, depth_offset;
	int			unique;
	int			device_count;
	int			near_mode;
	int			player;
	int			use_rgb;
	int			align_rgb_to_cloud;
	int			transform_cloud;
	
	vec2f *		depth_map_data;
	vec2f *		rgb_map_data;
	
	volatile char new_rgb_data;
	volatile char new_depth_data;
	volatile char new_cloud_data;
	
	MaxKinectBase() {
		// set up attrs:
		unique = 1;
		near_mode = 0;
		device_count = 0;
		player = 0;
		align_rgb_to_cloud = 0;
		transform_cloud = 0;
		use_rgb = 1;
		
		new_rgb_data = 0;
		new_depth_data = 0;
		new_cloud_data = 0;
		
		// can we accept a dict?
		depth_base = 0.085f;
		depth_offset = 0.0011f;
		depth_center.x = 314.f;
		depth_center.y = 241.f;
		depth_focal.x = 597.f;
		depth_focal.y = 597.f;
		rgb_center.x = 320.f;
		rgb_center.y = 240.f;
		rgb_focal.x = 524.f;
		rgb_focal.y = 524.f;
		
		rgb_translate.x = rgb_translate.y = rgb_translate.z = 0;
		rgb_rotate[0].y = rgb_rotate[0].z 
			= rgb_rotate[1].x = rgb_rotate[1].z 
			= rgb_rotate[2].x = rgb_rotate[2].y = 0;
		rgb_rotate[0].x = rgb_rotate[1].y = rgb_rotate[2].z = 1;
		
		trans_translate.x = trans_translate.y = trans_translate.z = 0;
		trans_rotate[0].y = trans_rotate[0].z 
			= trans_rotate[1].x = trans_rotate[1].z 
			= trans_rotate[2].x = trans_rotate[2].y = 0;
		trans_rotate[0].x = trans_rotate[1].y = trans_rotate[2].z = 1;
		
		
		// create matrices:
		t_jit_matrix_info info;
		
		rgb_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		rgb_mat = jit_object_method(rgb_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("char");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(rgb_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(rgb_mat, _jit_sym_clear);
		jit_object_method(rgb_mat, _jit_sym_getdata, &rgb_back);
		// cache name:
		atom_setsym(rgb_name, jit_attr_getsym(rgb_mat_wrapper, _jit_sym_name));
		
		depth_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		depth_mat = jit_object_method(depth_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 1;
		info.type = gensym("long");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(depth_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(depth_mat, _jit_sym_clear);
		jit_object_method(depth_mat, _jit_sym_getdata, &depth_back);
		// cache name:
		atom_setsym(depth_name, jit_attr_getsym(depth_mat_wrapper, _jit_sym_name));
		
		cloud_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		cloud_mat = jit_object_method(cloud_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("float32");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(cloud_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(cloud_mat, _jit_sym_clear);
		jit_object_method(cloud_mat, _jit_sym_getdata, &cloud_back);
		// cache name:
		atom_setsym(cloud_name, jit_attr_getsym(cloud_mat_wrapper, _jit_sym_name));
		
		trans_cloud_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		
		trans_cloud_mat = jit_object_method(trans_cloud_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("float32");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(trans_cloud_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(trans_cloud_mat, _jit_sym_clear);
		jit_object_method(trans_cloud_mat, _jit_sym_getdata, &trans_cloud_back);
		// cache name:
		atom_setsym(trans_cloud_name, jit_attr_getsym(trans_cloud_mat_wrapper, _jit_sym_name));
		
		rgb_cloud_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		rgb_cloud_mat = jit_object_method(rgb_cloud_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("char");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(rgb_cloud_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(rgb_cloud_mat, _jit_sym_clear);
		jit_object_method(rgb_cloud_mat, _jit_sym_getdata, &rgb_cloud_back);
		// cache name:
		atom_setsym(rgb_cloud_name, jit_attr_getsym(rgb_cloud_mat_wrapper, _jit_sym_name));
		
		// init undistortion maps with default data:
		depth_map_data = (vec2f *)sysmem_newptr(DEPTH_WIDTH*DEPTH_HEIGHT * sizeof(vec2f));
		rgb_map_data = (vec2f *)sysmem_newptr(DEPTH_WIDTH*DEPTH_HEIGHT * sizeof(vec2f));
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				depth_map_data[i].x = x;
				depth_map_data[i].y = y;
				rgb_map_data[i].x = x;
				rgb_map_data[i].y = y;
			}
		}
	}
	
	~MaxKinectBase() {
		if (rgb_mat_wrapper) {
			object_free(rgb_mat_wrapper);
			rgb_mat_wrapper = NULL;
		}
		if (depth_mat_wrapper) {
			object_free(depth_mat_wrapper);
			depth_mat_wrapper = NULL;
		}
		if (cloud_mat_wrapper) {
			object_free(cloud_mat_wrapper);
			cloud_mat_wrapper = NULL;
		}
		sysmem_freeptr(depth_map_data);
		sysmem_freeptr(rgb_map_data);
	}
	
	void depth_map(t_symbol * name) {
		`
	}
	
	void rgb_map(t_symbol * name) {
		t_jit_matrix_info in_info;
		long in_savelock;
		char * in_bp;
		t_jit_err err = 0;
		
		// get matrix from name:
		void * in_mat = jit_object_findregistered(name);
		if (!in_mat) {
			object_error(&ob, "failed to acquire matrix");
			err = JIT_ERR_INVALID_INPUT;
			goto out;
		}
		
		// lock it:
		in_savelock = (long)jit_object_method(in_mat, _jit_sym_lock, 1);
		
		// first ensure the type is correct:
		jit_object_method(in_mat, _jit_sym_getinfo, &in_info);
		jit_object_method(in_mat, _jit_sym_getdata, &in_bp);
		if (!in_bp) {
			err = JIT_ERR_INVALID_INPUT;
			goto unlock;
		}
		
		if (in_info.planecount != 2) {
			err = JIT_ERR_MISMATCH_PLANE;
			goto unlock;
		}
		
		if (in_info.type != _jit_sym_float32) {
			err = JIT_ERR_MISMATCH_TYPE;
			goto unlock;
		}
		
		if (in_info.dimcount != 2 || in_info.dim[0] != DEPTH_WIDTH || in_info.dim[1] != DEPTH_HEIGHT) {
			err = JIT_ERR_MISMATCH_DIM;
			goto unlock;
		}

		// copy matrix data into depth map:
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			// get row pointer:
			char * ip = in_bp + y*in_info.dimstride[1];
			
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				
				// convert column pointer to vec2f:
				const vec2f& v = *(vec2f *)(ip);
				float ix = (v.x);
				float iy = (v.y);
				
				// shift index by +0.5 so that (int) rounding (in cloud_process) 
				// puts it in the proper pixel center
				ix += 0.5;
				iy += 0.5;
				
				// clip at boundaries:
				ix = ix < 0 ? 0 : ix >= DEPTH_WIDTH-1 ? DEPTH_WIDTH-1 : ix;
				iy = iy < 0 ? 0 : iy >= DEPTH_HEIGHT-1 ? DEPTH_HEIGHT-1 : iy;
				
				// store:
				rgb_map_data[i].x = ix;
				rgb_map_data[i].y = iy;
				
				// move to next column:
				ip += in_info.dimstride[0];
			}
		}
		
	unlock:
		// restore matrix lock state:
		jit_object_method(in_mat, _jit_sym_lock, in_savelock);
	out:
		if (err) {
			jit_error_code(&ob, err);
		}
	}
	
	void bang() {
		if (unique) {
			if (use_rgb && new_rgb_data) {
				if (!align_rgb_to_cloud) 
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );	
				new_rgb_data = 0;			
			}
			if (new_depth_data) {
				outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
				new_depth_data = 0;
			}
			if (new_cloud_data) {
				if (use_rgb && align_rgb_to_cloud)
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_cloud_name  );
				if (transform_cloud) {
					outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, trans_cloud_name);
				} else {
					outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
				}
				new_cloud_data = 0;
			}
		} else {
			if (use_rgb) {
				if (align_rgb_to_cloud) {
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_cloud_name  );
				} else {
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
				}
			}
			outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
			if (transform_cloud) {
				outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, trans_cloud_name);
			} else {
				outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
			}
		}

	}
	
	void cloud_process() {
		float inv_depth_focal_x = 1.f/depth_focal.x;
		float inv_depth_focal_y = 1.f/depth_focal.y;
	
		// for each cell:
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				
				// remove the effects of lens distortion
				// (lookup into distortion map)
				
				// Of course this isn't optimal; actually we should be doing the reverse, 
				// i.e. converting cell index to match the distortion. 
				// But it's not trivial to invert the lens distortion.
				// Using a lookup map like this is also how OpenCV's undistort() works.
				
				vec2f di = depth_map_data[i];
				uint16_t d = depth_back[(int)(di.x) + (int)(di.y)*DEPTH_WIDTH];
				
				/*
					Using depth interpolation only makes sense if we have pre-filtered
					the depth data to remove null results (pixel too near, too far, or unknown)
				
				if (use_depth_interpolation) {
					float dxb = di.x - (int)(di.x);
					float dyb = di.y - (int)(di.y);
					float dxa = 1.-dxb;
					float dya = 1.-dyb;
					uint16_t d00 = d;
					uint16_t d10 = depth_back[(int)(di.x + 1) + (int)(di.y)*DEPTH_WIDTH];
					uint16_t d01 = depth_back[(int)(di.x) + (int)(di.y+1)*DEPTH_WIDTH];
					uint16_t d11 = depth_back[(int)(di.x + 1) + (int)(di.y+1)*DEPTH_WIDTH];
					d = (uint16_t)(
						  d00 * dxa * dya
						+ d10 * dxb * dya
						+ d01 * dxa * dyb
						+ d11 * dxb * dyb
						);
				}
				*/
				
//				if (d < 2047) {
					// convert pixel coordinate to NDC depth plane intersection
					float uv_x = (x - depth_center.x) * inv_depth_focal_x;
					float uv_y = (y - depth_center.y) * inv_depth_focal_y;
					
					// convert to meters
					float z = d * 0.001f;

					// and scale according to depth (projection)
					uv_x = uv_x * z;
					uv_y = uv_y * z;
					
					// flip for GL:
					cloud_back[i].x = uv_x;
					cloud_back[i].y = -uv_y;
					cloud_back[i].z = -z;
					
					if (transform_cloud) {
					
						float x1 = cloud_back[i].x;
						float y1 = cloud_back[i].y;
						float z1 = cloud_back[i].z;
						
						// rotate:
						float x2 = trans_rotate[0].x * x1
								 + trans_rotate[0].y * y1
								 + trans_rotate[0].z * z1;
						float y2 = trans_rotate[1].x * x1
								 + trans_rotate[1].y * y1
								 + trans_rotate[1].z * z1;
						float z2 = trans_rotate[2].x * x1
								 + trans_rotate[2].y * y1
								 + trans_rotate[2].z * z1;
						
						x2 += trans_translate.x;
						y2 += trans_translate.y;
						z2 += trans_translate.z;
								 
						trans_cloud_back[i].x = x2;
						trans_cloud_back[i].y = y2;
						trans_cloud_back[i].z = z2;
					}
					
				//} else {
//				
//					cloud_back[i].x = 0;
//					cloud_back[i].y = 0;
//					cloud_back[i].z = 0;
//				}
			}
		}
		
		new_cloud_data = 1;
	}
	
	// find a corresponding RGB color for each cloud point:
	// TODO: reduce to valid cloud points only?
	void cloud_rgb_process() {
		if (!align_rgb_to_cloud) return;
	
		// for each cell:
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				vec3f uv;
				// flip back from OpenGL:
				// move the point into the RGB camera's coordinate frame:
				float x0 =  cloud_back[i].x - rgb_translate.x;
				float y0 = -cloud_back[i].y - rgb_translate.y;
				float z0 = -cloud_back[i].z - rgb_translate.z;
				
				// rotate:
				float x1 = rgb_rotate[0].x * x0
					     + rgb_rotate[1].x * y0
					     + rgb_rotate[2].x * z0;
				float y1 = rgb_rotate[0].y * x0
					     + rgb_rotate[1].y * y0
					     + rgb_rotate[2].y * z0;
				float z1 = rgb_rotate[0].z * x0
					     + rgb_rotate[1].z * y0
					     + rgb_rotate[2].z * z0;
				
				// remove depth (location of the 3D depth point on the RGB image plane):
				float rz = 1.f/z1;
				x1 = x1 * rz;
				y1 = y1 * rz;
								
				// use this to index our pre-calculated RGB distortion map
				// the map is in the [-0.625, 0.625] range, but we want a coordinate in the appropriate range:
				// convert to [0..1] range using 0.5+ndc*0.8; then scale to pixel size:
				vec2f t;
				t.x = (0.5f + x1*0.8f) * (DEPTH_WIDTH);
				t.y = (0.5f + y1*0.8f) * (DEPTH_HEIGHT);
				
				// adjust for rounding:
				t.x += 0.5f;
				t.y += 0.5f;
				
				// ignore out of range depth points:
				if (t.x > DEPTH_WIDTH-1 || t.x < 0 || t.y > DEPTH_HEIGHT-1 || t.y < 0) {
					rgb_cloud_back[i].x = 0;
					rgb_cloud_back[i].y = 0;
					rgb_cloud_back[i].z = 0;
				
				} else {
				
					// warp texture coordinate according to the map:
					t = sample2f(rgb_map_data, t, DEPTH_WIDTH);
					
					// ignore out of range points:
					if (t.x > DEPTH_WIDTH-1 || t.x < 0 || t.y > DEPTH_HEIGHT-1 || t.y < 0) {
						rgb_cloud_back[i].x = 0;
						rgb_cloud_back[i].y = 0;
						rgb_cloud_back[i].z = 0;
					
					} else {
					
						// use it to sample the RGB view:
						sample3c(rgb_cloud_back[i], rgb_back, t, DEPTH_WIDTH);
					}
				}
			}
		}
	}
	
	void dictionary(t_symbol *s, long argc, t_atom *argv) {
		
		t_dictionary	*d = dictobj_findregistered_retain(s);
		long			numkeys = 0;
		t_symbol		**keys = NULL;
		int				i;
//		int				size;

		if (!d) {
			object_error(&ob, "unable to reference dictionary named %s", s);
			return;
		}


		dictionary_getkeys_ordered(d, &numkeys, &keys);
		for (i=0; i<numkeys; i++) {
			long	argc = 0;
			t_atom	*argv = NULL;
			t_symbol * key = keys[i];
			
			post("key %s", keys[i]->s_name);
			
			/*
			key R
			key T
			key depth_base_and_offset
			key depth_distortion
			key depth_intrinsics
			key depth_size
			key raw_depth_size
			key raw_rgb_size
			key rgb_distortion
			key rgb_intrinsics
			key rgb_size
			*/
			
			if (key == gensym("depth_base_and_offset")) {
				
			}

			/*
			
			dictionary_getatoms(d, keys[i], &argc, &argv);
			if (argc == 1 && atom_gettype(argv) == A_OBJ && object_classname(atom_getobj(argv)) == _sym_dictionary) {
				t_atom			a[2];
				t_dictionary	*d2 = (t_dictionary *) atom_getobj(argv);
				t_symbol		*d2_name = dictobj_namefromptr(d2);

				// if we have a dictionary, but it doesn't have a name,
				// then we register it so that it can be passed out and accessed by other dict objects
				// we don't want to leak this memory, however, so we track it with a linklist and clear it when we are done
				//
				// this means that operations on the output from dict.iter must be synchronous
				// the dict world generally presupposes synchronous operation, so maybe it's not that bad?

				if (!subdictionaries) {
					subdictionaries = linklist_new();
					linklist_flags(subdictionaries, OBJ_FLAG_DATA);
				}

				if (!d2_name) {
					d2 = dictobj_register(d2, &d2_name);
					linklist_append(subdictionaries, d2);
				}

				atom_setsym(a, _sym_dictionary);
				atom_setsym(a+1, d2_name);
				outlet_anything(x->outlet, keys[i], 2, a);
			}
			else
				outlet_anything(x->outlet, keys[i], argc, argv);
			*/
		}

		dictobj_release(d);
		if (keys)
			sysmem_freeptr(keys);
	}
};